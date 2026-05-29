% drone_combat_sim.m
% =========================================================================
%  Autonomous Drone Combat Simulation
%
%  Algorithms:
%    1. YOLO proxy         - detects targets within sensor range + noise
%    2. Triangulation      - depth estimation noise model
%    3. Kalman Filter      - predicts & smooths each tracked object
%    4. Hungarian (Munkres)- optimal track-to-detection assignment
%    5. RRT*               - global collision-free path planning
%    6. Visual Servoing    - image-plane proportional bearing/range control
%    7. DWA                - local dynamic window obstacle avoidance
%    8. ORCA               - velocity-obstacle inter-drone avoidance
%
%  HOW TO RUN: Just press Run. All helper files must be in the same folder.
%    kalman_tracker.m  hungarian_proxy.m  rrt_raw.m  dwa_raw.m
% =========================================================================
clc; clear; close all;
rng(42);

fprintf('=========================================\n');
fprintf('   Autonomous Drone Combat Simulation\n');
fprintf('=========================================\n');

% =========================================================================
%  ENVIRONMENT
% =========================================================================
MAP     = [-5 105 -5 105]; % Expanded slightly so drones on the edge are visible

% Static obstacles [x, y] - perfectly spaced out to allow a wide, smooth corridor in the center
OBS_POS = [35 35; 
           65 65; 
           35 65; 
           65 35];
OBS_R   = 5.0;   % must match rrt_raw.m and dwa_raw.m

% =========================================================================
%  SIMULATION PARAMETERS
% =========================================================================
DT           = 0.2;
MAX_STEPS    = 600;
SENSOR_R     = 70.0;    % FIX: larger sensor range so drones detect earlier
INTERCEPT_R  =  5.0;    % capture distance
RRT_INTERVAL =  15;     % replan global path every N steps
YOLO_NOISE   =  1.0;    % detection noise std (metres)

% Visual Servoing gains
VS_KP_V = 1.0;   % range gain
VS_KP_W = 1.8;   % bearing gain

% =========================================================================
%  VIDEO RECORDER SETUP
% =========================================================================
vidWriter = VideoWriter('drone_swarm_combat.avi');
vidWriter.FrameRate = 30;
open(vidWriter);

% =========================================================================
%  FRIENDLY DRONES  (blue dots)
%  State: [x, y, theta, v, omega]
% =========================================================================
NUM_FRIENDLY = 8;

% Dynamically generate 8 starting lanes spread across the map
y_pos = linspace(8, 92, NUM_FRIENDLY);
friendly_starts = zeros(NUM_FRIENDLY, 3);
for i = 1:NUM_FRIENDLY
    friendly_starts(i,:) = [10, y_pos(i), 0];
end

friendly = struct();
for k = 1:NUM_FRIENDLY
    friendly(k).state           = [friendly_starts(k,1), friendly_starts(k,2), ...
                                    friendly_starts(k,3), 0, 0];
    friendly(k).label           = sprintf('F%d', k);
    friendly(k).trackers        = [];
    friendly(k).next_id         = 1;
    friendly(k).rrt_path        = [];
    friendly(k).target_est      = [];
    friendly(k).mode            = 'scan';
    friendly(k).vs_error        = [0 0];
    friendly(k).last_target_pos = [];   % for replan-on-move
    friendly(k).trail           = friendly_starts(k,1:2); % position history
    friendly(k).start_pos       = friendly_starts(k,1:2);
    friendly(k).revolve_angle   = 0;
    friendly(k).wait_timer      = 0;
    friendly(k).target_enemy    = 0;
end

% =========================================================================
%  ENEMY DRONES  (red squares)  - start top-right, move evasively
% =========================================================================
% Enemies start on the opposite side of the map in matching lanes
NUM_ENEMY = 8;
y_pos = linspace(8, 92, NUM_ENEMY);
enemy_starts = zeros(NUM_ENEMY, 2);
for i = 1:NUM_ENEMY
    enemy_starts(i,:) = [90, y_pos(i)];
end

enemy = struct();
for k = 1:NUM_ENEMY
    enemy(k).state = [enemy_starts(k,1), enemy_starts(k,2), ...
                      pi + rand*pi/3, 0.5, 0];
    enemy(k).label = 'Unknown';
    enemy(k).mode = 'evade';
    enemy(k).start_pos = enemy_starts(k,:);
    enemy(k).trail = enemy_starts(k,:);
end% =========================================================================
%  STATIC OBSTACLES (Grey circles)
% =========================================================================
OBS_POS = [25, 25; 25, 75; 75, 25; 75, 75; 50, 50; 50, 20; 50, 80; 30, 50; 70, 50];
OBS_R   = 3.0;

% =========================================================================
%  DECOYS  (magenta x) - stress-test the tracker / Hungarian
% =========================================================================
decoys = [];

% =========================================================================
%  DYNAMIC OBSTACLES (Yellow circles)
%  State: [x, y, vx, vy]
% =========================================================================
NUM_DYN_OBS = 6;
dyn_obs = struct();
for k = 1:NUM_DYN_OBS
    dyn_obs(k).state = [20 + 60*rand(), 20 + 60*rand(), ...
                        6*rand()-3, 6*rand()-3];
end

% =========================================================================
%  FIGURE
% =========================================================================
fig = figure('Name', 'Drone Combat Simulation', ...
             'Position', [60 60 920 920], ...
             'Color', [0.06 0.06 0.10]);

intercept_log = [];

% =========================================================================
%  MAIN LOOP
% =========================================================================
for step = 1:MAX_STEPS

    % =====================================================================
    %  STEP 0.5: Move dynamic obstacles (linear + bounce)
    % =====================================================================
    for k = 1:NUM_DYN_OBS
        dyn_obs(k).state(1) = dyn_obs(k).state(1) + dyn_obs(k).state(3) * DT;
        dyn_obs(k).state(2) = dyn_obs(k).state(2) + dyn_obs(k).state(4) * DT;
        
        % Bounce off boundaries
        if dyn_obs(k).state(1) < 2 || dyn_obs(k).state(1) > 98
            dyn_obs(k).state(3) = -dyn_obs(k).state(3);
        end
        if dyn_obs(k).state(2) < 2 || dyn_obs(k).state(2) > 98
            dyn_obs(k).state(4) = -dyn_obs(k).state(4);
        end
    end

    % Combine static obstacles and current dynamic obstacle positions
    CURRENT_OBS_POS = OBS_POS;
    for dk = 1:NUM_DYN_OBS
        CURRENT_OBS_POS = [CURRENT_OBS_POS; dyn_obs(dk).state(1:2)];
    end

    % =====================================================================
    %  STEP 1: Move enemy drones (evasive wandering + boundary clamp)
    % =====================================================================
    for k = 1:NUM_ENEMY
        if strcmp(enemy(k).mode, 'captured')
            enemy(k).state(4) = 0;
            enemy(k).state(5) = 0;
            
        elseif strcmp(enemy(k).mode, 'return')
            dir = enemy(k).start_pos - enemy(k).state(1:2);
            d = norm(dir);
            if d < 1.0
                enemy(k).state(4) = 0;
                enemy(k).state(5) = 0;
                enemy(k).mode = 'finished';
            else
                tar_ang = atan2(dir(2), dir(1));
                err = wrap_pi_sim(tar_ang - enemy(k).state(3));
                ew = min(1.5, max(-1.5, 2.0*err));
                ev = min(8.0, d); % Increased return speed from 3.0 to 8.0
                
                % Simple obstacle avoidance for returning enemies
                for j = 1:size(CURRENT_OBS_POS,1)
                    dist_obs = norm(enemy(k).state(1:2) - CURRENT_OBS_POS(j,:));
                    if dist_obs < 12.0
                        angle_to_obs = atan2(CURRENT_OBS_POS(j,2)-enemy(k).state(2), CURRENT_OBS_POS(j,1)-enemy(k).state(1));
                        obs_diff = wrap_pi_sim(angle_to_obs - enemy(k).state(3));
                        ew = -sign(obs_diff) * 2.0; % Turn sharply away
                        ev = 4.0; % Slow down to maneuver
                        break;
                    end
                end
                
                enemy(k).state(4) = ev;
                enemy(k).state(5) = ew;
                enemy(k).state(1) = enemy(k).state(1) + ev * cos(enemy(k).state(3)) * DT;
                enemy(k).state(2) = enemy(k).state(2) + ev * sin(enemy(k).state(3)) * DT;
                enemy(k).state(3) = enemy(k).state(3) + ew * DT;
            end
            
        elseif strcmp(enemy(k).mode, 'evade')
            [ev, ew] = enemy_wander(enemy(k).state, CURRENT_OBS_POS, step, k);
            enemy(k).state(4) = ev;
            enemy(k).state(5) = ew;
            enemy(k).state(1) = enemy(k).state(1) + ev * cos(enemy(k).state(3)) * DT;
            enemy(k).state(2) = enemy(k).state(2) + ev * sin(enemy(k).state(3)) * DT;
            enemy(k).state(3) = enemy(k).state(3) + ew * DT;

            % Clamp enemies to map
            enemy(k).state(1) = max(2, min(98, enemy(k).state(1)));
            enemy(k).state(2) = max(2, min(98, enemy(k).state(2)));
        end
        
        if ~strcmp(enemy(k).mode, 'finished')
            enemy(k).trail = [enemy(k).trail; enemy(k).state(1:2)];
            if size(enemy(k).trail,1) > 150
                enemy(k).trail(1,:) = [];
            end
        end
    end

    % =====================================================================
    %  SENSOR PHASE (YOLO + DeepSORT Proxy)
    % =====================================================================
    yolo_detections = [];
    for ek = 1:NUM_ENEMY
        if ~strcmp(enemy(ek).mode, 'evade'), continue; end
        % Check if within 45m of ANY friendly drone
        detected = false;
        for fi = 1:NUM_FRIENDLY
            if norm(friendly(fi).state(1:2) - enemy(ek).state(1:2)) < 45.0
                detected = true; break;
            end
        end
        if detected
            % Add YOLO Gaussian noise (e.g. 0.5m std dev)
            nx = enemy(ek).state(1) + randn() * 0.5;
            ny = enemy(ek).state(2) + randn() * 0.5;
            yolo_detections = [yolo_detections; nx, ny];
        end
    end
    
    active_tracks = deepsort_proxy(yolo_detections, DT);
    N_TRK = length(active_tracks);

    % =====================================================================
    %  GLOBAL ASSIGNMENT (Hungarian on DeepSORT Tracks)
    % =====================================================================
    cost_matrix = inf(NUM_FRIENDLY, max(1, N_TRK)); % ensure matrix is at least Nx1 to avoid crashes
    for fi = 1:NUM_FRIENDLY
        % Only assign targets to drones in combat modes
        if strcmp(friendly(fi).mode, 'revolve') || strcmp(friendly(fi).mode, 'touch') || ...
           strcmp(friendly(fi).mode, 'wait') || strcmp(friendly(fi).mode, 'return') || ...
           strcmp(friendly(fi).mode, 'finished') || strcmp(friendly(fi).mode, 'stopped')
            continue;
        end
        for ti = 1:N_TRK
            cost_matrix(fi, ti) = norm(friendly(fi).state(1:2) - [active_tracks(ti).x, active_tracks(ti).y]);
        end
    end
    
    % Reset enemy labels
    for k = 1:NUM_ENEMY
        if strcmp(enemy(k).mode, 'evade')
            enemy(k).label = 'Unknown';
        end
    end

    if N_TRK > 0
        global_assign = munkres_sim(cost_matrix);
        
        % Update labels for assigned enemies based on nearest drone
        for fi = 1:NUM_FRIENDLY
            ti = global_assign(fi);
            if ti > 0 && ti <= N_TRK
                trk_pos = [active_tracks(ti).x, active_tracks(ti).y];
                min_d = inf;
                best_k = -1;
                for ek = 1:NUM_ENEMY
                    d = norm(enemy(ek).state(1:2) - trk_pos);
                    if d < min_d
                        min_d = d;
                        best_k = ek;
                    end
                end
                if best_k > 0 && min_d < 5.0
                    enemy(best_k).label = sprintf('Target %d', fi);
                end
            end
        end
    else
        global_assign = zeros(NUM_FRIENDLY, 1);
    end

    % =====================================================================
    %  PER FRIENDLY DRONE
    % =====================================================================
    for fi = 1:NUM_FRIENDLY
        fs = friendly(fi).state;

        % -----------------------------------------------------------------
        % ALGORITHM 1 & 2: YOLO detection + Triangulation noise
        % -----------------------------------------------------------------
        all_true = [];
        for k = 1:NUM_ENEMY
            if strcmp(enemy(k).mode, 'evade')
                all_true = [all_true; enemy(k).state(1:2)];
            end
        end
        all_true = [all_true; decoys];

        detections = [];
        for i = 1:size(all_true, 1)
            dist_to = norm(fs(1:2) - all_true(i,:));
            if dist_to < SENSOR_R
                % Triangulation: noise grows with range
                sigma  = YOLO_NOISE * (1 + 0.4 * dist_to / SENSOR_R);
                noisy  = all_true(i,:) + randn(1,2) * sigma;
                detections = [detections; noisy];
            end
        end

        % -----------------------------------------------------------------
        % ALGORITHM 3: Kalman predict step
        % -----------------------------------------------------------------
        for t = 1:length(friendly(fi).trackers)
            friendly(fi).trackers(t).predict();
        end

        % -----------------------------------------------------------------
        % ALGORITHM 4: Hungarian data association + Kalman update
        % -----------------------------------------------------------------
        if ~isempty(detections)
            if isempty(friendly(fi).trackers)
                for d = 1:size(detections,1)
                    id = friendly(fi).next_id;
                    friendly(fi).trackers = [friendly(fi).trackers; ...
                        kalman_tracker(detections(d,1), detections(d,2), id)];
                    friendly(fi).next_id = id + 1;
                end
            else
                [assigns, unm_t, unm_d] = hungarian_proxy(friendly(fi).trackers, detections);

                for a = 1:size(assigns,1)
                    friendly(fi).trackers(assigns(a,1)).update(detections(assigns(a,2),:)');
                end
                for ti = unm_t'
                    friendly(fi).trackers(ti).mark_missed();
                end
                for di = unm_d'
                    id = friendly(fi).next_id;
                    friendly(fi).trackers = [friendly(fi).trackers; ...
                        kalman_tracker(detections(di,1), detections(di,2), id)];
                    friendly(fi).next_id = id + 1;
                end
            end

            % Prune dead tracks
            keep = arrayfun(@(t) ~t.should_delete(), friendly(fi).trackers);
            friendly(fi).trackers = friendly(fi).trackers(keep);
        end

        % -----------------------------------------------------------------
        % TARGET SELECTION (GLOBAL HUNGARIAN ASSIGNMENT)
        % -----------------------------------------------------------------
        if strcmp(friendly(fi).mode, 'scan') || strcmp(friendly(fi).mode, 'seek') || strcmp(friendly(fi).mode, 'pursue')
            assigned_track = global_assign(fi);
            if assigned_track > 0 && assigned_track <= N_TRK
                trk = active_tracks(assigned_track);
                friendly(fi).target_est = [trk.x, trk.y];
                friendly(fi).target_id  = trk.id;
                friendly(fi).mode = 'pursue';
            else
                friendly(fi).target_est = [];
                % Sweep forward (positive X) to search for enemies
                if fs(1) < 180
                    friendly(fi).mode = 'seek';
                else
                    friendly(fi).mode = 'scan';
                end
            end
        end

        % -----------------------------------------------------------------
        % ALGORITHM 5: RRT* global path planning
        % FIX: replan when target moves >8m from last planned position
        % -----------------------------------------------------------------
        target_moved = false;
        if ~isempty(friendly(fi).target_est) && ~isempty(friendly(fi).last_target_pos)
            if norm(friendly(fi).target_est - friendly(fi).last_target_pos) > 8
                target_moved = true;
            end
        end

        if strcmp(friendly(fi).mode, 'pursue')
            if isempty(friendly(fi).rrt_path) || ...
               mod(step, RRT_INTERVAL) == 0   || ...
               target_moved
                friendly(fi).rrt_path = rrt_raw(fs(1:2), friendly(fi).target_est, CURRENT_OBS_POS);
                friendly(fi).last_target_pos = friendly(fi).target_est;
            end
        elseif strcmp(friendly(fi).mode, 'seek')
            % Search sweep forward
            seek_goal = [140, friendly(fi).start_pos(2)];
            drone_stuck = (fs(4) < 0.5); 
            if isempty(friendly(fi).rrt_path) || mod(step, RRT_INTERVAL*2) == 0 || drone_stuck
                friendly(fi).rrt_path = rrt_raw(fs(1:2), seek_goal, CURRENT_OBS_POS);
            end
        elseif strcmp(friendly(fi).mode, 'return')
            drone_stuck = (fs(4) < 0.5);
            if isempty(friendly(fi).rrt_path) || mod(step, RRT_INTERVAL) == 0 || drone_stuck
                friendly(fi).rrt_path = rrt_raw(fs(1:2), friendly(fi).start_pos, CURRENT_OBS_POS);
            end
        else
            friendly(fi).rrt_path = [];
        end

        % -----------------------------------------------------------------
        % ALGORITHM 6: Visual Servoing - bearing/range proportional ctrl
        % Computes error in image-plane (bearing & range to waypoint),
        % generates velocity references fed into DWA as local goal.
        % -----------------------------------------------------------------
        local_goal = [];
        if ~isempty(friendly(fi).rrt_path) && size(friendly(fi).rrt_path,1) >= 1

            % Trim waypoints the drone has already passed
            while size(friendly(fi).rrt_path,1) > 1
                wp_dist = norm(fs(1:2) - friendly(fi).rrt_path(1,:));
                if wp_dist < 6.0 % FIX: Increased from 2.0 because high speed drones have wider turning radii
                    friendly(fi).rrt_path(1,:) = [];
                else
                    break;
                end
            end

            % Lookahead point (immediate next waypoint)
            la_idx = min(1, size(friendly(fi).rrt_path,1)); % FIX: Follow immediate waypoint so it doesn't clip obstacles
            wp = friendly(fi).rrt_path(la_idx,:);

            % VS error signals
            dx_wp      = wp(1) - fs(1);
            dy_wp      = wp(2) - fs(2);
            range_wp   = sqrt(dx_wp^2 + dy_wp^2) + 1e-6;
            bearing_wp = atan2(dy_wp, dx_wp);
            bear_err   = wrap_pi_sim(bearing_wp - fs(3));

            friendly(fi).vs_error = [bear_err, range_wp];

            % Proportional VS control law
            if strcmp(friendly(fi).mode, 'return')
                % Much smoother return speed so DWA doesn't overshoot waypoints and orbit
                v_vs = min(1.0 * range_wp, 8.0);
            else
                % Keep aggressive minimum speed (+5.0) for high-speed intercepts
                v_vs = min(VS_KP_V * range_wp + 5.0, 12.0); 
            end
            w_vs = max(-pi, min(pi, VS_KP_W * bear_err));

            % Project VS velocity into a goal point for DWA
            look_t     = 1.2;
            proj_theta = fs(3) + w_vs * look_t * 0.5;
            local_goal = [fs(1) + v_vs * cos(proj_theta) * look_t, ...
                          fs(2) + v_vs * sin(proj_theta) * look_t];
            local_goal(1) = max(2, min(98, local_goal(1)));
            local_goal(2) = max(2, min(98, local_goal(2)));
        end

        % -----------------------------------------------------------------
        % ALGORITHM 7 & 8: DWA + ORCA
        % -----------------------------------------------------------------
        % -----------------------------------------------------------------
        % CINEMATIC MODES & AVOIDANCE (DWA + ORCA)
        % -----------------------------------------------------------------
        if strcmp(friendly(fi).mode, 'revolve')
            ep = enemy(friendly(fi).target_enemy).state(1:2);
            friendly(fi).revolve_angle = friendly(fi).revolve_angle + (2.0 * DT / 6.0); % angular velocity = v/r = 2/4
            target_pos = ep + 6.0 * [cos(friendly(fi).revolve_angle), sin(friendly(fi).revolve_angle)];
            
            dx = target_pos(1) - fs(1); dy = target_pos(2) - fs(2);
            dist = hypot(dx, dy);
            t_ang = atan2(dy, dx);
            err = wrap_pi_sim(t_ang - fs(3));
            
            v_cmd = min(3.0, 1.5 * dist);
            w_cmd = min(1.5, max(-1.5, 2.5 * err));
            
            % Check if full 360 completed (using wait_timer as a simple accumulator for revolve time)
            friendly(fi).wait_timer = friendly(fi).wait_timer + DT;
            if friendly(fi).wait_timer > (2 * pi * 6.0 / 2.0) % distance/speed = time
                friendly(fi).mode = 'return';
                enemy(friendly(fi).target_enemy).mode = 'return';
            end
            
        elseif strcmp(friendly(fi).mode, 'touch')
            ep = enemy(friendly(fi).target_enemy).state(1:2);
            dx = ep(1) - fs(1); dy = ep(2) - fs(2);
            dist = hypot(dx, dy);
            
            if dist < 1.5
                friendly(fi).mode = 'wait';
                friendly(fi).wait_timer = 0;
                v_cmd = 0; w_cmd = 0;
            else
                t_ang = atan2(dy, dx);
                err = wrap_pi_sim(t_ang - fs(3));
                v_cmd = min(2.0, dist);
                w_cmd = min(1.5, max(-1.5, 2.0 * err));
            end
            
        elseif strcmp(friendly(fi).mode, 'wait')
            v_cmd = 0; w_cmd = 0;
            friendly(fi).wait_timer = friendly(fi).wait_timer + DT;
            if friendly(fi).wait_timer > 3.5
                friendly(fi).mode = 'return';
                enemy(friendly(fi).target_enemy).mode = 'return';
            end
            
        elseif strcmp(friendly(fi).mode, 'finished') || strcmp(friendly(fi).mode, 'stopped')
            v_cmd = 0; w_cmd = 0;
            
        elseif ~isempty(local_goal)
            % Collect all other drone states for ORCA
            other = [];
            for fj = 1:NUM_FRIENDLY
                if fj ~= fi, other = [other; friendly(fj).state]; end
            end
            for ek = 1:NUM_ENEMY
                if ~strcmp(enemy(ek).mode, 'evade'), continue; end
                other = [other; enemy(ek).state];
            end
            % Add dynamic obstacles as "other drones" to ORCA
            for dk = 1:NUM_DYN_OBS
                vx = dyn_obs(dk).state(3); vy = dyn_obs(dk).state(4);
                theta = atan2(vy, vx);
                v_mag = sqrt(vx^2 + vy^2);
                other = [other; dyn_obs(dk).state(1), dyn_obs(dk).state(2), theta, v_mag, 0];
            end
            [v_cmd, w_cmd] = dwa_raw(fs, local_goal, OBS_POS, other, v_vs);
            
            % Check if finished return
            if strcmp(friendly(fi).mode, 'return') && norm(fs(1:2) - friendly(fi).start_pos) < 4.0
                friendly(fi).mode = 'finished';
                v_cmd = 0; w_cmd = 0;
            end

        elseif strcmp(friendly(fi).mode, 'scan')
            % Spin slowly to sweep sensor
            v_cmd = 0.3;  w_cmd = pi/5;

        else
            v_cmd = 0.5;  w_cmd = 0;
        end

        % -----------------------------------------------------------------
        % KINEMATICS UPDATE
        % -----------------------------------------------------------------
        friendly(fi).state(4) = v_cmd;
        friendly(fi).state(5) = w_cmd;
        friendly(fi).state(1) = fs(1) + v_cmd * cos(fs(3)) * DT;
        friendly(fi).state(2) = fs(2) + v_cmd * sin(fs(3)) * DT;
        friendly(fi).state(3) = fs(3) + w_cmd * DT;

        % Clamp to map
        friendly(fi).state(1) = max(2, min(198, friendly(fi).state(1)));
        friendly(fi).state(2) = max(2, min(198, friendly(fi).state(2)));

        % Record trail
        friendly(fi).trail = [friendly(fi).trail; friendly(fi).state(1:2)];
        if size(friendly(fi).trail,1) > 60
            friendly(fi).trail(1,:) = [];
        end

        % -----------------------------------------------------------------
        % INTERCEPT CHECK (Transition to Cinematic Sequence)
        % -----------------------------------------------------------------
        if strcmp(friendly(fi).mode, 'pursue') || strcmp(friendly(fi).mode, 'seek')
            for ek = 1:NUM_ENEMY
                if ~strcmp(enemy(ek).mode, 'evade'), continue; end
                if norm(friendly(fi).state(1:2) - enemy(ek).state(1:2)) < INTERCEPT_R
                    enemy(ek).mode = 'captured';
                    friendly(fi).mode = 'revolve';
                    friendly(fi).target_enemy = ek;
                    
                    % Calculate start angle for perfect circle math
                    dx = friendly(fi).state(1) - enemy(ek).state(1);
                    dy = friendly(fi).state(2) - enemy(ek).state(2);
                    friendly(fi).revolve_angle = atan2(dy, dx);
                    friendly(fi).wait_timer = 0; % use this to track revolve duration
                    
                    intercept_log   = [intercept_log; step, fi, ek];
                    fprintf('Step %3d | F%d caught E%d! Starting cinematic sequence.\n', step, fi, ek);
                    break;
                end
            end
        end
    end

    % =====================================================================
    %  RENDER  (skip frames to run as fast as possible)
    % =====================================================================
    if mod(step, 4) == 0
        render_frame(fig, MAP, OBS_POS, OBS_R, friendly, enemy, decoys, dyn_obs, step);
        writeVideo(vidWriter, getframe(fig));
    end

    % End simulation when all friendlies are finished returning
    all_finished = true;
    for k = 1:NUM_FRIENDLY
        if ~strcmp(friendly(k).mode, 'finished')
            all_finished = false;
        end
    end
    if all_finished && step > 50
        fprintf('\nAll drones successfully returned home at step %d!\n', step);
        render_frame(fig, MAP, OBS_POS, OBS_R, friendly, enemy, decoys, dyn_obs, step);
        break;
    end
end

fprintf('\nSimulation complete.\n');
close(vidWriter);
fprintf('Video saved to drone_swarm_combat.avi\n');
if ~isempty(intercept_log)
    fprintf('--- Intercept log [step, friendly_id, enemy_id] ---\n');
    disp(intercept_log);
end


% =========================================================================
%  ENEMY WANDER  - sinusoidal evasion with obstacle avoidance
%  FIX: added boundary bounce so enemies stay inside map
% =========================================================================
function [v, w] = enemy_wander(state, obstacles, step, id)
    v = 1.5 + 0.5 * sin(step * 0.06 + id * 1.1);
    w = 0.5 * sin(step * 0.08 + id * 1.4);

    % Bounce off map boundaries
    x = state(1); y = state(2); theta = state(3);
    if x < 8 || x > 142 || y < 8 || y > 142
        % Steer toward centre
        to_centre = atan2(75 - y, 75 - x);
        diff = mod(to_centre - theta + pi, 2*pi) - pi;
        w = sign(diff) * pi/2;
        v = 1.5;
        return;
    end

    % Obstacle avoidance
    for j = 1:size(obstacles,1)
        d = norm(state(1:2) - obstacles(j,:));
        if d < 8
            angle_to_obs = atan2(obstacles(j,2)-state(2), obstacles(j,1)-state(1));
            diff = mod(angle_to_obs - state(3) + pi, 2*pi) - pi;
            w = -sign(diff) * pi/2;
            v = 1.2;
            return;
        end
    end
end


% =========================================================================
%  RENDER_FRAME
% =========================================================================
function render_frame(fig, map, obs, obs_r, friendly, enemy, decoys, dyn_obs, step)
    figure(fig);
    
    % Save user's zoom/pan state before clearing
    ax = gca;
    xl = ax.XLim;
    yl = ax.YLim;
    
    cla; hold on; grid on; box on;
    set(gca, 'Color',     [0.06 0.06 0.10], ...
             'XColor',    [0.85 0.85 0.85], ...
             'YColor',    [0.85 0.85 0.85], ...
             'GridColor', [0.20 0.20 0.25], ...
             'GridAlpha', 0.6, ...
             'FontSize',  10);
             
    % Restore user's zoom/pan state, or initialize to map
    if xl(2) <= 1 && yl(2) <= 1
        axis(map);
    else
        xlim(xl);
        ylim(yl);
    end
    title(sprintf('Step: %d', step), 'Color', 'w', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('X (m)', 'Color', [0.8 0.8 0.8]);
    ylabel('Y (m)', 'Color', [0.8 0.8 0.8]);

    tc = linspace(0, 2*pi, 36);

    % -- Obstacles --
    for i = 1:size(obs,1)
        fill(obs(i,1)+obs_r*cos(tc), obs(i,2)+obs_r*sin(tc), ...
             [0.40 0.40 0.45], 'EdgeColor', [0.70 0.70 0.75], 'LineWidth', 0.8);
    end

    % -- Decoys --
    if ~isempty(decoys)
        plot(decoys(:,1), decoys(:,2), 'x', 'Color', [0.9 0.3 0.9], ...
             'MarkerSize', 9, 'LineWidth', 2);
        for d = 1:size(decoys,1)
            text(decoys(d,1)+1.2, decoys(d,2)+1.2, 'decoy', ...
                 'Color', [0.85 0.35 0.85], 'FontSize', 7);
        end
    end

    % -- Dynamic Obstacles --
    for dk = 1:length(dyn_obs)
        fill(dyn_obs(dk).state(1)+obs_r*cos(tc), dyn_obs(dk).state(2)+obs_r*sin(tc), ...
             [0.8 0.8 0.2], 'EdgeColor', 'w', 'LineWidth', 1.0);
    end

    % -- Friendly drones --
    for k = 1:length(friendly)
        fs = friendly(k).state;
        fc = [0.20 0.55 1.00];   % bright blue



        % RRT* path
        if ~isempty(friendly(k).rrt_path) && size(friendly(k).rrt_path,1) > 1
            plot(friendly(k).rrt_path(:,1), friendly(k).rrt_path(:,2), ...
                 '--', 'Color', [0.25 0.90 0.90], 'LineWidth', 1.4);
        end

        % Sensor ring (very faint)
        plot(fs(1)+45*cos(tc), fs(2)+45*sin(tc), ':', ...
             'Color', [0.2 0.5 1.0 ], 'LineWidth', 0.4);

        % Drone body - circle
        tc = linspace(0, 2*pi, 20);
        fill(fs(1)+1.2*cos(tc), fs(2)+1.2*sin(tc), ...
             fc, 'EdgeColor', 'w', 'LineWidth', 1.8);

        % Heading arrow
        plot([fs(1)+1.2*cos(fs(3)), fs(1)+2.5*cos(fs(3))], ...
             [fs(2)+1.2*sin(fs(3)), fs(2)+2.5*sin(fs(3))], 'w-', 'LineWidth', 2.5);

        % Draw assignment laser
        if strcmp(friendly(k).mode, 'pursue') && ~isempty(friendly(k).target_est)
            plot([fs(1), friendly(k).target_est(1)], [fs(2), friendly(k).target_est(2)], '-', 'Color', [0.3 1.0 0.3 0.4], 'LineWidth', 0.8);
        end
        
        % Label
        text(fs(1)+2, fs(2)+2, friendly(k).label, ...
             'Color', 'w', 'FontSize', 9, 'FontWeight', 'bold');

        % Mode label
        text(fs(1)+2, fs(2)-3, friendly(k).mode, ...
             'Color', [0.5 0.9 0.5], 'FontSize', 7, 'FontAngle', 'italic');

        % Kalman track estimates (green +)
        for t = 1:length(friendly(k).trackers)
            trk = friendly(k).trackers(t);
            if ~trk.confirmed, continue; end
            pos = trk.get_position();
            plot(pos(1), pos(2), '+', 'Color', [0.3 1.0 0.3], ...
                 'MarkerSize', 11, 'LineWidth', 2.5);
            vel = trk.get_velocity();
            if norm(vel) > 0.15
                quiver(pos(1), pos(2), vel(1)*2.5, vel(2)*2.5, 0, ...
                       'Color', [0.3 1.0 0.3], 'LineWidth', 1.2);
            end
        end
    end

    % -- Enemy drones --
    for k = 1:length(enemy)
        if strcmp(enemy(k).mode, 'captured')
            % Show intercept marker
            text(enemy(k).state(1), enemy(k).state(2), 'CAPTURED', ...
                 'Color', [1 0.8 0], 'FontSize', 10, 'FontWeight', 'bold', ...
                 'HorizontalAlignment', 'center');
        end
        es = enemy(k).state;
        ec = [1.00 0.15 0.15];   % red



        % Body - circle
        if strcmp(enemy(k).mode, 'captured') || strcmp(enemy(k).mode, 'return') || strcmp(enemy(k).mode, 'finished')
            fill_color = [0.8 0.4 0.4];
        else
            fill_color = ec;
        end
        tc = linspace(0, 2*pi, 20);
        fill(es(1)+1.2*cos(tc), es(2)+1.2*sin(tc), ...
             fill_color, 'EdgeColor', 'w', 'LineWidth', 1.5);

        % Heading arrow
        plot([es(1)+1.2*cos(es(3)), es(1)+2.5*cos(es(3))], ...
             [es(2)+1.2*sin(es(3)), es(2)+2.5*sin(es(3))], 'Color', [1.0 0.65 0.2], 'LineWidth', 2.5);

        text(es(1)+2, es(2)+2, enemy(k).label, ...
             'Color', 'w', 'FontSize', 9, 'FontWeight', 'bold');
    end

    % -- Legend --
    items  = {'Static obstacle','Dynamic obstacle','Decoy','Friendly', ...
              'Enemy', 'Kalman est','RRT* path'};
    colors = {[0.4 0.4 0.45], [0.8 0.8 0.2], [0.85 0.35 0.85], [0.2 0.55 1.0], ...
              [1.0 0.15 0.15], [0.3 1.0 0.3], [0.25 0.9 0.9]};
    lx = 68; ly = 99; ldy = 4.2;
    for li = 1:length(items)
        plot(lx, ly-(li-1)*ldy, 'o', 'MarkerSize', 7, ...
             'MarkerFaceColor', colors{li}, 'MarkerEdgeColor', 'w');
        text(lx+2, ly-(li-1)*ldy, items{li}, 'Color', 'w', 'FontSize', 7.5);
    end

    drawnow;
end


% =========================================================================
%  WRAP_PI_SIM
% =========================================================================
function a = wrap_pi_sim(angle)
    a = mod(angle + pi, 2*pi) - pi;
end


