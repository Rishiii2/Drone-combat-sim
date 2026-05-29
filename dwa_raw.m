function [v_opt, omega_opt] = dwa_raw(state, local_goal, obstacles, other_drone_states, v_pref)
% DWA_RAW  Dynamic Window Approach + ORCA velocity-obstacle penalty.
%
%   Scores sampled (v, w) pairs by:
%     (a) Goal-heading + range   - Visual Servoing proxy
%     (b) Static obstacle clearance
%     (c) ORCA inter-drone avoidance
%     (d) Speed reward
%
%   INPUTS
%     state             - [x, y, theta, v, omega]
%     local_goal        - [gx, gy]  intermediate RRT* waypoint
%     obstacles         - (M x 2) static obstacle centres
%     other_drone_states- (N x 5) optional other drones for ORCA
%     v_pref            - preferred maximum speed

    if nargin < 4, other_drone_states = []; end
    if nargin < 5, v_pref = 12.0; end

    % Kinematic limits
    V_MAX     =  v_pref;      % Use preferred speed from visual servoing so it can decelerate
    V_MIN     =  0.0;
    W_MAX     =  pi/2;
    W_MIN     = -pi/2;
    A_MAX     =  8.0;      % FIX: higher acceleration
    ALPHA_MAX =  pi/2;
    DT        =  0.1;
    T_PRED    =  1.5;

    % Sampling resolution
    V_RES = 0.25;           % FIX: coarser = faster per step
    W_RES = 0.1;

    % Cost weights
    W_HEAD  = 3.0;         % FIX: stronger heading pull
    W_CLEAR = 5.0;
    W_ORCA  = 3.0;
    W_SPEED = 0.5;

    OBS_R   = 2.5; % FIX: Slightly smaller than RRT's 5.0 so it doesn't reject valid paths
    DRONE_R = 2.0;

    x = state(1); y = state(2); theta = state(3);
    v = state(4); w = state(5);

    % Dynamic window
    v_lo = max(V_MIN, v - A_MAX*DT);
    v_hi = min(V_MAX, v + A_MAX*DT);
    w_lo = max(W_MIN, w - ALPHA_MAX*DT);
    w_hi = min(W_MAX, w + ALPHA_MAX*DT);

    % --- Direct bearing to goal (FIX: was computed from endpoint) ---
    dx_goal   = local_goal(1) - x;
    dy_goal   = local_goal(2) - y;
    goal_dist = sqrt(dx_goal^2 + dy_goal^2);
    goal_bear = atan2(dy_goal, dx_goal);

    best_cost = inf;
    v_opt = 0.1; omega_opt = 0;   % FIX: default forward not stationary

    for tv = v_lo : V_RES : v_hi
        for tw = w_lo : W_RES : w_hi

            traj = sim_traj(x, y, theta, tv, tw, T_PRED, DT);

            % (a) Heading: penalise bearing error AND distance to goal
            final_theta = traj(end,3);
            bear_err    = abs(wrap_pi_dwa(goal_bear - final_theta));
            dist_end    = norm(traj(end,1:2) - local_goal);
            cost_head   = W_HEAD * (bear_err + 0.5*(dist_end/100));

            % (b) Clearance
            mc = min_clearance(traj, obstacles);
            if mc < OBS_R
                continue;   % hard reject collision trajectories
            end
            cost_clear = W_CLEAR / mc;

            % (c) ORCA
            cost_orca = 0;
            for k = 1:size(other_drone_states,1)
                op  = other_drone_states(k,1:2);
                ov  = [other_drone_states(k,4)*cos(other_drone_states(k,3)), ...
                       other_drone_states(k,4)*sin(other_drone_states(k,3))];
                cost_orca = cost_orca + orca_cost(x,y,tv,tw,theta,op,ov,DRONE_R,T_PRED,DT);
            end
            cost_orca = W_ORCA * cost_orca;

            % (d) Speed reward
            cost_speed = W_SPEED * (V_MAX - tv);

            total = cost_head + cost_clear + cost_orca + cost_speed;

            if total < best_cost
                best_cost = total;
                v_opt     = tv;
                omega_opt = tw;
            end
        end
    end

    % Safety fallback
    if isinf(best_cost)
        v_opt     = 0;
        omega_opt = sign(wrap_pi_dwa(goal_bear - theta)) * W_MAX * 0.6;
        % FIX: rotate toward goal when stuck, not arbitrary direction
    end
end

function traj = sim_traj(x, y, theta, v, w, t_total, dt)
    steps = round(t_total/dt);
    traj  = zeros(steps, 3);
    for i = 1:steps
        x     = x + v*cos(theta)*dt;
        y     = y + v*sin(theta)*dt;
        theta = theta + w*dt;
        traj(i,:) = [x, y, theta];
    end
end

function d = min_clearance(traj, obstacles)
    if isempty(obstacles), d = inf; return; end
    d = inf;
    for i = 1:size(traj,1)
        dd = min(vecnorm(obstacles - traj(i,1:2), 2, 2));
        if dd < d, d = dd; end
    end
end

function p = orca_cost(x, y, tv, tw, theta, op, ov, r, t_pred, dt)
    my_vel  = [tv*cos(theta), tv*sin(theta)];
    rel_pos = op - [x, y];
    rel_vel = my_vel - ov;
    denom   = dot(rel_vel, rel_vel) + 1e-9;
    t_ca    = max(0, min(t_pred, dot(rel_pos, rel_vel)/denom));
    closest = norm(rel_pos - rel_vel*t_ca);
    if closest < r
        p = 15 * (r - closest) / r;
    else
        p = 0;
    end
end

function a = wrap_pi_dwa(angle)
    a = mod(angle + pi, 2*pi) - pi;
end
