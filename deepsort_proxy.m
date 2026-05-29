function tracks = deepsort_proxy(detections, dt)
% DEEPSORT_PROXY Simulates a Multi-Object Tracker (MOT)
%   Matches noisy (x,y) detections to existing tracks using Hungarian algorithm.
%   Maintains basic Kalman filter states (x, y, vx, vy) for each track.
%   Spawns new tracks for unmatched detections, deletes stale tracks.

    persistent trks next_id
    
    if isempty(trks)
        trks = struct('id', {}, 'x', {}, 'y', {}, 'vx', {}, 'vy', {}, 'miss', {});
        next_id = 1;
    end
    
    % 1. Predict existing tracks
    for i = 1:length(trks)
        % disabled vel pred: trks(i).x = trks(i).x + trks(i).vx * dt;
        % disabled vel pred: trks(i).y = trks(i).y + trks(i).vy * dt;
        trks(i).miss = trks(i).miss + 1; % Assume missed until matched
    end
    
    N_dets = size(detections, 1);
    N_trks = length(trks);
    
    matched_dets = false(N_dets, 1);
    
    if N_dets > 0 && N_trks > 0
        % 2. Build cost matrix (Euclidean distance)
        cost_matrix = inf(N_trks, N_dets);
        for i = 1:N_trks
            for j = 1:N_dets
                d = norm([trks(i).x, trks(i).y] - detections(j, 1:2));
                if d < 30.0 % Gating distance (only match if within 15 meters)
                    cost_matrix(i, j) = d;
                end
            end
        end
        
        % 3. Hungarian matching
        % munkres_sim returns assignment array of size max(N_trks, N_dets)
        % where index is row (track), value is col (detection).
        assignment = munkres_sim(cost_matrix);
        
        % 4. Update matched tracks
        for i = 1:N_trks
            if i <= length(assignment)
                j = assignment(i);
                if j > 0 && cost_matrix(i, j) ~= inf
                    % Update with detection (Simple alpha-beta filter proxy)
                    alpha = 0.6;
                    beta = 0.4;
                    
                    meas_x = detections(j, 1);
                    meas_y = detections(j, 2);
                    
                    % Update velocity
                    trks(i).vx = trks(i).vx + beta * ((meas_x - trks(i).x) / dt);
                    trks(i).vy = trks(i).vy + beta * ((meas_y - trks(i).y) / dt);
                    
                    % Update position
                    trks(i).x = trks(i).x + alpha * (meas_x - trks(i).x);
                    trks(i).y = trks(i).y + alpha * (meas_y - trks(i).y);
                    
                    trks(i).miss = 0; % Reset miss counter
                    matched_dets(j) = true;
                end
            end
        end
    end
    
    % 5. Spawn new tracks for unmatched detections
    for j = 1:N_dets
        if ~matched_dets(j)
            new_trk.id = next_id;
            next_id = next_id + 1;
            new_trk.x = detections(j, 1);
            new_trk.y = detections(j, 2);
            new_trk.vx = 0;
            new_trk.vy = 0;
            new_trk.miss = 0;
            trks(end+1) = new_trk;
        end
    end
    
    % 6. Delete stale tracks (missed for > 10 frames)
    keep = [trks.miss] < 10;
    trks = trks(keep);
    
    tracks = trks;
end
