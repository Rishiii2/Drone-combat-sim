function path = rrt_raw(start_pos, goal_pos, obstacles)
% RRT_RAW  RRT* (Rapidly-exploring Random Tree Star)
%   Asymptotically optimal path from start_pos to goal_pos.
%   Rewires neighbours through cheapest collision-free parent.
%   Post-processes with shortcutting for clean waypoints.

    MAX_ITER   = 4000;      % FIX: huge iterations for massive map
    STEP_SIZE  = 15.0;      % FIX: larger step for 200x200 map
    OBS_RADIUS = 4.5; % Increased to route waypoints further from obstacles      
    GOAL_R     = STEP_SIZE;
    MAP        = [0, 100, 0, 100];
    GOAL_BIAS  = 0.10;      % FIX: reduced bias so it explores around obstacles more
    REWIRE_R   = STEP_SIZE * 2.0;

    % Tree: [x, y, parent_idx, cost]
    nodes = [start_pos, 0, 0.0];

    path_found    = false;
    goal_node_idx = -1;

    for iter = 1:MAX_ITER
        % 1. Sample
        if rand < GOAL_BIAS
            q_rand = goal_pos;
        else
            q_rand = [rand*(MAP(2)-MAP(1))+MAP(1), rand*(MAP(4)-MAP(3))+MAP(3)];
        end

        % 2. Nearest node
        dists = vecnorm(nodes(:,1:2) - q_rand, 2, 2);
        [~, nearest_idx] = min(dists);
        q_near = nodes(nearest_idx, 1:2);

        % 3. Steer
        dir = q_rand - q_near;
        d   = norm(dir);
        if d < 1e-6, continue; end
        q_new = q_near + (dir/d) * min(STEP_SIZE, d);

        % Clamp to map
        q_new(1) = max(MAP(1)+1, min(MAP(2)-1, q_new(1)));
        q_new(2) = max(MAP(3)+1, min(MAP(4)-1, q_new(2)));

        % 4. Collision check
        if edge_collides(q_near, q_new, obstacles, OBS_RADIUS), continue; end

        % 5. Find neighbours
        dists_new = vecnorm(nodes(:,1:2) - q_new, 2, 2);
        near_idxs = find(dists_new <= REWIRE_R);

        % 6. Best parent (RRT* core)
        best_parent = nearest_idx;
        best_cost   = nodes(nearest_idx, 4) + norm(q_new - q_near);

        for ni = near_idxs'
            cc = nodes(ni, 4) + norm(q_new - nodes(ni, 1:2));
            if cc < best_cost && ~edge_collides(nodes(ni,1:2), q_new, obstacles, OBS_RADIUS)
                best_cost   = cc;
                best_parent = ni;
            end
        end

        % 7. Add node
        nodes = [nodes; q_new, best_parent, best_cost];
        new_idx = size(nodes, 1);

        % 8. Rewire (RRT* core)
        for ni = near_idxs'
            rc = best_cost + norm(nodes(ni,1:2) - q_new);
            if rc < nodes(ni,4) && ~edge_collides(q_new, nodes(ni,1:2), obstacles, OBS_RADIUS)
                nodes(ni,3) = new_idx;
                nodes(ni,4) = rc;
            end
        end

        % 9. Goal check
        if norm(q_new - goal_pos) <= GOAL_R
            path_found    = true;
            goal_node_idx = new_idx;
            break;
        end
    end

    % 10. Reconstruct
    if path_found
        path = [];
        idx  = goal_node_idx;
        while idx ~= 0
            path = [nodes(idx,1:2); path];
            idx  = nodes(idx,3);
        end
        path = [path; goal_pos];
        path = smooth_path(path, obstacles, OBS_RADIUS);
    else
        % Fallback: straight line, DWA handles local avoidance
        path = [start_pos; goal_pos];
    end
end

function hit = edge_collides(a, b, obstacles, radius)
    if isempty(obstacles), hit = false; return; end
    n   = max(4, ceil(norm(b-a) / (radius*0.4)));
    ts  = linspace(0, 1, n);
    hit = false;
    
    start_dists = vecnorm(obstacles - a, 2, 2);
    
    for t = ts
        pt = a + t*(b-a);
        dists = vecnorm(obstacles - pt, 2, 2);
        
        for i = 1:length(dists)
            if dists(i) < radius
                % If inside radius, it's a hit UNLESS it's strictly escaping
                if dists(i) < start_dists(i) - 1e-4
                    hit = true; return;
                end
            end
        end
    end
end

function spath = smooth_path(path, obstacles, radius)
    if size(path,1) <= 2, spath = path; return; end
    spath = path(1,:);
    i = 1;
    while i < size(path,1)
        j = size(path,1);
        while j > i+1
            if ~edge_collides(path(i,:), path(j,:), obstacles, radius), break; end
            j = j - 1;
        end
        spath = [spath; path(j,:)];
        i = j;
    end
end
