function [assignments, unmatched_tracks, unmatched_dets] = hungarian_proxy(tracks, detections)
% HUNGARIAN_PROXY  Optimal data association via Munkres (Hungarian) algorithm.
%   Builds Euclidean cost matrix, solves linear assignment exactly O(n^3),
%   gates by maximum distance threshold.
%
%   INPUTS
%     tracks     - array of kalman_tracker objects
%     detections - (D x 2) matrix of [x, y] positions
%
%   OUTPUTS
%     assignments       - (K x 2) [track_idx, det_idx]
%     unmatched_tracks  - track indices with no detection
%     unmatched_dets    - detection indices with no track

    GATE_DIST = 25.0;   % FIX: increased from 20 to 25 for faster-moving targets

    num_tracks = length(tracks);
    num_dets   = size(detections, 1);

    assignments      = zeros(0, 2);
    unmatched_tracks = (1:num_tracks)';
    unmatched_dets   = (1:num_dets)';

    if num_tracks == 0 || num_dets == 0
        return;
    end

    % Build cost matrix (T x D)
    cost_matrix = inf(num_tracks, num_dets);
    for i = 1:num_tracks
        pred = tracks(i).get_position();
        for j = 1:num_dets
            d = norm(pred - detections(j,:));
            if d <= GATE_DIST
                cost_matrix(i,j) = d;
            end
        end
    end

    % Solve via Munkres
    row_assign = munkres(cost_matrix);

    assigned_tracks = false(1, num_tracks);
    assigned_dets   = false(1, num_dets);

    for i = 1:num_tracks
        j = row_assign(i);
        if j > 0 && cost_matrix(i,j) <= GATE_DIST
            assignments        = [assignments; i, j];
            assigned_tracks(i) = true;
            assigned_dets(j)   = true;
        end
    end

    unmatched_tracks = find(~assigned_tracks)';
    unmatched_dets   = find(~assigned_dets)';
end

% -----------------------------------------------------------------------
% MUNKRES exact O(n^3) Hungarian algorithm
% -----------------------------------------------------------------------
function row_assign = munkres(cost)
    BIG = 1e9;
    cost(isinf(cost)) = BIG;

    [M, N] = size(cost);
    sz = max(M, N);
    C  = ones(sz, sz) * BIG;
    C(1:M, 1:N) = cost;

    C = C - min(C, [], 2);
    C = C - min(C, [], 1);

    mask    = zeros(sz);
    row_cov = false(1, sz);
    col_cov = false(1, sz);

    for i = 1:sz
        for j = 1:sz
            if C(i,j) == 0 && ~row_cov(i) && ~col_cov(j)
                mask(i,j)  = 1;
                row_cov(i) = true;
                col_cov(j) = true;
            end
        end
    end
    row_cov(:) = false; col_cov(:) = false;

    for j = 1:sz
        if any(mask(:,j) == 1), col_cov(j) = true; end
    end

    path_row = zeros(sz*sz, 1);
    path_col = zeros(sz*sz, 1);
    step = 3;

    while step ~= 7
        switch step
            case 3
                if sum(col_cov) >= sz, step = 7;
                else, step = 4; end

            case 4
                done4 = false;
                while ~done4
                    [r4, c4] = find_uncovered_zero(C, row_cov, col_cov, sz);
                    if r4 == -1
                        step = 6; done4 = true;
                    else
                        mask(r4, c4) = 2;
                        sc = find(mask(r4,:) == 1, 1);
                        if ~isempty(sc)
                            row_cov(r4) = true;
                            col_cov(sc) = false;
                        else
                            path_row(1) = r4; path_col(1) = c4;
                            step = 5; done4 = true;
                        end
                    end
                end

            case 5
                pc = 1; done5 = false;
                while ~done5
                    r = find(mask(:, path_col(pc)) == 1, 1);
                    if isempty(r), done5 = true;
                    else
                        pc = pc+1; path_row(pc) = r; path_col(pc) = path_col(pc-1);
                        c = find(mask(r,:) == 2, 1);
                        pc = pc+1; path_row(pc) = r; path_col(pc) = c;
                    end
                end
                for p = 1:pc
                    if mask(path_row(p), path_col(p)) == 1
                        mask(path_row(p), path_col(p)) = 0;
                    else
                        mask(path_row(p), path_col(p)) = 1;
                    end
                end
                mask(mask == 2) = 0;
                row_cov(:) = false; col_cov(:) = false;
                for j = 1:sz
                    if any(mask(:,j) == 1), col_cov(j) = true; end
                end
                step = 3;

            case 6
                mv = find_min_uncovered(C, row_cov, col_cov, sz);
                for i = 1:sz
                    for j = 1:sz
                        if row_cov(i),  C(i,j) = C(i,j) + mv; end
                        if ~col_cov(j), C(i,j) = C(i,j) - mv; end
                    end
                end
                step = 4;
        end
    end

    row_assign = zeros(1, M);
    for i = 1:M
        j = find(mask(i,:) == 1, 1);
        if ~isempty(j) && j <= N
            row_assign(i) = j;
        end
    end
end

function [r, c] = find_uncovered_zero(C, row_cov, col_cov, sz)
    r = -1; c = -1;
    for i = 1:sz
        if row_cov(i), continue; end
        for j = 1:sz
            if ~col_cov(j) && C(i,j) == 0
                r = i; c = j; return;
            end
        end
    end
end

function m = find_min_uncovered(C, row_cov, col_cov, sz)
    m = inf;
    for i = 1:sz
        if row_cov(i), continue; end
        for j = 1:sz
            if ~col_cov(j) && C(i,j) < m, m = C(i,j); end
        end
    end
end
