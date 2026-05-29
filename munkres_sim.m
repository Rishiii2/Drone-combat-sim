function row_assign = munkres_sim(cost)
% MUNKRES_SIM exact O(n^3) Hungarian algorithm
% Used by deepsort_proxy and drone_combat_sim

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
