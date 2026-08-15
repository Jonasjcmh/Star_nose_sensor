function rows = muca_stats_by_point_depth(d, own_cell_col, max_round)
% MUCA_STATS_BY_POINT_DEPTH  Hold-phase mean load_cell_N (raw, unzeroed --
% to match the LCR side's convention) and mean own-cell normalized reading,
% grouped by (point, depth_mm), pooling round_idx 0..max_round-1.
% Returns an Nx4 matrix: [point, depth_mm, mean_force_N, mean_own_cell_value].
    mask = strcmp(d.phase, 'hold') & (d.round_idx >= 0) & (d.round_idx <= max_round - 1);
    pt   = d.ur5_point(mask);
    dep  = d.depth_mm(mask);
    fc   = d.load_cell_N(mask);
    cells = d.cells(mask, :);

    upts  = unique(pt(~isnan(pt)));
    udeps = unique(dep(~isnan(dep)));

    rows = zeros(0, 4);
    for i = 1:numel(upts)
        p = upts(i);
        if p < 1 || p > 19 || p ~= round(p)
            continue;
        end
        col = own_cell_col(p);
        for j = 1:numel(udeps)
            m = (pt == p) & (dep == udeps(j));
            if any(m)
                rows(end + 1, :) = [p, udeps(j), mean(fc(m)), mean(cells(m, col))]; %#ok<AGROW>
            end
        end
    end
end
