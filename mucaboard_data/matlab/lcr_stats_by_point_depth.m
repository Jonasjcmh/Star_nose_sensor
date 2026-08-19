function rows = lcr_stats_by_point_depth(d)
% LCR_STATS_BY_POINT_DEPTH  Hold-phase mean load_cell_N and Cp_pF, grouped
% by (point, depth_mm). Returns an Nx4 matrix: [point, depth_mm, mean_force_N, mean_Cp_pF].
    mask = strcmp(d.phase, 'hold');
    pt  = d.point(mask);
    dep = d.depth_mm(mask);
    fc  = d.load_cell_N(mask);
    cp  = d.Cp_pF(mask);

    upts  = unique(pt(~isnan(pt)));
    udeps = unique(dep(~isnan(dep)));

    rows = zeros(0, 4);
    for i = 1:numel(upts)
        for j = 1:numel(udeps)
            m = (pt == upts(i)) & (dep == udeps(j));
            if any(m)
                rows(end + 1, :) = [upts(i), udeps(j), mean(fc(m)), mean(cp(m))]; %#ok<AGROW>
            end
        end
    end
end
