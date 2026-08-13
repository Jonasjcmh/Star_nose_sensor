function [labels, var_mat, median_mat] = label_stats_by_depth(point_col, kind_mask, hold_mask, depth_col, depths_mm, values)
% LABEL_STATS_BY_DEPTH  Per-label, per-depth variance and median of `values`
% over hold-phase rows matching kind_mask (e.g. point_kind == 'diagonal').
%
%   point_col : cellstr, the 'point' column (label per row)
%   kind_mask : logical, rows belonging to the point_kind of interest
%   hold_mask : logical, rows where phase == 'hold'
%   depth_col : numeric, the 'depth_mm' column
%   depths_mm : depths to break out as separate columns
%   values    : numeric column to summarize (e.g. futek_force_N)
%
% Returns:
%   labels     : unique point labels for this kind, sorted, Nx1 cellstr
%   var_mat    : N x numel(depths_mm) variance
%   median_mat : N x numel(depths_mm) median
    mask = kind_mask & hold_mask;
    labels = unique(point_col(mask));
    n = numel(labels);
    m = numel(depths_mm);

    var_mat = nan(n, m);
    median_mat = nan(n, m);
    for i = 1:n
        li_mask = mask & strcmp(point_col, labels{i});
        for j = 1:m
            lj_mask = li_mask & (depth_col == depths_mm(j));
            if any(lj_mask)
                var_mat(i, j) = var(values(lj_mask));
                median_mat(i, j) = median(values(lj_mask));
            end
        end
    end
end
