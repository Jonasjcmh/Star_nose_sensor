function vals = muca_hold_means(d, depth_mm, max_round, own_cell_col)
% MUCA_HOLD_MEANS  Per-point mean of the point's own capacitive cell during
% the 'hold' phase, at depth_mm, over round_idx in [0, max_round-1].
% Returns 19x1 (NaN where a point has no matching rows).
    vals = nan(19, 1);
    hold_mask = strcmp(d.phase, 'hold') & (d.depth_mm == depth_mm) ...
              & (d.round_idx >= 0) & (d.round_idx <= max_round - 1);
    for pt = 1:19
        pt_mask = hold_mask & (d.ur5_point == pt);
        if any(pt_mask)
            vals(pt) = mean(d.cells(pt_mask, own_cell_col(pt)));
        end
    end
end
