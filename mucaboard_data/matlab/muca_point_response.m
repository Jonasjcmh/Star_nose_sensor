function [vals, n_iters_used] = muca_point_response(d, point_id, depth_mm, max_round, own_cell_col)
% MUCA_POINT_RESPONSE  Full 19-cell response snapshot during ONE point's own
% press event(s) -- i.e. what every physical position reads (not just the
% pressed one) while point_id is held at depth_mm, for round_idx in
% [0, max_round-1]. Reveals cross-talk/spread from a single indentation,
% unlike the own-cell mapping (which only ever looks at the diagonal
% cell_<point>).
%
% own_cell_col (from muca_layout.m) is required here: raw CSV column
% cell_i is NOT the pad physically at point i (the board is mounted rotated
% relative to the UR5 point layout -- see Integration_2/analyze_session.py's
% UR5_TO_IDX), so vals(k) must read raw column own_cell_col(k), not column k.
%
% Returns 19x1 vals (NaN-filled if no matching rows) and n_iters_used = the
% number of distinct round_idx actually found (<= max_round).
    mask = strcmp(d.phase, 'hold') & (d.depth_mm == depth_mm) ...
         & (d.ur5_point == point_id) ...
         & (d.round_idx >= 0) & (d.round_idx <= max_round - 1);
    if any(mask)
        raw_means = mean(d.cells(mask, :), 1);   % 1x19, raw CSV column order
        vals = raw_means(own_cell_col)';          % vals(k) = raw column own_cell_col(k)
        n_iters_used = numel(unique(d.round_idx(mask)));
    else
        vals = nan(19, 1);
        n_iters_used = 0;
    end
end
