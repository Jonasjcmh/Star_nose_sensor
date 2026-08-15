function [vals, n_iters_used] = muca_point_response_median(d, point_id, depth_mm, max_round, own_cell_col)
% MUCA_POINT_RESPONSE_MEDIAN  Same cross-talk snapshot as
% muca_point_response.m (full 19-cell response while point_id is pressed),
% but aggregated differently:
%
%   1. Restrict to hold-phase rows of point_id's own press, same as before.
%   2. For EACH iteration (round_idx) SEPARATELY, average every raw cell
%      column over that round's hold-phase rows -> one 19-value vector per
%      round (round_means).
%   3. Take the MEDIAN across rounds, per cell -- not one big pooled mean
%      over every row from every round at once.
%
% This is more robust to a single noisy/outlier press dragging the average:
% each iteration counts as exactly one data point going into the median,
% regardless of how many hold-phase samples it happened to contain.
%
% own_cell_col (from muca_layout.m): raw CSV column cell_i is NOT the pad
% physically at point i (board mounted rotated vs. the UR5 point layout --
% see Integration_2/analyze_session.py's UR5_TO_IDX), so vals(k) must read
% raw column own_cell_col(k), not column k.
%
% Returns 19x1 vals (NaN-filled if no matching rows) and n_iters_used = the
% number of rounds that actually had hold-phase data (<= max_round).
    round_means = nan(max_round, 19);   % row r+1 = per-column mean for round_idx r
    found = false(max_round, 1);
    for r = 0:(max_round - 1)
        mask = strcmp(d.phase, 'hold') & (d.depth_mm == depth_mm) ...
             & (d.ur5_point == point_id) & (d.round_idx == r);
        if any(mask)
            round_means(r + 1, :) = mean(d.cells(mask, :), 1);
            found(r + 1) = true;
        end
    end
    n_iters_used = sum(found);
    if n_iters_used == 0
        vals = nan(19, 1);
        return;
    end
    raw_median = median(round_means(found, :), 1);   % 1x19, median across the found rounds
    vals = raw_median(own_cell_col)';
end
