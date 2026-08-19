function rows = muca_release_hold_stats(d, point_id, own_cell_col, max_round, depth_mm)
% MUCA_RELEASE_HOLD_STATS  Per-iteration release (V0) and hold-phase reading
% for one point, computed from the ACTUAL raw samples logged in each phase
% -- this supersedes the earlier "local minimum of the whole round" /
% "local minimum of the hold phase" definitions of V0.
%
% Correct phase semantics (mucaboard_ramp_collector.py, do_indentation()):
%   'locate' -- robot has just arrived at the surface for this depth step,
%               pressing=False: the probe is NOT indenting the pad at all.
%               This is "total release" -- the true V0 baseline.
%   'press'  -- ramping down to depth_mm (transient, not used here)
%   'hold'   -- pressing=True, dwelling AT depth_mm: the actual loaded
%               measurement.
%   'retract'/'post' -- releasing back to the surface (not used here)
%
% For each round_idx r in [0, max_round-1] where point_id was pressed to
% depth_mm:
%   locate_mean/min/max/n = stats of raw V over that round's 'locate' rows
%   hold_mean/min/max/n   = stats of raw V over that round's 'hold' rows
%   delta_v               = hold_mean - locate_mean
% (mean, not median, within a round/phase -- matches how every other
% hold-phase aggregate in this codebase reduces raw samples, e.g.
% muca_stats_by_point_depth.m.)
%
% own_cell_col (from muca_layout.m): raw CSV column cell_i is NOT the pad
% physically at point i, so this reads column own_cell_col(point_id).
%
% Returns rows: Rx10
%   [round_idx, locate_mean, locate_min, locate_max, locate_n,
%    hold_mean, hold_min, hold_max, hold_n, delta_v]
% one row per round that had BOTH locate and hold data at depth_mm for
% this point (R <= max_round).
    col = own_cell_col(point_id);
    rows = zeros(0, 10);
    for r = 0:(max_round - 1)
        loc_mask = strcmp(d.phase, 'locate') & (d.depth_mm == depth_mm) ...
                 & (d.ur5_point == point_id) & (d.round_idx == r);
        hold_mask = strcmp(d.phase, 'hold') & (d.depth_mm == depth_mm) ...
                  & (d.ur5_point == point_id) & (d.round_idx == r);
        if ~any(loc_mask) || ~any(hold_mask)
            continue;
        end
        lv = d.cells(loc_mask, col);
        hv = d.cells(hold_mask, col);
        rows(end + 1, :) = [r, ...
            mean(lv), min(lv), max(lv), numel(lv), ...
            mean(hv), min(hv), max(hv), numel(hv), ...
            mean(hv) - mean(lv)]; %#ok<AGROW>
    end
end
