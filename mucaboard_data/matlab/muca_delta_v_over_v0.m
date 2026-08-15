function [vals, skipped] = muca_delta_v_over_v0(muca_rows, depth_shallow, depth_deep, min_v0)
% MUCA_DELTA_V_OVER_V0  Per-point relative change in the mucaboard's OWN
% normalized [0,1] reading, (V_deep - V0)/V0, mirroring
% lcr_delta_c_over_c0.m's delta-C/C0 but computed from the muca board's own
% signal instead of LCR-measured pF -- gives full 19-point coverage on all
% 3 surfaces (the LCR-based map is stuck at 3 points for hollow).
%
% depth_mm=0 in the mucaboard logs is NOT a usable per-point baseline (it's
% just a handful of idle rows before the point sweep starts, with empty
% phase/point) -- so V0 uses depth_shallow (5mm, the shallowest depth with
% real per-point hold-phase data), not depth 0.
%
% min_v0 (default 0.05): points whose V0 falls below this are excluded
% (NaN), not just V0==0 -- a V0 near the sensor's noise floor makes the
% RATIO wildly unstable even though the absolute reading itself is fine
% (e.g. V0=0.003 turning a modest absolute rise into a delta-V/V0 in the
% hundreds or thousands, which swamps the color scale for every other
% point). Second output `skipped` lists [point, v0] rows that were excluded
% this way, for a console warning.
%
% muca_rows: Nx4 [point, depth_mm, mean_force_N, mean_own_cell_value] (see
% muca_stats_by_point_depth.m). Returns 19x1 (NaN where point/depth missing,
% V0 below min_v0, or V0 is exactly zero).
    if nargin < 4
        min_v0 = 0.05;
    end
    vals = nan(19, 1);
    skipped = zeros(0, 2);
    for p = 1:19
        r0 = muca_rows(muca_rows(:, 1) == p & muca_rows(:, 2) == depth_shallow, :);
        r1 = muca_rows(muca_rows(:, 1) == p & muca_rows(:, 2) == depth_deep, :);
        if isempty(r0) || isempty(r1)
            continue;
        end
        v0 = r0(1, 4);
        v1 = r1(1, 4);
        if abs(v0) < min_v0
            skipped(end + 1, :) = [p, v0]; %#ok<AGROW>
            continue;
        end
        vals(p) = (v1 - v0) / v0;
    end
end
