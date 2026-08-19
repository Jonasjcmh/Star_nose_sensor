function [delta_pf, delta_over_c0] = lcr_delta_c_over_c0(lcr_rows)
% LCR_DELTA_C_OVER_C0  Per-point capacitance swing across the LCR depth
% sweep, both as an absolute delta (picoFarads) and as a delta-over-baseline
% ratio, where the baseline C0 is defined as the MINIMUM Cp_pF value
% observed for that point across ALL measured depths on that surface (not
% tied to a specific depth label like "shallowest") -- this is the more
% robust definition since Cp vs depth is not perfectly monotonic for every
% point. The corresponding "high" value is the MAXIMUM Cp_pF observed for
% that point, so delta_pf and delta_over_c0 are both >= 0 by construction.
%
% delta_pf        = max(Cp) - min(Cp)                 [picoFarads]
% delta_over_c0   = (max(Cp) - min(Cp)) / min(Cp)      [unitless ratio]
%
% This is a standard capacitive-sensing normalization (delta-C over
% baseline-C) that corrects for each pad's very different absolute
% self-capacitance baseline (the main reason the pooled raw-Cp fit across
% points has a low R^2 -- see fit_by_point.m).
%
% lcr_rows: Nx4 [point, depth_mm, mean_force_N, mean_Cp_pF] (see
% lcr_stats_by_point_depth.m). Both outputs are 19x1 (NaN where point not
% present, or where C0 is exactly zero for delta_over_c0).
    delta_pf = nan(19, 1);
    delta_over_c0 = nan(19, 1);
    pts = unique(lcr_rows(:, 1));
    for i = 1:numel(pts)
        p = pts(i);
        if p < 1 || p > 19 || p ~= round(p)
            continue;
        end
        rows_p = lcr_rows(lcr_rows(:, 1) == p, :);
        c0 = min(rows_p(:, 4));
        c1 = max(rows_p(:, 4));
        delta_pf(p) = c1 - c0;
        if c0 ~= 0
            delta_over_c0(p) = (c1 - c0) / c0;
        end
    end
end
