function rows = fit_by_point(matched)
% FIT_BY_POINT  Separate linear fit (Cp_pF vs muca normalized value) PER
% POINT, instead of one fit pooled across all points -- pooling mixes
% together pads with very different absolute capacitance baselines and
% different depth-sensitivity, which is why the pooled R^2 is much lower
% than the per-point R^2 tends to be.
%
% matched: Mx6 [point, depth_mm, lcr_force_N, muca_force_N, Cp_pF, muca_value]
%          (see match_lcr_muca_by_depth.m)
% Returns Px5: [point, slope, intercept, r2, n] (only points with >=2 rows).
    pts = unique(matched(:, 1));
    rows = zeros(0, 5);
    for i = 1:numel(pts)
        p = pts(i);
        sub = matched(matched(:, 1) == p, :);
        x = sub(:, 6);
        y = sub(:, 5);
        n = numel(x);
        if n < 2
            continue;
        end
        pfit = polyfit(x, y, 1);
        yfit = polyval(pfit, x);
        resid = y - yfit;
        ss_res = sum(resid .^ 2);
        ss_tot = sum((y - mean(y)) .^ 2);
        if ss_tot > 0
            r2 = 1 - ss_res / ss_tot;
        else
            r2 = NaN;
        end
        rows(end + 1, :) = [p, pfit(1), pfit(2), r2, n]; %#ok<AGROW>
    end
end
