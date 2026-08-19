function v = simple_percentile(x, p)
% SIMPLE_PERCENTILE  Linear-interpolation percentile of vector x at
% percentage p (0-100), with no dependency on the Statistics toolbox
% (MATLAB) or the 'statistics' forge package (Octave) -- quantile()/
% prctile() aren't guaranteed available in either without them.
    x = sort(x(:));
    n = numel(x);
    if n == 0
        v = NaN;
        return;
    end
    if n == 1
        v = x(1);
        return;
    end
    rank = (p / 100) * (n - 1) + 1;   % 1-based fractional rank
    lo = max(1, min(n, floor(rank)));
    hi = max(1, min(n, ceil(rank)));
    frac = rank - floor(rank);
    v = x(lo) + frac * (x(hi) - x(lo));
end
