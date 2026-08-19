function rgb = value_to_rgb(val, vmin, vmax, cmap)
% VALUE_TO_RGB  Map a scalar value to an RGB triple via a colormap matrix
% (Nx3, e.g. from viridis()/parula()/hot()). NaN -> light grey (matches
% Interdome_touch/analyze_interdome.py's '#eeeeee' NaN color).
    if isnan(val)
        rgb = [0.933, 0.933, 0.933];
        return;
    end
    if vmax <= vmin
        vmax = vmin + 1e-6;
    end
    t = (val - vmin) / (vmax - vmin);
    t = min(max(t, 0), 1);
    n = size(cmap, 1);
    idx = 1 + round(t * (n - 1));
    rgb = cmap(idx, :);
end
