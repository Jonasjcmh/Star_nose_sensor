function cmap = get_cmap(n)
% GET_CMAP  Colormap matrix (n x 3) for the hex-schematic plots: a smooth
% green -> tan/yellow -> salmon-red diverging gradient, matching the
% reference swatch the user supplied (mucaboard_data/matlab/Screenshot
% 2026-08-13 at 17.49.26.png), sampled pixel-by-pixel and interpolated with
% interp1 (core MATLAB/Octave, no toolbox/package dependency).
    if nargin < 1
        n = 256;
    end
    stops = [
        132 158  87
        148 166  93
        174 182 106
        198 189 108
        222 197 110
        227 180  98
        221 156  87
        210 127  76
        201 112  82
        195  98  91
    ] / 255;
    m = size(stops, 1);
    x_stops = linspace(0, 1, m);
    x_query = linspace(0, 1, n);
    cmap = zeros(n, 3);
    for c = 1:3
        cmap(:, c) = interp1(x_stops, stops(:, c), x_query, 'linear')';
    end
end
