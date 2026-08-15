function h = scatter_safe(ax, x, y, sz, rgb, alpha_val)
% SCATTER_SAFE  scatter() with MarkerFaceAlpha, falling back to opaque
% markers if the graphics backend doesn't support it (e.g. Octave's
% gnuplot toolkit in headless/no-qt environments).
    try
        h = scatter(ax, x, y, sz, rgb, 'filled', 'MarkerFaceAlpha', alpha_val);
    catch
        h = scatter(ax, x, y, sz, rgb, 'filled');
    end
end
