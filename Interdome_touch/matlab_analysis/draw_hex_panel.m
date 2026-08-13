function draw_hex_panel(ax, ids, xy, values, cmap, vmin, vmax, r, panel_title)
% DRAW_HEX_PANEL  Draw one 19-hex schematic panel into axes ax.
%   ids     : Nx1 point ids (1-19)
%   xy      : Nx2 hex-center coordinates (mm)
%   values  : Nx1 values to color by (NaN -> grey)
    axes(ax); %#ok<LAXES>
    hold(ax, 'on');
    for k = 1:numel(ids)
        x = xy(k, 1);
        y = xy(k, 2);
        val = values(k);
        rgb = value_to_rgb(val, vmin, vmax, cmap);
        [vx, vy] = hex_vertices(x, y, r);
        patch(ax, vx, vy, rgb, 'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 1);
        if isnan(val)
            lbl = sprintf('P%02d\nn/a', ids(k));
        else
            lbl = sprintf('P%02d\n%.3f', ids(k), val);
        end
        text(ax, x, y, lbl, 'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', 'FontSize', 6.5, 'FontWeight', 'bold');
    end
    axis(ax, 'equal');
    pad = r * 2;
    xlim(ax, [min(xy(:, 1)) - pad, max(xy(:, 1)) + pad]);
    ylim(ax, [min(xy(:, 2)) - pad, max(xy(:, 2)) + pad]);
    set(ax, 'XTick', [], 'YTick', []);
    title(ax, panel_title, 'FontSize', 9, 'FontWeight', 'bold');
end
