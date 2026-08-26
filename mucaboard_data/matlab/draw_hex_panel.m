function draw_hex_panel(ax, ids, xy, values, cmap, vmin, vmax, r, panel_title, highlight_id, show_labels)
% DRAW_HEX_PANEL  Draw one 19-hex schematic panel into axes ax.
%   ids          : Nx1 point ids (1-19)
%   xy           : Nx2 hex-center coordinates (mm)
%   values       : Nx1 values to color by (NaN -> grey)
%   highlight_id : optional point id to outline in red (e.g. the point that
%                  was actually pressed, when `values` is a cross-talk
%                  response snapshot rather than each point's own reading)
%   show_labels  : optional, default true. false = draw the hexagons and
%                  colors only, no "P##"/value text inside each cell.
    if nargin < 10
        highlight_id = [];
    end
    if nargin < 11
        show_labels = true;
    end
    axes(ax); %#ok<LAXES>
    hold(ax, 'on');
    for k = 1:numel(ids)
        x = xy(k, 1);
        y = xy(k, 2);
        val = values(k);
        rgb = value_to_rgb(val, vmin, vmax, cmap);
        [vx, vy] = hex_vertices(x, y, r);
        if ~isempty(highlight_id) && ids(k) == highlight_id
            patch(ax, vx, vy, rgb, 'EdgeColor', [0.85 0.1 0.1], 'LineWidth', 3.5);
        else
            patch(ax, vx, vy, rgb, 'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 1);
        end
        if show_labels
            if isnan(val)
                lbl = sprintf('P%02d\nn/a', ids(k));
            else
                lbl = sprintf('P%02d\n%.3f', ids(k), val);
            end
            text(ax, x, y, lbl, 'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'middle', 'FontSize', 6.5, 'FontWeight', 'bold');
        end
    end
    axis(ax, 'equal');
    pad = r * 2;
    xlim(ax, [min(xy(:, 1)) - pad, max(xy(:, 1)) + pad]);
    ylim(ax, [min(xy(:, 2)) - pad, max(xy(:, 2)) + pad]);
    set(ax, 'XTick', [], 'YTick', []);
    box(ax, 'on');   % full rectangular frame around the panel
    title(ax, panel_title, 'FontSize', 10, 'FontWeight', 'bold');
end
