function draw_phase_shading(ax, t_press_end, t_hold_end, t_total_end, y_top)
% DRAW_PHASE_SHADING  Light neutral background bands marking the press /
% hold / retract regions on a force-vs-time axes, drawn behind the data
% (call this BEFORE plotting the lines/fill). y_top sets the band height
% (e.g. the axes' intended y-limit max); bands start at y=0.
    if any(isnan([t_press_end, t_hold_end, t_total_end]))
        return;   % no phase timing available -- skip shading rather than guess
    end
    bands = [0, t_press_end; t_press_end, t_hold_end; t_hold_end, t_total_end];
    shades = [0.90 0.90 0.90; 0.96 0.96 0.96; 0.90 0.90 0.90];
    labels = {'press', 'hold', 'retract'};
    for i = 1:3
        x0 = bands(i, 1);
        x1 = bands(i, 2);
        if x1 <= x0
            continue;
        end
        patch(ax, [x0 x1 x1 x0], [0 0 y_top y_top], shades(i, :), ...
            'EdgeColor', 'none', 'HandleVisibility', 'off');
        text(ax, (x0 + x1) / 2, y_top * 0.97, labels{i}, ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
            'FontName', 'Helvetica', 'FontSize', 8, 'Color', [0.5 0.5 0.5]);
    end
end
