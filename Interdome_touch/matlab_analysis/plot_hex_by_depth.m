function plot_hex_by_depth(stats, ids, xy, depths_mm, r, out_path_no_ext, suptitle_str, unit_label, cmap)
% PLOT_HEX_BY_DEPTH  One 19-hex schematic panel per depth, plus an
% all-depths-mean summary panel, colored on a shared scale, saved (PNG+SVG)
% to out_path_no_ext. stats is 19 x numel(depths_mm) (rows follow ids order).
    n_depths = numel(depths_mm);
    n_panels = n_depths + 1;
    ncols = min(3, n_panels);
    nrows = ceil(n_panels / ncols);

    all_vals = stats(~isnan(stats));
    if isempty(all_vals)
        fprintf('  [warn] no data for "%s" -- skipping plot\n', suptitle_str);
        return;
    end
    vmin = min(all_vals);
    vmax = max(all_vals);

    fig = figure('Position', [100 100 ncols * 380 nrows * 380]);
    axes_list = cell(1, n_panels);
    for di = 1:n_depths
        ax = subplot(nrows, ncols, di);
        axes_list{di} = ax;
        draw_hex_panel(ax, ids, xy, stats(:, di), cmap, vmin, vmax, r, sprintf('%.1f mm', depths_mm(di)));
    end

    summary = nan(numel(ids), 1);
    for i = 1:numel(ids)
        row = stats(i, :);
        row = row(~isnan(row));
        if ~isempty(row)
            summary(i) = mean(row);
        end
    end
    ax = subplot(nrows, ncols, n_panels);
    axes_list{n_panels} = ax;
    draw_hex_panel(ax, ids, xy, summary, cmap, vmin, vmax, r, 'All depths (mean)');

    colormap(fig, cmap);
    for i = 1:n_panels
        set(axes_list{i}, 'CLim', [vmin, vmax]);
    end
    cb = colorbar(axes_list{n_panels});
    ylabel(cb, unit_label);

    fig_suptitle(fig, suptitle_str);
    save_fig(fig, out_path_no_ext);
end
