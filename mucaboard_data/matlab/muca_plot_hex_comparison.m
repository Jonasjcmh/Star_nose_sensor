function fig = muca_plot_hex_comparison(values, cb_label, title_str, highlight_id, clim)
% MUCA_PLOT_HEX_COMPARISON  Draw the standard 3-panel (flat/solid/hollow)
% 19-cell hex comparison for ANY values you hand it. This is the exact
% plotting machinery every hex figure in this repo uses (draw_hex_panel.m
% + get_cmap.m + muca_layout.m), pulled out on its own so you can plug in
% whatever you computed yourself -- this function does no math at all.
%
% Inputs
%   values    : struct with fields .flat, .solid, .hollow, each a 19x1
%               vector (values.(surface)(p) = the number to color point p
%               with). Use NaN for a point you have no value for -- vectors
%               must stay length 19, don't shrink them. A field can also
%               be omitted/empty to skip that surface's panel entirely.
%   cb_label  : colorbar label string, e.g. 'V0 (release), mean of my own samples'
%   title_str : figure title
%   highlight_id : optional point id to outline in red (e.g. the point
%               that was pressed, for a cross-talk-style plot). Omit or
%               pass [] for none.
%   clim      : optional [vmin vmax] to FIX the color scale yourself.
%               Omit or pass [] to auto-scale to the min/max of the data
%               you handed in (the old/default behavior). Fixing this is
%               what you want when comparing several figures against each
%               other (e.g. V0 vs Press, or P3 vs P10 vs P14) on the same
%               scale, or to stop one outlier point from stretching the
%               whole colormap. Values outside [vmin, vmax] are clamped
%               to the nearest end color (see value_to_rgb.m), not
%               clipped to grey.
%
% Returns the figure handle. Does NOT save to disk -- call
% save_fig(fig, out_path_no_ext) yourself if you want PNG/SVG output.
%
% Example (auto-scaled, default)
%   v0.flat = my_flat_v0_19x1; v0.solid = my_solid_v0_19x1; v0.hollow = my_hollow_v0_19x1;
%   fig = muca_plot_hex_comparison(v0, 'V0 (mine)', 'My V0 comparison');
%   save_fig(fig, fullfile(RESULTS_DIR, 'my_v0_hexmap'));
%
% Example (fixed scale, e.g. to match another figure or crop out an outlier)
%   fig = muca_plot_hex_comparison(v0, 'V0 (mine)', 'My V0 comparison', [], [0 0.5]);
    if nargin < 4
        highlight_id = [];
    end
    if nargin < 5
        clim = [];
    end
    [point_ids, point_xy, ~] = muca_layout();
    HEX_RADIUS = 8.0 / sqrt(3);
    SURFACE_NAMES = {'flat', 'solid', 'hollow'};

    present = false(1, 3);
    vecs = cell(1, 3);
    for s = 1:3
        sname = SURFACE_NAMES{s};
        if isfield(values, sname) && ~isempty(values.(sname))
            v = values.(sname);
            if numel(v) ~= 19
                error('muca_plot_hex_comparison:badsize', ...
                    'values.%s has %d elements, expected 19.', sname, numel(v));
            end
            vecs{s} = v(:);
            present(s) = true;
        end
    end
    if ~any(present)
        error('muca_plot_hex_comparison:novalues', 'No surfaces provided in values.');
    end

    if isempty(clim)
        all_vals = cat(1, vecs{present});
        all_vals = all_vals(~isnan(all_vals));
        if isempty(all_vals)
            error('muca_plot_hex_comparison:allnan', 'All provided values are NaN -- nothing to plot.');
        end
        vmin = min(all_vals);
        vmax = max(all_vals);
    else
        if numel(clim) ~= 2 || clim(2) <= clim(1)
            error('muca_plot_hex_comparison:badclim', 'clim must be [vmin vmax] with vmax > vmin.');
        end
        vmin = clim(1);
        vmax = clim(2);
    end
    cmap = get_cmap();

    n_panels = sum(present);
    panel_slots = find(present);
    panel_left_all  = [0.03, 0.35, 0.67];
    panel_width = 0.27;

    fig = figure('Position', [100 100 500 * n_panels + 100, 550]);
    axes_list = cell(1, n_panels);
    for i = 1:n_panels
        s = panel_slots(i);
        ax = axes('Parent', fig, 'Position', [panel_left_all(min(i,3)), 0.12, panel_width, 0.72]);
        axes_list{i} = ax;
        draw_hex_panel(ax, point_ids, point_xy, vecs{s}, cmap, vmin, vmax, ...
            HEX_RADIUS, upper(SURFACE_NAMES{s}), highlight_id);
    end
    colormap(fig, cmap);
    for i = 1:n_panels
        set(axes_list{i}, 'CLim', [vmin, vmax]);
    end
    cb = colorbar(axes_list{end}, 'Position', [0.96, 0.12, 0.02, 0.72]);
    ylabel(cb, cb_label);
    for i = 1:n_panels
        set(axes_list{i}, 'Position', [panel_left_all(min(i,3)), 0.12, panel_width, 0.72]);
    end
    fig_suptitle(fig, title_str);
end
