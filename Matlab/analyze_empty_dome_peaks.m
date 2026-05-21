%% analyze_empty_dome_peaks.m
% Peak capacitance analysis for the dome_empty_tuesday sessions.
%
% KEY: The sensor is mounted 120° CCW relative to the UR5 robot frame.
%      Pressing ur5_point X excites cell UR5_TO_IDX(X), NOT cell X.
%      UR5_TO_IDX is defined below.
%
% Produces four figures in empty_dome_graphics/:
%   1. hexmap_points.png       – sensor layout (P labels + cell numbers)
%   2. peak_barplots.png       – one bar chart per ur5_point (19 subplots)
%   3. combined_overview.png   – hexmap (mean peaks) + all bar charts
%   4. hexmaps_per_point.png   – activation hex map per point (reference-style)
%
% Run from the repository root or adjust LOG_DIR / OUT_DIR below.
% -------------------------------------------------------------------------

clearvars; close all; clc;

%% ── Paths ────────────────────────────────────────────────────────────────
script_dir = fileparts(mfilename('fullpath'));
repo_root  = fileparts(script_dir);
LOG_DIR    = fullfile(repo_root, 'logs');
OUT_DIR    = fullfile(repo_root, 'empty_dome_graphics');
if ~exist(OUT_DIR, 'dir'), mkdir(OUT_DIR); end

%% ── Sensor layout (POINTS_MM) — matches visualizer_2d.py ────────────────
% Row: [x_mm, y_mm], index 1..19 = cell_1..cell_19
POINTS_MM = [ ...
   -8, +14;   0, +14;  +8, +14; ...          % cells 1-3
  -12,  +7;  -4,  +7;  +4,  +7; +12, +7; ...% cells 4-7
  -16,   0;  -8,   0;   0,   0;  +8,  0; +16, 0; ... % cells 8-12
  -12,  -7;  -4,  -7;  +4,  -7; +12, -7; ...% cells 13-16
   -8, -14;   0, -14;  +8, -14  ...          % cells 17-19
];
N = 19;

%% ── UR5 → cell index mapping (1-based) ──────────────────────────────────
% ur5_point → cell index (1-based column in the CSV)
UR5_TO_CELL = zeros(1, N);
UR5_TO_CELL( 1)=17; UR5_TO_CELL( 2)=13; UR5_TO_CELL( 3)= 8;
UR5_TO_CELL( 4)=18; UR5_TO_CELL( 5)=14; UR5_TO_CELL( 6)= 9; UR5_TO_CELL( 7)= 4;
UR5_TO_CELL( 8)=19; UR5_TO_CELL( 9)=15; UR5_TO_CELL(10)=10; UR5_TO_CELL(11)= 5; UR5_TO_CELL(12)= 1;
UR5_TO_CELL(13)=16; UR5_TO_CELL(14)=11; UR5_TO_CELL(15)= 6; UR5_TO_CELL(16)= 2;
UR5_TO_CELL(17)=12; UR5_TO_CELL(18)= 7; UR5_TO_CELL(19)= 3;

% Inverse: cell index → ur5_point (for labelling)
CELL_TO_UR5 = zeros(1, N);
for pt = 1:N
    CELL_TO_UR5(UR5_TO_CELL(pt)) = pt;
end

% Visit sequence (robot presses P10 three times)
VISIT_SEQ_PT  = [10,1,2,3,7,6,5,4,8,9,10,11,12,16,15,14,13,17,18,19,10];
VISIT_SEQ_VIS = [ 1,1,1,1,1,1,1,1,1,1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3];

%% ── Colormap (matches analyze_session.py "star_nose") ───────────────────
CMAP_COLORS = [ ...
    0.165, 0.710, 0.627; ...   % #2ab5a0  teal
    0.200, 0.902, 0.400; ...   % #33e666  green
    1.000, 0.902, 0.098; ...   % #ffe619  yellow
    1.000, 0.451, 0.000; ...   % #ff7300  orange
    0.863, 0.000, 0.000  ...   % #dc0000  red
];
star_nose_cmap = make_cmap(CMAP_COLORS, 256);

% 10-colour tab10-like palette for datasets
BAR_COLORS = [ ...
    0.122, 0.471, 0.706; ...
    1.000, 0.498, 0.055; ...
    0.173, 0.627, 0.173; ...
    0.839, 0.153, 0.157; ...
    0.580, 0.404, 0.741; ...
    0.549, 0.337, 0.294; ...
    0.890, 0.467, 0.761; ...
    0.498, 0.498, 0.498; ...
    0.737, 0.741, 0.133; ...
    0.090, 0.745, 0.812  ...
];

%% ── Load datasets & extract peak values ─────────────────────────────────
files = dir(fullfile(LOG_DIR, 'dome_empty_tuesday_*.csv'));
nums  = zeros(numel(files), 1);
for k = 1:numel(files)
    tok = regexp(files(k).name, 'tuesday_(\d+)_', 'tokens');
    if ~isempty(tok), nums(k) = str2double(tok{1}{1}); end
end
[~, order] = sort(nums);
files      = files(order);
n_ds       = numel(files);
fprintf('Found %d datasets\n', n_ds);

ds_labels = cell(1, n_ds);
for di = 1:n_ds
    tok = regexp(files(di).name, 'tuesday_(\d+)_', 'tokens');
    ds_labels{di} = sprintf('DS %s', tok{1}{1});
end

% peak_matrix(pt, ds) = peak target-cell value
peak_matrix  = NaN(N, n_ds);

% all_hex_mean(cell_idx) = mean peak across all datasets & visits
all_hex_sum = zeros(N, 1);
all_hex_cnt = zeros(N, 1);

% all_visit_peaks{pt}{visit}  = [n_ds x 19] matrix of mean peaks
all_visit_peaks = cell(N, 3);   % max 3 visits (P10 visited 3×)
for k = 1:numel(all_visit_peaks), all_visit_peaks{k} = []; end

for di = 1:n_ds
    fpath = fullfile(files(di).folder, files(di).name);
    T     = readtable(fpath);

    events = extract_press_events(T, N, UR5_TO_CELL);

    % Track per-point max across visits for bar plots
    pt_max_seen = containers.Map('KeyType','int32','ValueType','double');

    for ei = 1:numel(events)
        e    = events{ei};
        pt   = e.point;
        vis  = e.visit;         % 1-based
        cell = UR5_TO_CELL(pt); % 1-based cell index

        % Max target peak for bar plot
        if isKey(pt_max_seen, int32(pt))
            pt_max_seen(int32(pt)) = max(pt_max_seen(int32(pt)), e.target_peak);
        else
            pt_max_seen(int32(pt)) = e.target_peak;
        end

        % Accumulate hex mean
        all_hex_sum(cell) = all_hex_sum(cell) + e.target_peak;
        all_hex_cnt(cell) = all_hex_cnt(cell) + 1;

        % Store 19-vector for hex-per-point plot
        if vis <= 3
            if isempty(all_visit_peaks{pt, vis})
                all_visit_peaks{pt, vis} = zeros(n_ds, N);
            end
            all_visit_peaks{pt, vis}(di, :) = e.peak19;
        end
    end

    % Fill peak_matrix
    pts_done = keys(pt_max_seen);
    for k = 1:numel(pts_done)
        pt = double(pts_done{k});
        peak_matrix(pt, di) = pt_max_seen(int32(pt));
    end
end

good = all_hex_cnt > 0;
all_hex_mean = zeros(N, 1);
all_hex_mean(good) = all_hex_sum(good) ./ all_hex_cnt(good);

fprintf('Peak matrix NaN count: %d\n', sum(isnan(peak_matrix(:))));

%% ── Figure 1: Hex layout (point labels + cell numbers) ──────────────────
fig1 = figure('Units','inches','Position',[1 1 5 5],'Visible','off','Color','w');
ax1  = axes(fig1);
draw_hex_labeled(ax1, POINTS_MM, CELL_TO_UR5, ...
                 'Sensor layout  (P = ur5\_point, c = cell col)');
exportgraphics(fig1, fullfile(OUT_DIR,'hexmap_points.png'), 'Resolution',150);
fprintf('Saved hexmap_points.png\n');
close(fig1);

%% ── Figure 2: Bar plots — one per ur5_point (visit-order) ───────────────
% Visit order (unique points)
visit_order_pts = unique_ordered(VISIT_SEQ_PT);  % [10,1,2,3,7,6,5,4,8,9,11,12,16,15,14,13,17,18,19]

fig2 = figure('Units','inches','Position',[1 1 18 14],'Visible','off','Color','w');
sgtitle('Peak capacitance per sensor point — dome\_empty\_tuesday datasets', ...
        'FontSize',12,'FontWeight','bold');

x = 1:n_ds;
for sub_i = 1:numel(visit_order_pts)
    pt   = visit_order_pts(sub_i);
    cell = UR5_TO_CELL(pt);
    ax   = subplot(4, 5, sub_i);
    vals = peak_matrix(pt, :);

    hold(ax,'on');
    for di = 1:n_ds
        bar(ax, di, vals(di), 0.7, ...
            'FaceColor', BAR_COLORS(di,:), ...
            'EdgeColor', 'white', 'LineWidth', 0.4);
    end
    hold(ax,'off');

    title(ax, sprintf('P%d  →  c%d', pt, cell), ...
          'FontSize',8,'FontWeight','bold');
    xticks(ax, x);
    xticklabels(ax, ds_labels);
    xtickangle(ax, 45);
    ax.XAxis.FontSize = 6;
    ylim(ax,[0 1.05]);
    ylabel(ax,'Peak cap.','FontSize',6);
    ax.YAxis.FontSize = 6;
    grid(ax,'on'); ax.YGrid='on'; ax.XGrid='off';
    ax.GridLineStyle='--'; ax.GridAlpha=0.4;
    ax.Box='off'; ax.Color=[0.98 0.98 0.98];

    mean_val = mean(vals,'omitnan');
    yline(ax, mean_val, ':', 'Color','#333333','LineWidth',1.0, ...
          'Label', sprintf('μ=%.3f',mean_val), ...
          'LabelHorizontalAlignment','right','FontSize',5.5);
end

% 20th subplot → legend
ax_leg = subplot(4,5,20); axis(ax_leg,'off');
leg_p  = gobjects(n_ds,1);
for di = 1:n_ds
    leg_p(di) = patch(ax_leg, NaN, NaN, BAR_COLORS(di,:));
end
legend(ax_leg, leg_p, ds_labels,'Location','northwest','FontSize',7,'Box','on');
title(ax_leg,'Datasets','FontSize',8,'FontWeight','bold');

exportgraphics(fig2, fullfile(OUT_DIR,'peak_barplots.png'), 'Resolution',150);
fprintf('Saved peak_barplots.png\n');
close(fig2);

%% ── Figure 3: Combined — hexmap + bar grid ───────────────────────────────
fig3 = figure('Units','inches','Position',[1 1 24 16],'Visible','off','Color','w');
sgtitle('Empty dome — peak capacitance analysis (Tuesday sessions)', ...
        'FontSize',13,'FontWeight','bold');

% Hex map (left)
ax_hex3 = axes(fig3,'Position',[0.01 0.08 0.18 0.82]);
draw_hexmap_data(ax_hex3, POINTS_MM, all_hex_mean, -1, ...
    sprintf('Mean peak cap. per cell\n(all datasets & visits)'), ...
    1.0, star_nose_cmap);

% Bar grid (right)
left0=0.23; w0=0.135; h0=0.185; hgap=0.036; vgap=0.055;
for sub_i = 1:numel(visit_order_pts)
    pt   = visit_order_pts(sub_i);
    cell = UR5_TO_CELL(pt);
    ri   = floor((sub_i-1)/5);
    ci   = mod(sub_i-1, 5);
    L    = left0 + ci*(w0+hgap);
    B    = 0.92  - (ri+1)*(h0+vgap) + vgap;
    ax   = axes(fig3,'Position',[L B w0 h0]); %#ok<LAXES>
    vals = peak_matrix(pt,:);
    hold(ax,'on');
    for di=1:n_ds
        bar(ax,di,vals(di),0.7,'FaceColor',BAR_COLORS(di,:),'EdgeColor','w','LineWidth',0.3);
    end
    hold(ax,'off');
    title(ax,sprintf('P%d→c%d',pt,cell),'FontSize',8,'FontWeight','bold');
    xticks(ax,x); xticklabels(ax,ds_labels); xtickangle(ax,55);
    ax.XAxis.FontSize=5.5; ylim(ax,[0 1.05]); ylabel(ax,'Peak','FontSize',5.5);
    ax.YAxis.FontSize=5.5; grid(ax,'on'); ax.YGrid='on'; ax.XGrid='off';
    ax.GridLineStyle='--'; ax.GridAlpha=0.3; ax.Box='off';
    ax.Color=[0.98 0.98 0.98];
    yline(ax,mean(vals,'omitnan'),':','Color','#333333','LineWidth',0.9, ...
          'Label',sprintf('μ=%.3f',mean(vals,'omitnan')), ...
          'LabelHorizontalAlignment','right','FontSize',5);
end

% Legend
ax_leg3=axes(fig3,'Position',[left0+4*(w0+hgap) 0.03 w0 h0]);
axis(ax_leg3,'off');
leg_p3=gobjects(n_ds,1);
for di=1:n_ds, leg_p3(di)=patch(ax_leg3,NaN,NaN,BAR_COLORS(di,:)); end
legend(ax_leg3,leg_p3,ds_labels,'Location','northwest','FontSize',7,'Box','on');
title(ax_leg3,'Datasets','FontSize',8,'FontWeight','bold');

exportgraphics(fig3, fullfile(OUT_DIR,'combined_overview.png'), 'Resolution',150);
fprintf('Saved combined_overview.png\n');
close(fig3);

%% ── Figure 4: Hex maps per point (reference style) ──────────────────────
% Build list of unique (pt, visit) pairs in VISIT_SEQUENCE order
pv_pts  = [];
pv_vis  = [];
for k = 1:numel(VISIT_SEQ_PT)
    pt  = VISIT_SEQ_PT(k);
    vis = VISIT_SEQ_VIS(k);
    if ~isempty(all_visit_peaks{pt, vis}) && ...
       ~any(pv_pts==pt & pv_vis==vis)
        pv_pts(end+1) = pt;  %#ok<AGROW>
        pv_vis(end+1) = vis; %#ok<AGROW>
    end
end

n_panels = numel(pv_pts);
n_cols4  = 5;
n_rows4  = ceil(n_panels / n_cols4);
fig4 = figure('Units','inches', ...
    'Position',[1 1 n_cols4*3.2 n_rows4*3.2],'Visible','off','Color','w');
sgtitle('Hex activation maps per point — dome\_empty\_tuesday (mean over 10 datasets)', ...
        'FontSize',11,'FontWeight','bold');

for idx = 1:n_panels
    pt   = pv_pts(idx);
    vis  = pv_vis(idx);
    data = all_visit_peaks{pt, vis};  % [n_ds x 19]
    avg  = mean(data, 1, 'omitnan');  % 1x19
    ti   = UR5_TO_CELL(pt);           % 1-based target cell
    tag  = '';
    if vis > 1, tag = sprintf(' #%d', vis); end
    n_ev = size(data,1);

    ax = subplot(n_rows4, n_cols4, idx);
    draw_hexmap_data(ax, POINTS_MM, avg(:), ti, ...
                     sprintf('P%d%s (n=%d)', pt, tag, n_ev), ...
                     1.0, star_nose_cmap);
end

exportgraphics(fig4, fullfile(OUT_DIR,'hexmaps_per_point.png'), 'Resolution',150);
fprintf('Saved hexmaps_per_point.png\n');
close(fig4);

%% ── Summary statistics ───────────────────────────────────────────────────
fprintf('\n── Peak statistics per ur5_point (target cell value) ──\n');
fprintf('%-8s %6s %8s %8s %8s %8s\n','Point','Cell','Mean','Std','Min','Max');
for pt = 1:N
    v  = peak_matrix(pt,:);
    fprintf('P%-6d c%3d  %8.4f %8.4f %8.4f %8.4f\n', ...
            pt, UR5_TO_CELL(pt), mean(v,'omitnan'), std(v,'omitnan'), ...
            min(v,[],'omitnan'), max(v,[],'omitnan'));
end
fprintf('\nAll outputs saved to:\n  %s\n', OUT_DIR);

%% ═══════════════════════════════════════════════════════════════════════
%%  LOCAL FUNCTIONS
%% ═══════════════════════════════════════════════════════════════════════

function events = extract_press_events(T, N, UR5_TO_CELL)
%EXTRACT_PRESS_EVENTS  Parse CSV table into press event structs.
    cell_vars = cell(1, N);
    for i = 1:N, cell_vars{i} = sprintf('cell_%d', i); end

    events      = {};
    in_press    = false;
    rows_buf    = {};
    pt_cur      = NaN;
    visit_cnt   = zeros(1, N);

    for ri = 1:height(T)
        row = T(ri,:);
        pressing = double(row.ur5_pressing);
        if pressing == 1
            if ~in_press
                in_press = true;
                rows_buf = {};
                pt_cur   = double(row.ur5_point);
            end
            rows_buf{end+1} = row; %#ok<AGROW>
        else
            if in_press && ~isempty(rows_buf)
                pt_i = round(pt_cur);
                if ~isnan(pt_i) && pt_i >= 1 && pt_i <= N
                    % Build [n_frames x 19] matrix
                    n_frames = numel(rows_buf);
                    arr = zeros(n_frames, N);
                    for fi = 1:n_frames
                        for ci = 1:N
                            arr(fi, ci) = double(rows_buf{fi}.(cell_vars{ci}));
                        end
                    end
                    peak19     = max(arr, [], 1);       % 1×19
                    ti         = UR5_TO_CELL(pt_i);     % 1-based
                    visit_cnt(pt_i) = visit_cnt(pt_i) + 1;
                    vis         = visit_cnt(pt_i);

                    e.point        = pt_i;
                    e.visit        = vis;
                    e.peak19       = peak19;
                    e.target_peak  = peak19(ti);
                    e.target_idx   = ti;
                    events{end+1}  = e; %#ok<AGROW>
                end
            end
            in_press = false;
            rows_buf = {};
        end
    end
end

function draw_hex_labeled(ax, pts, cell_to_ur5, ttl)
%DRAW_HEX_LABELED  Draw hex map with P-labels and cell numbers.
    hold(ax,'on'); axis(ax,'equal','off');
    r = 4.5;
    theta_deg = 0:60:300;   % orientation=0: flat-top
    tx = r * cosd(theta_deg);
    ty = r * sind(theta_deg);
    for i = 1:size(pts,1)
        xc = pts(i,1); yc = pts(i,2);
        fill(ax, xc+tx, yc+ty, [0.82 0.86 0.93], ...
             'EdgeColor',[0.46 0.47 0.53],'LineWidth',0.8);
        ur5_pt = cell_to_ur5(i);
        text(ax, xc, yc+1.3, sprintf('P%d', ur5_pt), ...
             'HorizontalAlignment','center','FontSize',7, ...
             'FontWeight','bold','Color',[0.1 0.1 0.1]);
        text(ax, xc, yc-1.5, sprintf('c%d', i), ...
             'HorizontalAlignment','center','FontSize',5,'Color',[0.33 0.33 0.33]);
    end
    hold(ax,'off');
    xlim(ax,[-22 22]); ylim(ax,[-20 20]);
    title(ax, ttl, 'FontSize',9,'FontWeight','bold');
end

function draw_hexmap_data(ax, pts, values, target_idx, ttl, vmax, cmap)
%DRAW_HEXMAP_DATA  Draw coloured hex map with optional target highlight.
    vmax = max(double(vmax), 1e-6);
    r = 4.5;
    theta_deg = 0:60:300;
    tx = r * cosd(theta_deg);
    ty = r * sind(theta_deg);
    N  = size(pts,1);
    hold(ax,'on');
    for i = 1:N
        xc = pts(i,1); yc = pts(i,2);
        v  = max(0, min(1, double(values(i)) / vmax));
        ci = max(1, min(size(cmap,1), round(v*(size(cmap,1)-1))+1));
        fc = cmap(ci,:);
        lw = 2.5;
        ec = [0.8 0.8 0.8];
        if i == target_idx
            ec = [0.86 0 0]; lw = 2.5;
        else
            lw = 0.5;
        end
        fill(ax, xc+tx, yc+ty, fc, 'EdgeColor', ec, 'LineWidth', lw);
        val = double(values(i));
        if abs(val) > 0.01
            tc = 'white';
            if v < 0.45, tc = [0.13 0.13 0.13]; end
            text(ax, xc, yc, sprintf('%.2f', val), ...
                 'HorizontalAlignment','center','VerticalAlignment','middle', ...
                 'FontSize',5,'Color',tc);
        end
    end
    hold(ax,'off');
    axis(ax,'equal','off');
    xlim(ax,[-22 22]); ylim(ax,[-20 20]);
    title(ax, ttl, 'FontSize',9,'FontWeight','bold','Interpreter','none');
    % Colorbar
    colormap(ax, cmap);
    clim(ax,[0 vmax]);
    cb = colorbar(ax,'ShrinkFactor',0.55,'Position', ...
        [ax.Position(1)+ax.Position(3)+0.005, ...
         ax.Position(2)+ax.Position(4)*0.2, 0.012, ax.Position(4)*0.6]);
    cb.FontSize = 7;
end

function cmap = make_cmap(key_colors, n)
%MAKE_CMAP  Interpolate a colormap from key_colors (k×3) to n steps.
    k   = size(key_colors,1);
    x   = linspace(0,1,k);
    xi  = linspace(0,1,n);
    cmap = zeros(n,3);
    for c = 1:3
        cmap(:,c) = interp1(x, key_colors(:,c), xi, 'linear');
    end
    cmap = max(0, min(1, cmap));
end

function out = unique_ordered(v)
%UNIQUE_ORDERED  Remove duplicates while preserving first occurrence order.
    out  = [];
    seen = false(1, max(v)+1);
    for k = 1:numel(v)
        if ~seen(v(k)+1)
            out(end+1) = v(k); %#ok<AGROW>
            seen(v(k)+1) = true;
        end
    end
end
