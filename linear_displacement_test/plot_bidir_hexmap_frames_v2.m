% plot_bidir_hexmap_frames_v2.m — Bidirectional slide | hexmap + response, animated
% =========================================================================
% The bidirectional counterpart of plot_linear_hexmap_frames_v2.m, for logs
% from bidirectional_displacement.py. Animates one full ROUND TRIP on a
% time axis: the honeycomb at rest, the press down on c3, the contact
% travelling out to X, the hold there, the contact travelling back to c3,
% and the lift-off -- with the start-cell and destination-cell traces drawn
% progressively underneath, in step with the tip.
%
% Values come from bidir_slide_timeline.m (segment-level locate baseline,
% median across the 5 passes). The tip marker moves between the pads'
% CANONICAL board coordinates, at constant speed within each slide.
%
% Frame count scales with the round trip (FRAMES_PER_S of data time), so
% the long c3 <-> outer-ring segments get proportionally more frames.
%
% Output: results/bidirectional/hexmap_frames_v2/frames/<tag>_<seg>/f###.png,
% assembled by
%   ./make_slide_videos.sh -d results/bidirectional/hexmap_frames_v2 -o c3_bidir -c
% =========================================================================

clear; close all; clc;

HERE = fileparts(mfilename('fullpath'));
addpath(HERE);
addpath(fullfile(HERE, '..', 'Interdome_touch', 'matlab_analysis'));

%% ---- helpers (Octave requires these at the TOP) ------------------------
function v = dcc_at_time(T_e, tshift, t)
% The 19 cell values at an arbitrary time t on the shifted axis, linearly
% interpolated between the measured bin centres.
    v = nan(1, 19);
    x = T_e.t_centre(:) + tshift;
    for k = 1:19
        y = T_e.dcc(:, k);
        ok = isfinite(y);
        if nnz(ok) < 2, continue; end
        v(k) = interp1(x(ok), y(ok), t, 'linear', 'extrap');
    end
end

function draw_hex_frame(ax, CANON, vals, tip_xy, in_contact, vmin, vmax, cmap, hex_r, xr, yr)
% Honeycomb filled by dC/C0 plus a marker at the tip.
%
% NOTE: no axis(ax,'equal'). 'equal' makes the toolkit shrink the plot box
% to fit the data aspect, so the drawn box no longer matches the Position
% rectangle and cannot be lined up with the trace panel below. Instead the
% caller sizes the box and passes xr/yr chosen to have the SAME mm-per-
% pixel on both axes, which gives undistorted hexagons and a box whose
% left and right edges land exactly on the trace panel's.
    hold(ax, 'on');
    for k = 1:19
        [vx, vy] = hex_vertices(CANON(k, 1), CANON(k, 2), hex_r);
        fill(vx, vy, value_to_rgb(vals(k), vmin, vmax, cmap), 'Parent', ax, ...
            'EdgeColor', [0.35 0.35 0.35], 'LineWidth', 0.7);
    end
    th = linspace(0, 2 * pi, 40);
    r = 1.6;
    cx = tip_xy(1) + r * cos(th); cy = tip_xy(2) + r * sin(th);
    if in_contact
        % solid white disc: the tip is pressing
        fill(cx, cy, [1 1 1], 'Parent', ax, 'EdgeColor', [0 0 0], 'LineWidth', 1.6);
    else
        % hollow ring: the tip is clear of the surface
        plot(ax, cx, cy, '-', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.4);
    end
    xlim(ax, xr); ylim(ax, yr);
    set(ax, 'XTick', [], 'YTick', [], 'Box', 'on');
end

% ── CONFIG ───────────────────────────────────────────────────────────────
SESSIONS = { ...
    'hollow_dome_5_iterations_c3_bidirectional_allpoints_session_20260930_160646.csv', 'c3_bidir'};

T_PRE  = 6.0;    % s before the outward slide; also the axis shift, so the
                 % slide out begins at an even 6 s
T_POST = 3.0;    % s after the return hold (the lift-off)
BIN_S  = 0.25;
FRAMES_PER_S = 4;   % frames per second of DATA time (the one-way v2 used 6
                    % over 15 s; a round trip here runs 21-39 s)

START_RGB = [0.8784 0.4941 0.2353];   % SDU orange - start cell
END_RGB   = [0.4784 0.3765 0.2510];   % SDU brown  - destination cell
STAGE_RGB5 = [0.918 0.918 0.918;      % approach + press down
              0.831 0.855 0.878;      % slide out
              0.945 0.945 0.945;      % hold at X
              0.851 0.878 0.851;      % slide back
              0.945 0.945 0.945];     % hold + release
STAGE_NAMES = {'approach + press', 'slide out', 'hold', 'slide back', 'hold + release'};
DEPTH_ZERO_MM = 5.0;   % logged depth is measured 5 mm above the surface
HEX_RADIUS = 8.0 / sqrt(3);
% Layout, in pixels. Both panels share AX_L and AX_W so their boxes line
% up; the hexmap's height is then derived from the data extent so that
% mm-per-pixel matches on x and y (see draw_hex_frame).
FIG_W = 800; FIG_H = 960;
AX_L  = 95;  AX_W = 591;
TR_B  = 85;  TR_H = 185;    % trace panel
HX_B  = 350;                % hexmap bottom

% Canonical board layout, row k = point P<k>.
CANON = [ -16,   0;  -12,   7;   -8,  14;  -12,  -7;   -8,   0;
           -4,   7;    0,  14;   -8, -14;   -4,  -7;    0,   0;
            4,   7;    8,  14;    0, -14;    4,  -7;    8,   0;
           12,   7;    8, -14;   12,  -7;   16,   0];
LAB = {'a1','a2','a3','b1','b2','b3','b4','c1','c2','c3', ...
       'c4','c5','d2','d3','d4','d5','e3','e4','e5'};

% Re-render only these '<tag>_<from>_to_<to>' segments; {} = all. Settable
% from the shell because the 'clear' above wipes preset variables:
%   ONLY_SEGMENTS='c3_near_c3_to_b2' octave plot_linear_hexmap_frames_v2.m
ONLY_SEGMENTS = {};
env_only = getenv('ONLY_SEGMENTS');
if ~isempty(env_only)
    ONLY_SEGMENTS = strsplit(env_only, ',');
    fprintf('ONLY_SEGMENTS from env: %s\n', env_only);
end

% Data extent of the honeycomb, and the box height that keeps the scale
% equal on both axes.
HPAD = HEX_RADIUS * 1.2;
XR = [min(CANON(:, 1)) - HPAD, max(CANON(:, 1)) + HPAD];
YR = [min(CANON(:, 2)) - HPAD, max(CANON(:, 2)) + HPAD];
HX_H = round(AX_W * (YR(2) - YR(1)) / (XR(2) - XR(1)));

LOG_DIR     = fullfile(HERE, 'logs');
RESULTS_DIR = fullfile(HERE, 'results', 'bidirectional', 'hexmap_frames_v2');
if ~exist(RESULTS_DIR, 'dir'), mkdir(RESULTS_DIR); end

for si = 1:size(SESSIONS, 1)
    csv_name = SESSIONS{si, 1};
    tag      = SESSIONS{si, 2};
    csv_path = fullfile(LOG_DIR, csv_name);
    fprintf('\n%s\n  SESSION: %s\n%s\n', repmat('=', 1, 70), tag, repmat('=', 1, 70));
    if ~exist(csv_path, 'file')
        warning('plot_linear_hexmap_frames_v2:missing', 'skipping %s', csv_name);
        continue;
    end

    T = bidir_slide_timeline(csv_path, T_PRE, T_POST, BIN_S);

    % Shared colour scale, from 0 (matching the response figures, whose y
    % axis also starts at 0 -- the sub-zero values are the noise floor
    % either side of the press, never a real drop under load).
    allv = [];
    for e = 1:numel(T)
        v = T(e).dcc(:); allv = [allv; v(isfinite(v))];   %#ok<AGROW>
    end
    vmin = 0; vmax = max(allv);
    cmap = sdu_hex_cmap();
    % One y-scale for the trace panel across the session.
    ally = [];
    for e = 1:numel(T)
        kk = [find(strcmp(LAB, T(e).from_label), 1), find(strcmp(LAB, T(e).to_label), 1)];
        v = reshape(T(e).dcc(:, kk), [], 1);
        ally = [ally; v(isfinite(v))];   %#ok<AGROW>
    end
    ylo = 0; yhi = max(ally) * 1.10;
    fprintf('  %d segments | colour [%.3f %.3f] | trace y [%.2f %.2f]\n', ...
        numel(T), vmin, vmax, ylo, yhi);

    for e = 1:numel(T)
        seg = sprintf('%s_to_%s', T(e).from_label, T(e).to_label);
        if ~isempty(ONLY_SEGMENTS) && ...
                ~any(strcmp(ONLY_SEGMENTS, sprintf('%s_%s', tag, seg)))
            continue;
        end
        k_from = find(strcmp(LAB, T(e).from_label), 1);
        k_to   = find(strcmp(LAB, T(e).to_label), 1);
        p_from = CANON(k_from, :); p_to = CANON(k_to, :);

        tshift  = T_PRE;                       % slide out starts at an even 6 s
        pm      = T(e).marks;
        b_out   = tshift;
        b_oend  = tshift + pm.fwd_end;
        b_back  = tshift + pm.rev_start;
        b_bend  = tshift + pm.rev_end;
        b_stop  = tshift + T(e).t_stop;
        b_eng   = tshift + pm.engage;          % first contact
        b_rep   = tshift + pm.retract;         % lift-off
        bounds  = [0, b_out, b_oend, b_back, b_bend, b_stop];
        n_anim  = round(b_stop * FRAMES_PER_S) + 1;

        anim_dir = fullfile(RESULTS_DIR, 'frames', sprintf('%s_%s', tag, seg));
        if ~exist(anim_dir, 'dir'), mkdir(anim_dir); end

        tt = linspace(0, b_stop, n_anim);
        for b = 1:n_anim
            t = tt(b);

            % tip: on the start pad, out to X, held there, back to the start
            if t <= b_out
                frac = 0;
            elseif t < b_oend
                frac = (t - b_out) / (b_oend - b_out);
            elseif t <= b_back
                frac = 1;
            elseif t < b_bend
                frac = 1 - (t - b_back) / (b_bend - b_back);
            else
                frac = 0;
            end
            tip = p_from + frac * (p_to - p_from);
            in_contact = (t >= b_eng) && (t <= b_rep);

            f2 = figure('Position', [40 40 FIG_W FIG_H], 'Color', [1 1 1], 'Visible', 'off');

            % ---- honeycomb ------------------------------------------------
            axH = axes('Parent', f2, 'Units', 'pixels', ...
                'Position', [AX_L HX_B AX_W HX_H]);
            draw_hex_frame(axH, CANON, dcc_at_time(T(e), tshift, t), tip, ...
                in_contact, vmin, vmax, cmap, HEX_RADIUS, XR, YR);
            colormap(f2, cmap); caxis(axH, [vmin, vmax]);
            cb = colorbar(axH, 'Units', 'pixels', ...
                'Position', [AX_L + AX_W + 18, HX_B, 22, HX_H]);
            try, set(cb, 'FontName', 'Helvetica', 'FontSize', 10); catch, end

            % ---- response traces with a cursor ----------------------------
            axR = axes('Parent', f2, 'Units', 'pixels', ...
                'Position', [AX_L TR_B AX_W TR_H]);
            hold(axR, 'on');
            % stage shading first -- fill(), not patch(), which renders
            % nothing under Octave's gnuplot toolkit
            for sI = 1:5
                fill([bounds(sI) bounds(sI + 1) bounds(sI + 1) bounds(sI)], ...
                     [ylo ylo yhi yhi], STAGE_RGB5(sI, :), ...
                     'Parent', axR, 'EdgeColor', 'none');
            end
            for sI = 2:5
                plot(axR, [bounds(sI) bounds(sI)], [ylo yhi], '-', ...
                    'Color', [0.62 0.62 0.62], 'LineWidth', 0.8);
            end
            x = T(e).t_centre(:) + tshift;
            % PROGRESSIVE: only the part of each curve up to the cursor is
            % drawn, so the traces grow in step with the tip travelling
            % across the honeycomb above instead of being there from frame
            % one. The curve ahead of the cursor is simply not plotted.
            drawn = x <= t;
            kcs = [k_from, k_to]; rgbs = {START_RGB, END_RGB};
            for tI = 1:2
                y = T(e).dcc(:, kcs(tI));
                if nnz(drawn) >= 2
                    plot(axR, x(drawn), y(drawn), '-', ...
                        'Color', rgbs{tI}, 'LineWidth', 2.6);
                end
            end
            % cursor on top of the curves
            plot(axR, [t t], [ylo yhi], '-', 'Color', [0.10 0.10 0.10], 'LineWidth', 2.8);
            % leading dot per curve, interpolated so it tracks smoothly
            % between bin centres rather than hopping bin to bin
            for tI = 1:2
                y = T(e).dcc(:, kcs(tI)); ok = isfinite(y);
                if nnz(ok) >= 2
                    yv = interp1(x(ok), y(ok), t, 'linear', 'extrap');
                    plot(axR, t, yv, 'o', 'MarkerFaceColor', rgbs{tI}, ...
                        'MarkerEdgeColor', [1 1 1], 'MarkerSize', 10, 'LineWidth', 1.4);
                end
            end
            set(axR, 'FontName', 'Helvetica', 'FontSize', 12, 'Box', 'on', ...
                'XColor', [0 0 0], 'YColor', [0 0 0], 'XTick', 0:4:ceil(b_stop));
            xlim(axR, [0, b_stop]); ylim(axR, [ylo, yhi]);
            xlabel(axR, 'time (s)', 'FontName', 'Helvetica', 'FontSize', 13);
            ylabel(axR, '\DeltaC/C_0', 'Interpreter', 'tex', ...
                'FontName', 'Helvetica', 'FontSize', 13);
            % stage names along the top of the trace panel
            for sI = 1:5
                % the 1 s holds are too narrow for a label
                if (bounds(sI + 1) - bounds(sI)) * AX_W / b_stop < 70, continue; end
                text(axR, (bounds(sI) + bounds(sI + 1)) / 2, ylo + 0.92 * (yhi - ylo), ...
                    STAGE_NAMES{sI}, 'Color', [0.30 0.30 0.30], ...
                    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                    'FontName', 'Helvetica', 'FontSize', 10, 'Interpreter', 'none');
            end
            % which curve is which, inline
            text(axR, 0.25, ylo + 0.70 * (yhi - ylo), sprintf('start %s', T(e).from_label), ...
                'Color', START_RGB, 'FontName', 'Helvetica', 'FontSize', 11, ...
                'FontWeight', 'bold', 'Interpreter', 'none');
            text(axR, 0.25, ylo + 0.52 * (yhi - ylo), sprintf('end %s', T(e).to_label), ...
                'Color', END_RGB, 'FontName', 'Helvetica', 'FontSize', 11, ...
                'FontWeight', 'bold', 'Interpreter', 'none');

            % ---- heading on an invisible full-figure axes ------------------
            axh = axes('Parent', f2, 'Position', [0 0 1 1], 'Visible', 'off');
            if t > b_oend && t <= b_bend
                arrow = '<-';   % on the way back
            else
                arrow = '->';
            end
            text(axh, 0.45, 0.995, sprintf('%s %s %s    t = %.1f s', ...
                    T(e).from_label, arrow, T(e).to_label, t), ...
                'Units', 'normalized', 'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'top', 'FontWeight', 'bold', ...
                'FontName', 'Helvetica', 'FontSize', 15, 'Interpreter', 'none');
            text(axh, 0.45, 0.966, sprintf('speed = %g mm/s    depth = %g mm    round trip', ...
                    T(e).speed_mm_s, T(e).depth_mm - DEPTH_ZERO_MM), ...
                'Units', 'normalized', 'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'top', 'Color', [0.35 0.35 0.35], ...
                'FontName', 'Helvetica', 'FontSize', 11, 'Interpreter', 'none');

            print(f2, fullfile(anim_dir, sprintf('f%03d.png', b)), '-dpng', '-r100');
            close(f2);
        end
        fprintf('    %s: %d frames -> %s\n', seg, n_anim, anim_dir);
    end
    fprintf('  done -> %s\n', RESULTS_DIR);
end

fprintf('\nAll sessions done.\n');
fprintf(['\nTo build the videos:\n' ...
         '  ./make_slide_videos.sh -d results/bidirectional/hexmap_frames_v2 -o c3_bidir -c\n']);
