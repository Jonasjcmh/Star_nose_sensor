% plot_linear_hexmap_frames_v2.m — Linear slide | hexmap + response, animated
% =========================================================================
% Version 2 of the travelling-contact animation. Two differences from
% plot_linear_hexmap_frames.m, which is left untouched:
%
%   1. It animates the WHOLE PASS on a TIME axis, not just the slide on a
%      progress axis. So the honeycomb is seen at rest, warming under the
%      descent, handing off during the slide, and cooling back afterwards.
%   2. Underneath the honeycomb it carries the two response traces (start
%      cell and destination cell), DRAWN PROGRESSIVELY -- each curve grows
%      only as far as the cursor, in step with the tip moving across the
%      map above, so the two panels read as one animation. The two boxes
%      share their left and right edges.
%
% Values come from linear_slide_timeline.m: LOCAL baseline (each pass's own
% pre-contact 'locate' mean), median across the 5 passes.
%
% STAGE BOUNDARIES are all even numbers, which is the reason 'approach' and
% 'press down' are drawn as one merged stage: the descent takes 3.71-3.76 s,
% never a whole number, so it cannot start on an even second at the same
% time as the slide. Merged, the stages run 0 -> 6 (approach + press down),
% 6 -> 8 or 6 -> 10 (slide), then the return.
%
% Output: results/hexmap_frames_v2/frames/<tag>_<seg>/f###.png, assembled
% by ./make_slide_videos.sh -d results/hexmap_frames_v2 -c
%
% The b3 session (tag b3_near) is INCLUDED, but with a caveat: logs/NOTES.md
% flags it as a failed run. The data is complete (6 neighbours x 5 passes)
% but the robot landed ~3 mm off the b3 pad (toward b4) and pressed ~0.35 mm
% deeper than the c3 sessions (Fz ~ -15 N vs -12 N), so at slide start the
% hottest cell is mostly b4, not b3. Read its start/end labels as nominal.
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
    'hollow_dome_5_iterations_c3_session_20260918_162945.csv',              'c3_near'; ...
    'hollow_dome_5_iterations_c3_long_displacement_session_20260918_163853.csv', 'c3_long'; ...
    'hollow_dome_5_iterations_b3_session_20260918_181142.csv',              'b3_near'};
% Render only these session tags; {} = all. Settable from the shell, e.g.
%   ONLY_SESSIONS='b3_near' octave <this script>
ONLY_SESSIONS = {};
env_sess = getenv('ONLY_SESSIONS');
if ~isempty(env_sess)
    ONLY_SESSIONS = strsplit(env_sess, ',');
    fprintf('ONLY_SESSIONS from env: %s\n', env_sess);
end

T_PRE  = 6.0;    % s loaded before slide start; also the axis shift, so the
                 % slide begins at an even 6 s
T_POST = 9.0;    % s after slide start -> axis runs 0 .. 15
BIN_S  = 0.25;
N_ANIM = 90;     % frames across the full 15 s window (0.167 s apart)

START_RGB = [0.8784 0.4941 0.2353];   % SDU orange - start cell
END_RGB   = [0.4784 0.3765 0.2510];   % SDU brown  - destination cell
STAGE_RGB3 = [0.918 0.918 0.918;      % approach + press down
              0.831 0.855 0.878;      % slide
              0.945 0.945 0.945];     % release / return
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
RESULTS_DIR = fullfile(HERE, 'results', 'hexmap_frames_v2');
if ~exist(RESULTS_DIR, 'dir'), mkdir(RESULTS_DIR); end

for si = 1:size(SESSIONS, 1)
    csv_name = SESSIONS{si, 1};
    tag      = SESSIONS{si, 2};
    if ~isempty(ONLY_SESSIONS) && ~any(strcmp(ONLY_SESSIONS, tag)), continue; end
    csv_path = fullfile(LOG_DIR, csv_name);
    fprintf('\n%s\n  SESSION: %s\n%s\n', repmat('=', 1, 70), tag, repmat('=', 1, 70));
    if ~exist(csv_path, 'file')
        warning('plot_linear_hexmap_frames_v2:missing', 'skipping %s', csv_name);
        continue;
    end

    T = linear_slide_timeline(csv_path, T_PRE, T_POST, BIN_S);
    S = linear_slide_load(csv_path, 12);   % only for depth_mm / speed_mm_s

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

        % matching entry in S, for the logged depth/speed
        js = find(strcmp({S.to_label}, T(e).to_label), 1);

        tshift  = T_PRE;                       % slide starts at an even 6 s
        pm      = T(e).phase_marks;
        b_slide = tshift;
        b_end   = tshift + T(e).t_slide_end;
        b_stop  = tshift + T_POST;
        b_eng   = tshift + pm.engage;          % first contact
        b_rep   = tshift + pm.reposition;      % lift-off
        bounds  = [0, b_slide, b_end, b_stop];
        names   = {'approach + press down', 'slide', 'release / return'};

        anim_dir = fullfile(RESULTS_DIR, 'frames', sprintf('%s_%s', tag, seg));
        if ~exist(anim_dir, 'dir'), mkdir(anim_dir); end

        tt = linspace(0, b_stop, N_ANIM);
        for b = 1:N_ANIM
            t = tt(b);

            % tip: parked on the start pad until the slide, travelling
            % during it, parked on the destination afterwards
            if t <= b_slide
                frac = 0;
            elseif t >= b_end
                frac = 1;
            else
                frac = (t - b_slide) / (b_end - b_slide);
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
            for sI = 1:3
                fill([bounds(sI) bounds(sI + 1) bounds(sI + 1) bounds(sI)], ...
                     [ylo ylo yhi yhi], STAGE_RGB3(sI, :), ...
                     'Parent', axR, 'EdgeColor', 'none');
            end
            for sI = 2:3
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
                'XColor', [0 0 0], 'YColor', [0 0 0], 'XTick', 0:2:ceil(b_stop));
            xlim(axR, [0, b_stop]); ylim(axR, [ylo, yhi]);
            xlabel(axR, 'time (s)', 'FontName', 'Helvetica', 'FontSize', 13);
            ylabel(axR, '\DeltaC/C_0', 'Interpreter', 'tex', ...
                'FontName', 'Helvetica', 'FontSize', 13);
            % stage names along the top of the trace panel
            for sI = 1:3
                text(axR, (bounds(sI) + bounds(sI + 1)) / 2, ylo + 0.92 * (yhi - ylo), ...
                    names{sI}, 'Color', [0.30 0.30 0.30], ...
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
            text(axh, 0.45, 0.995, sprintf('%s -> %s    t = %.1f s', ...
                    T(e).from_label, T(e).to_label, t), ...
                'Units', 'normalized', 'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'top', 'FontWeight', 'bold', ...
                'FontName', 'Helvetica', 'FontSize', 15, 'Interpreter', 'none');
            if ~isempty(js)
                text(axh, 0.45, 0.966, sprintf('speed = %g mm/s    depth = %g mm', ...
                        S(js).speed_mm_s, S(js).depth_mm - DEPTH_ZERO_MM), ...
                    'Units', 'normalized', 'HorizontalAlignment', 'center', ...
                    'VerticalAlignment', 'top', 'Color', [0.35 0.35 0.35], ...
                    'FontName', 'Helvetica', 'FontSize', 11, 'Interpreter', 'none');
            end

            print(f2, fullfile(anim_dir, sprintf('f%03d.png', b)), '-dpng', '-r100');
            close(f2);
        end
        fprintf('    %s: %d frames -> %s\n', seg, N_ANIM, anim_dir);
    end
    fprintf('  done -> %s\n', RESULTS_DIR);
end

fprintf('\nAll sessions done.\n');
fprintf(['\nTo build the videos:\n' ...
         '  ./make_slide_videos.sh -d results/hexmap_frames_v2 -c\n']);
