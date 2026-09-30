% plot_linear_hexmap_frames.m — Linear slide | hexmap snapshots along the path
% =========================================================================
% The travelling-contact view: for each c3 -> X slide, the honeycomb is
% rendered at several points along the path, cells filled by dC/C0, with a
% marker showing where the tip is. Read left-to-right the active region
% walks from the start pad to the destination.
%
% Two outputs per segment:
%   *_frames.(png|svg)  -- all snapshots as one strip, easy to compare
%   frames/<seg>/f###.png -- N_ANIM frames per segment, assembled into
%                            video by ./make_slide_videos.sh (use -c to
%                            also concatenate every segment into one
%                            results/hexmap_frames/all_slides.mp4).
%   (no movie is written here: Octave's gnuplot toolkit has no reliable
%    writer, and the frames are reusable anyway.)
%
% Tip position is interpolated between the two pads' CANONICAL board
% coordinates using the logger's own 'progress' column, rather than
% transforming tcp_x/y -- the pads are known points on the board, so this
% needs no robot->board calibration and cannot drift.
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

%% ---- helpers (defined up top -- Octave requires that) ------------------
function v = dcc_at(S_e, t)
% The 19 cell values at an arbitrary progress t in [0,1], linearly
% interpolated between the measured bin centres (which do not reach 0 or
% 1, hence 'extrap' for the two ends).
    v = nan(1, 19);
    for k = 1:19
        y = S_e.dcc(:, k);
        ok = isfinite(y);
        if nnz(ok) < 2, continue; end
        v(k) = interp1(S_e.bin_centre(ok), y(ok), t, 'linear', 'extrap');
    end
end

function draw_frame(ax, CANON, vals, tip_xy, vmin, vmax, cmap, hex_r)
% Honeycomb filled by dC/C0 + a marker at the tip.
    hold(ax, 'on');
    for k = 1:19
        [vx, vy] = hex_vertices(CANON(k, 1), CANON(k, 2), hex_r);
        fill(vx, vy, value_to_rgb(vals(k), vmin, vmax, cmap), 'Parent', ax, ...
            'EdgeColor', [0.35 0.35 0.35], 'LineWidth', 0.7);
    end
    th = linspace(0, 2 * pi, 40);
    r = 1.6;
    % White disc with a black ring: readable on any colormap value.
    fill(tip_xy(1) + r * cos(th), tip_xy(2) + r * sin(th), [1 1 1], ...
        'Parent', ax, 'EdgeColor', [0 0 0], 'LineWidth', 1.6);
    axis(ax, 'equal');
    pad = hex_r * 1.2;
    xlim(ax, [min(CANON(:, 1)) - pad, max(CANON(:, 1)) + pad]);
    ylim(ax, [min(CANON(:, 2)) - pad, max(CANON(:, 2)) + pad]);
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

N_BINS    = 12;    % measured bins (12 keeps every bin populated -- see
                   % linear_slide_load.m; finer binning leaves gaps)
% Strip snapshots at exact round percentages. The measured bin CENTRES sit
% at 4.2%, 12.5%, ... 95.8%, so these are interpolated between bins rather
% than picked from them -- that is the only way to land on round numbers.
STRIP_PCT = [0 20 40 60 80 100];
% Animation frames, also interpolated between the 12 measured bins. More
% frames buys smooth motion, not extra information.
N_ANIM    = 61;
WRITE_ANIM_FRAMES = true;
% Re-render only these '<tag>_<from>_to_<to>' segments; {} = all. Useful
% because a full 18-segment run takes ~7 min and has hung on a gnuplot
% child before -- this lets a partial run be finished off without
% redoing the segments that already came out.
% Also settable without editing this file, e.g.
%   ONLY_SEGMENTS='c3_long_c3_to_c1,c3_long_c3_to_c5' octave plot_...m
% (an env var, because the 'clear' at the top of this script wipes any
% variable you try to preset in the workspace).
ONLY_SEGMENTS = {};
env_only = getenv('ONLY_SEGMENTS');
if ~isempty(env_only)
    ONLY_SEGMENTS = strsplit(env_only, ',');
    fprintf('ONLY_SEGMENTS from env: %s\n', env_only);
end
% The logger's depth_mm is measured from a zero that sits 5 mm above the
% surface, so the INDENTATION actually applied is depth_mm - 5. These logs
% record 9.0 mm, i.e. 4 mm of indentation -- the same convention as the
% interdome press analyses.
DEPTH_ZERO_MM = 5.0;
HEX_RADIUS = 8.0 / sqrt(3);
KEEP_FIGURES_OPEN = false;

% Canonical board layout, row k = point P<k>; same table the interdome
% figures use, so c3 = P10 sits at the origin and P01..P19 read left to
% right along y = 0.
CANON = [ -16,   0;  -12,   7;   -8,  14;  -12,  -7;   -8,   0;
           -4,   7;    0,  14;   -8, -14;   -4,  -7;    0,   0;
            4,   7;    8,  14;    0, -14;    4,  -7;    8,   0;
           12,   7;    8, -14;   12,  -7;   16,   0];
LAB = {'a1','a2','a3','b1','b2','b3','b4','c1','c2','c3', ...
       'c4','c5','d2','d3','d4','d5','e3','e4','e5'};

LOG_DIR     = fullfile(HERE, 'logs');
RESULTS_DIR = fullfile(HERE, 'results', 'hexmap_frames');
if ~exist(RESULTS_DIR, 'dir'), mkdir(RESULTS_DIR); end

for si = 1:size(SESSIONS, 1)
    csv_name = SESSIONS{si, 1};
    tag      = SESSIONS{si, 2};
    if ~isempty(ONLY_SESSIONS) && ~any(strcmp(ONLY_SESSIONS, tag)), continue; end
    csv_path = fullfile(LOG_DIR, csv_name);
    fprintf('\n%s\n  SESSION: %s\n%s\n', repmat('=', 1, 70), tag, repmat('=', 1, 70));
    if ~exist(csv_path, 'file')
        warning('plot_linear_hexmap_frames:missing', 'skipping %s', csv_name);
        continue;
    end

    S = linear_slide_load(csv_path, N_BINS);
    allv = [];
    for e = 1:numel(S)
        v = S(e).dcc(:); allv = [allv; v(isfinite(v))];   %#ok<AGROW>
    end
    vmin = min(allv); vmax = max(allv);
    cmap = sdu_hex_cmap();
    fprintf('  %d segments | shared dC/C0 scale [%.3f, %.3f]\n', numel(S), vmin, vmax);

    for e = 1:numel(S)
        k_from = find(strcmp(LAB, S(e).from_label), 1);
        k_to   = find(strcmp(LAB, S(e).to_label), 1);
        p_from = CANON(k_from, :); p_to = CANON(k_to, :);
        seg = sprintf('%s_to_%s', S(e).from_label, S(e).to_label);
        if ~isempty(ONLY_SEGMENTS) && ...
                ~any(strcmp(ONLY_SEGMENTS, sprintf('%s_%s', tag, seg)))
            continue;
        end

        % ---- strip of N_FRAMES snapshots ---------------------------------
        n_strip = numel(STRIP_PCT);
        fig = figure('Position', [50 50 300 * n_strip + 150, 400], 'Color', [1 1 1]);
        mL = 0.02; mR = 0.10; mT = 0.08; mB = 0.04; gx = 0.008;
        pw = (1 - mL - mR - (n_strip - 1) * gx) / n_strip;
        last_ax = [];
        for fI = 1:n_strip
            t = STRIP_PCT(fI) / 100;
            ax = axes('Parent', fig, 'Position', [mL + (fI - 1) * (pw + gx), mB, pw, 1 - mT - mB]);
            draw_frame(ax, CANON, dcc_at(S(e), t), p_from + t * (p_to - p_from), ...
                vmin, vmax, cmap, HEX_RADIUS);
            % only the percentage -- the figure title was dropped per request
            title(ax, sprintf('%d%%', STRIP_PCT(fI)), 'FontName', 'Helvetica', 'FontSize', 13);
            last_ax = ax;
        end
        colormap(fig, cmap); caxis(last_ax, [vmin, vmax]);
        cb = colorbar(last_ax, 'Position', [1 - mR + 0.025, mB, 0.014, 1 - mT - mB]);
        try, set(cb, 'FontName', 'Helvetica', 'FontSize', 9); catch, end
        ylabel(cb, 'dC/C0', 'Interpreter', 'none');
        save_fig(fig, fullfile(RESULTS_DIR, sprintf('%s_%s_frames', tag, seg)));
        if ~KEEP_FIGURES_OPEN, close(fig); end

        % ---- individual frames for animation ------------------------------
        if WRITE_ANIM_FRAMES
            anim_dir = fullfile(RESULTS_DIR, 'frames', sprintf('%s_%s', tag, seg));
            if ~exist(anim_dir, 'dir'), mkdir(anim_dir); end
            nb = N_ANIM;
            tt = linspace(0, 1, nb);
            for b = 1:nb
                t = tt(b);
                f2 = figure('Position', [50 50 620 620], 'Color', [1 1 1], 'Visible', 'off');
                % 0.82 high, not 0.86: leaves room for the two-line heading
                ax2 = axes('Parent', f2, 'Position', [0.04 0.04 0.80 0.82]);
                draw_frame(ax2, CANON, dcc_at(S(e), t), p_from + t * (p_to - p_from), ...
                    vmin, vmax, cmap, HEX_RADIUS);
                colormap(f2, cmap); caxis(ax2, [vmin, vmax]);
                cb2 = colorbar(ax2, 'Position', [0.87 0.04 0.03 0.82]);
                try, set(cb2, 'FontName', 'Helvetica', 'FontSize', 9); catch, end
                % Two-line heading, kept on the ANIMATION frames (unlike the
                % strip): without it a concatenated all-segment video gives
                % no clue which slide is playing. Drawn on an invisible
                % full-figure axes rather than title()+subtitle(), because
                % Octave has no subtitle() and a two-line title cellstr
                % would give both lines the same size. Created AFTER the
                % colorbar so it does not steal the current axes.
                axh = axes('Parent', f2, 'Position', [0 0 1 1], 'Visible', 'off');
                text(axh, 0.44, 0.985, sprintf('%s -> %s    %.0f%%', ...
                        S(e).from_label, S(e).to_label, 100 * t), ...
                    'Units', 'normalized', 'HorizontalAlignment', 'center', ...
                    'VerticalAlignment', 'top', 'FontWeight', 'bold', ...
                    'FontName', 'Helvetica', 'FontSize', 14, 'Interpreter', 'none');
                text(axh, 0.44, 0.940, sprintf('speed = %g mm/s    depth = %g mm', ...
                        S(e).speed_mm_s, S(e).depth_mm - DEPTH_ZERO_MM), ...
                    'Units', 'normalized', 'HorizontalAlignment', 'center', ...
                    'VerticalAlignment', 'top', 'Color', [0.35 0.35 0.35], ...
                    'FontName', 'Helvetica', 'FontSize', 11, 'Interpreter', 'none');
                print(f2, fullfile(anim_dir, sprintf('f%03d.png', b)), '-dpng', '-r110');
                close(f2);
            end
            fprintf('    %s: %d frames -> %s\n', seg, nb, anim_dir);
        end
    end
    fprintf('  done -> %s\n', RESULTS_DIR);
end

fprintf('\nAll sessions done.\n');
fprintf(['\nTo build the videos (per segment + one compilation):\n' ...
         '  ./make_slide_videos.sh -c\n' ...
         'That writes results/hexmap_frames/frames/<segment>/<segment>.mp4\n' ...
         'and results/hexmap_frames/all_slides.mp4.\n']);
