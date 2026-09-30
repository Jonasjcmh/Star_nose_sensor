% plot_linear_response.m — Linear slide | cell response over the pass
% =========================================================================
% For each c3 -> X (and b3 -> X) slide, every sensor cell's dC/C0 plotted against TIME,
% with t = 0 at the start of the slide. This is the "linearisation" view:
% it shows the response handing off from the start cell to the destination
% cell as the contact travels -- and, either side of that, the cells at
% rest before contact and their return to rest after the lift-off.
%
% By default only TWO cells are drawn: the START cell (c3) and the
% DESTINATION cell, each with a q1-q3 band across the 5 repeat passes.
% Set SHOW_OTHER_CELLS to draw the remaining 17 as faint background lines
% (the crosstalk the rest of the board picks up while the contact moves),
% tinted on the SDU gradient by their own peak dC/C0.
%
% ON NEGATIVE VALUES. dC/C0 goes slightly negative, but only OUTSIDE the
% press. Measured over the near session: before contact the values are
% symmetric noise about zero (mean -0.001, sd 0.030, 53% negative), which
% is exactly what a correct local baseline should give -- the baseline is
% the mean of those same samples, so half of them must fall below it.
% DURING the slide 0% of values are negative (min -0.009). So no cell ever
% genuinely loses capacitance under load; the negatives are the sensor
% noise floor either side of the press. Y_MIN_ZERO therefore clips the
% AXIS at zero rather than clamping the data -- clamping would fold a
% symmetric +-0.03 noise band onto one side and bias the apparent resting
% level upward by ~0.012.
%
% dC/C0 uses a LOCAL baseline: each pass's own mean over its 'locate'
% samples, taken seconds before the tip touches down. calib_* (the
% session-start snapshot) is not used -- it carries all the drift since
% the session began.
%
% The trace spans the WHOLE pass, not just the slide: it starts in the
% approach (before contact) and runs past the release into 'reposition',
% so the figure shows where the cells sit at rest, what the press does,
% and whether they return to equilibrium.
%
% TIME ORIGIN. t = 0 is the start of the plotted window, so the axis
% carries no negative times. The window is anchored so PRESS DOWN begins
% at exactly PRESS_START_S (2.0 s), trimming the approach to whatever is
% left -- the approach only establishes the resting level, so its length
% does not matter.
%
% Only ONE stage boundary can be an integer at a time. Measured across all
% 18 segments the descent takes 3.71-3.76 s, never a whole number, so
% putting the start of the press on an integer necessarily puts the slide
% start at ~5.7 s, and vice versa. Set STAGE_ANCHOR to choose which one is
% round.
%
% VERIFIED before building: at progress 0 the hottest cell is c3 in all 6
% short-session segments, and at progress 1 it is exactly the destination
% pad's cell in all 6. So cell k belongs to point P<k> (identity) here and
% the peak really does track the tip.
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
% Shared plotting helpers (SDU colormap, hexagon maths, save_fig, ...).
addpath(fullfile(HERE, '..', 'Interdome_touch', 'matlab_analysis'));

% ── CONFIG ───────────────────────────────────────────────────────────────
% b3_near is flagged failed in logs/NOTES.md -- see the caveat above.
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

% Loaded generously (the approach is trimmed later by the anchoring), but
% not past ~6.2 s, where 'locate' begins and the earlier bins run empty.
T_PRE  = 6.0;    % s loaded before slide start (reaches into 'locate')
T_POST = 9.0;    % s kept after  slide start (into 'reposition')
BIN_S  = 0.25;   % time-bin width (~5 samples per pass per bin at 20 Hz)
LAB = {'a1','a2','a3','b1','b2','b3','b4','c1','c2','c3', ...
       'c4','c5','d2','d3','d4','d5','e3','e4','e5'};
% SDU brand palette for the two highlighted cells (the same trio used for
% surfaces elsewhere in the project). Deliberately NOT the greens/tans of
% the background gradient below, so the two stay readable on top of it.
START_RGB = [0.8784 0.4941 0.2353];   % SDU orange #E07E3C - start cell
END_RGB   = [0.4784 0.3765 0.2510];   % SDU brown  #7A6040 - destination
AX_FONT   = 20;                       % axis labels
TICK_FONT = 18;                       % tick numbers
SHOW_OTHER_CELLS  = false;  % the 17 non-endpoint cells as faint background
% Alternating shaded bands behind the stages ("shadows"), lightest for the
% quiet phases and a touch stronger under the slide, which is the subject.
STAGE_SHADING = true;
% Merge 'approach' and 'press down' into one stage. Doing so is what makes
% EVERY boundary an even number: the merged stage runs 0 -> 6, the slide
% 6 -> 8 (near) or 6 -> 10 (long), the return from there. Kept separate,
% the descent boundary lands on ~2.3 s, which cannot be made whole at the
% same time as the slide (the descent takes 3.71-3.76 s).
COMBINE_APPROACH = true;
% Which boundary lands on a whole number -- they cannot both (see above).
%   'press' : press down starts at PRESS_START_S, slide starts at ~5.7
%   'slide' : slide starts at T_PRE (6.0), press down starts at ~2.3
STAGE_ANCHOR  = 'slide';
PRESS_START_S = 2.0;
SHOW_TITLES   = false;   % no figure title and no per-axes title
STAGE_RGB = [0.945 0.945 0.945;   % approach
             0.898 0.898 0.898;   % press down
             0.831 0.855 0.878;   % slide  - cool tint, the subject
             0.945 0.945 0.945];  % release / return
STAGE_RGB3 = [0.918 0.918 0.918;  % approach + press down (merged)
              0.831 0.855 0.878;  % slide
              0.945 0.945 0.945]; % release / return
Y_MIN_ZERO        = true;   % start the y axis at 0 (display only, see above)
% Panel shape, width:height. 4 gives the wide, flat trace the phases read
% along. Set in PIXELS so the axes box really is this ratio -- MATLAB
% clamps an oversized figure window and squashes the axes, which looks
% like the ratio being ignored.
PANEL_ASPECT_RATIO = 4.0;
AX_WIDTH_PX        = 1280;
KEEP_FIGURES_OPEN = false;

LOG_DIR     = fullfile(HERE, 'logs');
RESULTS_DIR = fullfile(HERE, 'results', 'response_vs_position');
if ~exist(RESULTS_DIR, 'dir'), mkdir(RESULTS_DIR); end

for si = 1:size(SESSIONS, 1)
    csv_name = SESSIONS{si, 1};
    tag      = SESSIONS{si, 2};
    if ~isempty(ONLY_SESSIONS) && ~any(strcmp(ONLY_SESSIONS, tag)), continue; end
    csv_path = fullfile(LOG_DIR, csv_name);
    fprintf('\n%s\n  SESSION: %s\n%s\n', repmat('=', 1, 70), tag, repmat('=', 1, 70));
    if ~exist(csv_path, 'file')
        warning('plot_linear_response:missing', 'skipping %s', csv_name);
        continue;
    end

    S = linear_slide_timeline(csv_path, T_PRE, T_POST, BIN_S);
    fprintf('  %d slide segments, %d passes each (local baseline, t=0 at slide start)\n', ...
        numel(S), S(1).n_pass);

    % One y-scale across the session so segments are comparable. Built
    % from the cells that are actually PLOTTED -- with the background
    % cells off, their spread must not inflate the scale.
    allv = [];
    for e = 1:numel(S)
        if SHOW_OTHER_CELLS
            kk = 1:19;
        else
            kk = [find(strcmp(LAB, S(e).from_label), 1), ...
                  find(strcmp(LAB, S(e).to_label), 1)];
        end
        v = [reshape(S(e).dcc(:, kk), [], 1); ...
             reshape(S(e).dcc_q1(:, kk), [], 1); ...
             reshape(S(e).dcc_q3(:, kk), [], 1)];
        allv = [allv; v(isfinite(v))];   %#ok<AGROW>
    end
    if Y_MIN_ZERO
        ylo = 0;
    else
        ylo = min(0, min(allv));
    end
    yhi = max(allv) * 1.08;
    fprintf('  shared y-scale: [%.3f, %.3f]\n', ylo, yhi);

    for e = 1:numel(S)
        k_start = find(strcmp(LAB, S(e).from_label), 1);
        k_end   = find(strcmp(LAB, S(e).to_label), 1);

        % Margins in pixels, sized for AX_FONT/TICK_FONT above; the axes
        % box itself is exactly AX_WIDTH_PX x AX_WIDTH_PX/PANEL_ASPECT_RATIO.
        mL = 130; mR = 40; mB = 105;
        if SHOW_TITLES, mT = 70; else, mT = 22; end
        axw = AX_WIDTH_PX; axh = round(AX_WIDTH_PX / PANEL_ASPECT_RATIO);
        figw = axw + mL + mR; figh = axh + mB + mT;
        fig = figure('Position', [70 70 figw figh], 'Color', [1 1 1]);
        ax = axes('Parent', fig, 'Units', 'pixels', ...
            'Position', [mL mB axw axh]);
        hold(ax, 'on');

        % ---- stage shading + annotation ---------------------------------
        % Boundaries on the shifted axis (t = 0 at the window start).
        pm = S(e).phase_marks;
        % Shift from the loader's slide-anchored time to the plotted axis.
        if strcmp(STAGE_ANCHOR, 'press')
            tshift = PRESS_START_S - pm.engage;   % press down -> 2.0 s
        else
            tshift = T_PRE;                        % slide -> 6.0 s
        end
        b_engage = tshift + pm.engage;       % descent begins
        b_slide  = tshift;                   % slide begins
        b_end    = tshift + S(e).t_slide_end;% slide ends
        b_stop   = tshift + T_POST;          % right edge
        if COMBINE_APPROACH
            bounds = [0, b_slide, b_end, b_stop];
            names  = {'approach + press down', 'slide', 'release / return'};
            srgb   = STAGE_RGB3;
        else
            bounds = [0, b_engage, b_slide, b_end, b_stop];
            names  = {'approach', 'press down', 'slide', 'release / return'};
            srgb   = STAGE_RGB;
        end
        n_stage = numel(names);

        if STAGE_SHADING
            % fill(), not patch(): patch renders nothing under Octave's
            % gnuplot toolkit. Drawn first so the traces sit on top.
            for sI = 1:n_stage
                x0 = bounds(sI); x1 = bounds(sI + 1);
                fill([x0 x1 x1 x0], [ylo ylo yhi yhi], srgb(sI, :), ...
                    'Parent', ax, 'EdgeColor', 'none');
            end
        end
        % thin dividers on top of the shading
        for sI = 2:n_stage
            plot(ax, [bounds(sI) bounds(sI)], [ylo yhi], '-', ...
                'Color', [0.62 0.62 0.62], 'LineWidth', 0.8);
        end
        % zero line = the untouched 'locate' level each pass was normalised
        % to, so anything on it is a cell sitting at rest.
        plot(ax, [0 b_stop], [0 0], '-', 'Color', [0.75 0.75 0.75]);


        % NOTE: bin_centre is 1xN and dcc(:,k) is Nx1. Passing those to
        % plot() as-is makes Octave treat them as a matrix and draw N
        % lines of scribble instead of one curve -- both are forced to
        % columns here.
        % the 17 other cells, tinted on the SDU gradient by their own
        % peak response so a strongly-coupled cell is visually distinct
        % from one that never moves
        if SHOW_OTHER_CELLS
            cmap = sdu_hex_cmap();
            peaks = max(S(e).dcc, [], 1);
            pk_lo = min(peaks(isfinite(peaks))); pk_hi = max(peaks(isfinite(peaks)));
            for k = 1:19
                if k == k_start || k == k_end, continue; end
                rgb = value_to_rgb(peaks(k), pk_lo, pk_hi, cmap);
                % blended 45% toward white: keeps the SDU hue coding but lets
                % the two brand-coloured cells sit clearly on top
                rgb = rgb + 0.45 * ([1 1 1] - rgb);
                plot(ax, (S(e).t_centre(:) + tshift), S(e).dcc(:, k), '-', 'Color', rgb, 'LineWidth', 1.0);
            end
        end
        % start + destination cells, with their spread across passes
        h = [];
        for kk = [k_start, k_end]
            if isempty(kk), continue; end
            if kk == k_start, rgb = START_RGB; else, rgb = END_RGB; end
            lo = S(e).dcc_q1(:, kk)'; hi = S(e).dcc_q3(:, kk)';
            ok = isfinite(lo) & isfinite(hi);
            if any(ok)
                xb = S(e).t_centre(ok) + tshift;
                % fill(), not patch(): patch renders nothing under Octave's
                % gnuplot toolkit.
                fill([xb, fliplr(xb)], [lo(ok), fliplr(hi(ok))], rgb, ...
                    'Parent', ax, 'FaceAlpha', 0.18, 'EdgeColor', 'none');
            end
            h(end + 1) = plot(ax, (S(e).t_centre(:) + tshift), S(e).dcc(:, kk), '-', ...
                'Color', rgb, 'LineWidth', 2.4);   %#ok<AGROW>
        end
        % stage names along the TOP of the panel
        ytop = ylo + 0.94 * (yhi - ylo);
        for sI = 1:n_stage
            text(ax, (bounds(sI) + bounds(sI + 1)) / 2, ytop, names{sI}, ...
                'Color', [0.28 0.28 0.28], 'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'middle', 'FontName', 'Helvetica', ...
                'FontSize', 13, 'Interpreter', 'none');
        end

        set(ax, 'FontName', 'Helvetica', 'FontSize', TICK_FONT, 'Box', 'on', ...
            'XColor', [0 0 0], 'YColor', [0 0 0], 'Layer', 'top', ...
            'XTick', 0:1:ceil(b_stop));
        xlim(ax, [0, b_stop]); ylim(ax, [ylo, yhi]);
        xlabel(ax, sprintf('time (s)   [%s \\rightarrow %s]', ...
            S(e).from_label, S(e).to_label), 'Interpreter', 'tex', ...
            'FontName', 'Helvetica', 'FontSize', AX_FONT);
        % 'tex', not 'latex': under Octave's gnuplot toolkit the latex
        % interpreter prints the source verbatim ("$\Delta C/C_0$"),
        % while tex renders the glyphs properly -- and tex behaves the
        % same way in real MATLAB.
        ylabel(ax, '\DeltaC/C_0', 'Interpreter', 'tex', ...
            'FontName', 'Helvetica', 'FontSize', AX_FONT);
        if SHOW_TITLES
            title(ax, sprintf('%s -> %s   (%d passes, local baseline)', ...
                S(e).from_label, S(e).to_label, S(e).n_pass), ...
                'Interpreter', 'none', 'FontName', 'Helvetica', 'FontSize', 12);
        end
        % ---- legend, drawn by hand --------------------------------------
        % Not legend(): the stage names now occupy the top row, and under
        % Octave setting a legend's 'Position' to move it clear is
        % silently ignored (the box stays northwest, on top of
        % 'approach'). Two line stubs plus text put it exactly where the
        % panel is empty -- the left third above ~0.5, before the descent.
        lx0 = 0.25; lx1 = 1.15; ltx = 1.35;
        lys = [0.86, 0.75] * yhi;
        lcol = {START_RGB, END_RGB};
        ltxt = {sprintf('start cell %s', S(e).from_label), ...
                sprintf('end cell %s',   S(e).to_label)};
        for lI = 1:2
            plot(ax, [lx0 lx1], [lys(lI) lys(lI)], '-', ...
                'Color', lcol{lI}, 'LineWidth', 2.4);
            text(ax, ltx, lys(lI), ltxt{lI}, 'FontName', 'Helvetica', ...
                'FontSize', 13, 'VerticalAlignment', 'middle', ...
                'Interpreter', 'none', 'Color', [0.15 0.15 0.15]);
        end

        if SHOW_TITLES
            fig_suptitle(fig, sprintf('Linear slide (%s) - cell response over the pass', tag));
        end
        save_fig(fig, fullfile(RESULTS_DIR, sprintf('%s_slide_%s_to_%s', ...
            tag, S(e).from_label, S(e).to_label)));
        if ~KEEP_FIGURES_OPEN, close(fig); end
    end
    fprintf('  done -> %s\n', RESULTS_DIR);
end

fprintf('\nAll sessions done.\n');
