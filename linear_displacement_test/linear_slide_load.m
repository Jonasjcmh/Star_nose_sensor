function S = linear_slide_load(csv_path, n_bins)
% LINEAR_SLIDE_LOAD  Per-segment cell response along each linear slide.
%
% The collector drives the tip to a fixed depth at a start pad and slides
% it in a straight line to an end pad, logging all 19 raw cells the whole
% way (see README.md). Only 'slide'-phase rows are used; the pass is
% parameterised by the logger's own 'progress' column (0 at the start pad,
% 1 at the end pad).
%
% dC/C0 uses a LOCAL baseline: each PASS's own mean over its 'locate'
% samples -- the phase before the tip touches down, so a genuine untouched
% reading taken seconds before the slide. Same baseline the press analyses
% use, and the same one linear_slide_timeline.m uses, so the two views of
% this data agree. calib_* (present in the log) is deliberately NOT used:
% it is a single snapshot from the start of the whole session and carries
% every bit of drift since.
%
% Values are binned on progress and reduced with the MEDIAN across the
% repeat passes, matching how the press analyses aggregate across rounds.
%
%   csv_path : one session log
%   n_bins   : progress bins across [0,1] (default 25)
%
% Returns S, a struct array with one entry per slide segment:
%   from_label, to_label : pad names ('c3', 'b2', ...)
%   bin_centre           : 1 x n_bins, progress at each bin centre
%   dcc                  : n_bins x 19, median dC/C0 per cell
%   dcc_q1, dcc_q3       : same shape, quartiles across passes
%   n_pass               : passes contributing
%   depth_mm, speed_mm_s : as logged
    if nargin < 2 || isempty(n_bins)
        % 12, not 25: a slide carries only ~40 samples per pass, so 25 bins
        % leaves a third of them empty and every gap breaks the plotted
        % line into disconnected fragments. Measured: 25 bins -> 33.3% NaN,
        % 12 bins -> 0%.
        n_bins = 12;
    end

    fid = fopen(csv_path, 'r');
    if fid < 0
        error('linear_slide_load:open', 'cannot open %s', csv_path);
    end
    hdr = strsplit(fgetl(fid), ',', 'CollapseDelimiters', false);
    n_cols = numel(hdr);
    col = @(nm) find(strcmp(hdr, nm), 1);
    fmt = repmat({'%f'}, 1, n_cols);
    for nm = {'datetime', 'from_label', 'to_label', 'phase', 'direction'}
        ci = col(nm{1});
        if ~isempty(ci), fmt{ci} = '%s'; end
    end
    C = textscan(fid, strjoin(fmt, ''), 'Delimiter', ',', 'Whitespace', '');
    fclose(fid);

    phase = C{col('phase')};
    from_l = C{col('from_label')};
    to_l   = C{col('to_label')};
    prog   = C{col('progress')};
    iter   = C{col('iter_idx')};
    depth  = C{col('depth_mm')};
    speed  = C{col('speed_mm_s')};
    cell_c  = col('cell_1')  + (0:18);

    cells = zeros(numel(phase), 19);
    for k = 1:19
        cells(:, k) = C{cell_c(k)};
    end
    is_locate = strcmp(phase, 'locate');

    is_slide = strcmp(phase, 'slide');
    segs = unique(strcat(from_l(is_slide), '>', to_l(is_slide)));

    edges = linspace(0, 1, n_bins + 1);
    centres = (edges(1:end - 1) + edges(2:end)) / 2;

    S = struct([]);
    for si = 1:numel(segs)
        parts = strsplit(segs{si}, '>');
        m = is_slide & strcmp(from_l, parts{1}) & strcmp(to_l, parts{2});
        passes = unique(iter(m));
        passes = passes(isfinite(passes));

        % --- LOCAL baseline, one row per pass: that pass's pre-contact
        % 'locate' mean. seg_rows covers ALL phases of the segment, not
        % just the slide, so the locate samples are reachable.
        seg_rows = strcmp(to_l, parts{2});
        base = nan(numel(passes), 19);
        for pi_ = 1:numel(passes)
            bm = seg_rows & is_locate & (iter == passes(pi_));
            if ~any(bm), continue; end
            base(pi_, :) = mean(cells(bm, :), 1);
        end
        base(abs(base) < 1e-6) = NaN;   % guard: a zero baseline blows up dC/C0

        dcc = nan(n_bins, 19); q1 = nan(n_bins, 19); q3 = nan(n_bins, 19);
        for b = 1:n_bins
            inb = m & prog >= edges(b) & prog < edges(b + 1);
            if b == n_bins
                inb = m & prog >= edges(b) & prog <= edges(b + 1);
            end
            if ~any(inb), continue; end
            % One value per PASS first (mean of that pass's samples in the
            % bin), then the median across passes -- same order as the
            % press analyses, so one noisy pass cannot dominate.
            pv = nan(numel(passes), 19);
            for pi_ = 1:numel(passes)
                sel = inb & (iter == passes(pi_));
                if ~any(sel), continue; end
                pv(pi_, :) = mean(cells(sel, :), 1);
            end
            % Divide each pass by ITS OWN locate baseline (computed below,
            % once per segment) before taking the median across passes --
            % the division has to happen per pass for a drifting baseline.
            frac = (pv - base) ./ base;
            for k = 1:19
                v = frac(:, k); v = v(isfinite(v));
                if isempty(v), continue; end
                dcc(b, k) = median(v);
                if numel(v) > 1
                    q1(b, k) = simple_percentile(v, 25);
                    q3(b, k) = simple_percentile(v, 75);
                else
                    q1(b, k) = v; q3(b, k) = v;
                end
            end
        end

        e = numel(S) + 1;
        S(e).from_label = parts{1};
        S(e).to_label   = parts{2};
        S(e).bin_centre = centres;
        S(e).dcc = dcc; S(e).dcc_q1 = q1; S(e).dcc_q3 = q3;
        S(e).n_pass = numel(passes);
        d = depth(m); s = speed(m);
        S(e).depth_mm = d(1); S(e).speed_mm_s = s(1);
    end

    % Loud if the binning outran the data -- empty bins become NaN and
    % silently shred any line drawn through them.
    tot = 0; nn = 0;
    for e = 1:numel(S)
        tot = tot + numel(S(e).dcc);
        nn  = nn  + sum(~isfinite(S(e).dcc(:)));
    end
    if tot > 0 && nn / tot > 0.02
        warning('linear_slide_load:sparse', ...
            ['%.1f%% of bins are empty at n_bins=%d -- lines will render ' ...
             'with gaps. Lower n_bins.'], 100 * nn / tot, n_bins);
    end
end
