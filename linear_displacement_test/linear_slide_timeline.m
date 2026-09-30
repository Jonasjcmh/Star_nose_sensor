function T = linear_slide_timeline(csv_path, t_pre, t_post, bin_s)
% LINEAR_SLIDE_TIMELINE  Cell response over the WHOLE pass, on a time axis
% anchored at the start of the slide, with a LOCAL baseline.
%
% Unlike linear_slide_load.m (which covers the slide only, parameterised by
% progress, and divides by calib_*), this keeps the approach and the
% retreat so the trace shows where the cells start, what the press does to
% them, and whether they return to equilibrium afterwards.
%
% One pass runs (measured, c3->d4):
%     locate 2.3s | engage 2.7s | hold_start 1.0s | slide 2.0s |
%     hold_end 1.0s | reposition 9.0s            sampled at ~20 Hz
%
% BASELINE. dC/C0 = (cell - base)/base with base = that PASS's own mean
% over its 'locate' samples -- the tip is still clear of the surface then,
% so it is a true untouched reading taken seconds before the press. This
% is the same idea as the press analyses' locate/post bracket, and it is
% per pass, so slow drift between passes cancels. calib_* (the
% session-start snapshot) is NOT used: it is one reading from the start of
% the whole session and carries all the drift since.
%
% TIME ZERO is the first 'slide' sample of each pass, so passes and
% segments line up even though their earlier phases vary slightly.
% Negative time is the approach, positive is the slide and everything
% after it.
%
%   csv_path : one session log
%   t_pre    : seconds to keep BEFORE slide start (default 6.5, which
%              reaches back into 'locate')
%   t_post   : seconds to keep AFTER slide start (default 9)
%   bin_s    : time-bin width, seconds (default 0.25 = 5 samples/pass/bin)
%
% Returns T, one entry per slide segment:
%   from_label, to_label
%   t_centre        : 1 x nb, bin centre in seconds from slide start
%   dcc             : nb x 19 median dC/C0 across passes
%   dcc_q1, dcc_q3  : quartiles across passes
%   n_pass
%   t_slide_end     : median slide duration (s) -- where the slide stops
%   phase_marks     : struct of median phase-start times, for annotation
    if nargin < 2 || isempty(t_pre),  t_pre  = 6.5; end
    if nargin < 3 || isempty(t_post), t_post = 9.0; end
    if nargin < 4 || isempty(bin_s),  bin_s  = 0.25; end

    fid = fopen(csv_path, 'r');
    if fid < 0
        error('linear_slide_timeline:open', 'cannot open %s', csv_path);
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

    ts     = C{col('timestamp')};
    phase  = C{col('phase')};
    from_l = C{col('from_label')};
    to_l   = C{col('to_label')};
    iter   = C{col('iter_idx')};
    cell_c = col('cell_1') + (0:18);
    cells  = zeros(numel(ts), 19);
    for k = 1:19
        cells(:, k) = C{cell_c(k)};
    end

    edges   = (-t_pre):bin_s:t_post;
    centres = edges(1:end - 1) + bin_s / 2;
    nb = numel(centres);

    is_slide = strcmp(phase, 'slide');
    segs = unique(strcat(from_l(is_slide), '>', to_l(is_slide)));

    T = struct([]);
    for si = 1:numel(segs)
        parts = strsplit(segs{si}, '>');
        seg_rows = strcmp(to_l, parts{2});        % every phase of this segment
        passes = unique(iter(seg_rows & is_slide));
        passes = passes(isfinite(passes));

        per_pass = nan(numel(passes), nb, 19);
        slide_len = nan(numel(passes), 1);
        marks = struct('locate', [], 'engage', [], 'hold_start', [], ...
                       'slide', [], 'hold_end', [], 'reposition', []);

        for pi_ = 1:numel(passes)
            m = seg_rows & (iter == passes(pi_));
            sl = m & is_slide;
            if ~any(sl), continue; end
            t0 = min(ts(sl));                      % time zero = slide start
            slide_len(pi_) = max(ts(sl)) - t0;

            % --- LOCAL baseline: this pass's own pre-contact 'locate' ---
            bm = m & strcmp(phase, 'locate');
            if ~any(bm)
                continue;   % without an untouched reading there is no local baseline
            end
            base = mean(cells(bm, :), 1);
            base(abs(base) < 1e-6) = NaN;          % guard against /0

            rel = ts(m) - t0;
            cm  = cells(m, :);
            keep = rel >= -t_pre & rel <= t_post;
            rel = rel(keep); cm = cm(keep, :);

            frac = (cm - repmat(base, size(cm, 1), 1)) ./ repmat(base, size(cm, 1), 1);
            for b = 1:nb
                inb = rel >= edges(b) & rel < edges(b + 1);
                if ~any(inb), continue; end
                per_pass(pi_, b, :) = mean(frac(inb, :), 1);
            end

            for f = {'locate', 'engage', 'hold_start', 'slide', 'hold_end', 'reposition'}
                pm = m & strcmp(phase, f{1});
                if any(pm)
                    marks.(f{1})(end + 1) = min(ts(pm)) - t0;   %#ok<AGROW>
                end
            end
        end

        dcc = nan(nb, 19); q1 = nan(nb, 19); q3 = nan(nb, 19);
        for b = 1:nb
            for k = 1:19
                v = per_pass(:, b, k); v = v(isfinite(v));
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

        for f = fieldnames(marks)'
            marks.(f{1}) = median(marks.(f{1}));
        end

        e = numel(T) + 1;
        T(e).from_label = parts{1};
        T(e).to_label   = parts{2};
        T(e).t_centre   = centres;
        T(e).dcc = dcc; T(e).dcc_q1 = q1; T(e).dcc_q3 = q3;
        T(e).n_pass = numel(passes);
        T(e).t_slide_end = median(slide_len(isfinite(slide_len)));
        T(e).phase_marks = marks;
    end
end
