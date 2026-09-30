function T = bidir_slide_timeline(csv_path, t_pre, t_post, bin_s)
% BIDIR_SLIDE_TIMELINE  Cell response over one ROUND TRIP of a bidirectional
% slide (bidirectional_displacement.py), on a time axis anchored at the
% start of the OUTWARD slide.
%
% The bidirectional sibling of linear_slide_timeline.m. One segment is
% c3 <-> X repeated for N passes, and the tip STAYS ENGAGED for the whole
% segment:
%
%   locate | engage | hold_start | slide fwd | hold_end | slide rev | hold_return
%          (pass 0 only)          '------------- one pass (x N) --------------'
%   ... last pass: hold_return | retract
%
% BASELINE. Because the tip never lifts between passes, only pass 0 has an
% untouched reading ('locate'). That one segment-level locate mean is the
% baseline for EVERY pass of the segment -- the one-way loader's per-pass
% locate would leave passes 1..N-1 with no baseline at all.
%
% WINDOW PER PASS. Pass k contributes its own rows plus, for k >= 1, the
% 'hold_return' of pass k-1 -- the tip resting engaged on the start pad,
% i.e. the same physical state as pass 0's 'hold_start'. So:
%   * the approach/press-down (t < -hold) comes from pass 0 only,
%   * the hold at the start pad, both slides and both holds use every pass,
%   * the lift-off at the end ('retract') comes from the last pass only.
% Bins are reduced with the median across whichever passes cover them.
%
% 'progress' is NOT used: the logger restarts it from 0 on the return
% slide, so it cannot tell outward from return samples. Time from the
% slide start is used instead (the speed is constant, so time maps
% linearly to position within each slide).
%
%   csv_path : one bidirectional session log
%   t_pre    : seconds kept BEFORE the outward slide starts (default 6)
%   t_post   : seconds kept AFTER the end of the return hold (default 3,
%              which covers the lift-off)
%   bin_s    : time-bin width, seconds (default 0.25)
%
% Returns T, one entry per segment (from_label -> to_label):
%   t_centre          : 1 x nb bin centres, s from outward-slide start
%   dcc, dcc_q1/_q3   : nb x 19 median / quartiles of dC/C0 across passes
%   n_pass
%   marks             : median stage times relative to outward start:
%                       engage (pass 0), fwd_end, rev_start, rev_end,
%                       ret_end (end of the return hold), retract (last pass)
%   t_stop            : right edge of the window
%   depth_mm, speed_mm_s
    if nargin < 2 || isempty(t_pre),  t_pre  = 6.0; end
    if nargin < 3 || isempty(t_post), t_post = 3.0; end
    if nargin < 4 || isempty(bin_s),  bin_s  = 0.25; end

    fid = fopen(csv_path, 'r');
    if fid < 0
        error('bidir_slide_timeline:open', 'cannot open %s', csv_path);
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
    dirn   = C{col('direction')};
    from_l = C{col('from_label')};
    to_l   = C{col('to_label')};
    iter   = C{col('iter_idx')};
    depth  = C{col('depth_mm')};
    speed  = C{col('speed_mm_s')};
    cell_c = col('cell_1') + (0:18);
    cells  = zeros(numel(ts), 19);
    for k = 1:19
        cells(:, k) = C{cell_c(k)};
    end

    is_slide = strcmp(phase, 'slide');
    is_fwd   = strcmp(dirn, 'fwd');
    is_rev   = strcmp(dirn, 'rev');
    is_hret  = strcmp(phase, 'hold_return');

    % segments in the order they were recorded
    seg_to = to_l(is_slide);
    [~, first] = unique(seg_to);
    seg_to = seg_to(sort(first));

    T = struct([]);
    for si = 1:numel(seg_to)
        seg_rows = strcmp(to_l, seg_to{si});
        fr = from_l(seg_rows & is_slide);
        passes = unique(iter(seg_rows & is_slide));
        passes = passes(isfinite(passes));
        np = numel(passes);

        % --- segment-level LOCAL baseline (pass 0's locate) --------------
        bm = seg_rows & strcmp(phase, 'locate');
        if ~any(bm)
            warning('bidir_slide_timeline:nolocate', ...
                'segment %s has no locate samples -- skipped', seg_to{si});
            continue;
        end
        base = mean(cells(bm, :), 1);
        base(abs(base) < 1e-6) = NaN;

        % --- first sweep: stage times per pass ---------------------------
        t0 = nan(np, 1);
        mk = nan(np, 6);   % engage fwd_end rev_start rev_end ret_end retract
        for pi_ = 1:np
            m = seg_rows & (iter == passes(pi_));
            f = m & is_slide & is_fwd;
            if ~any(f), continue; end
            t0(pi_) = min(ts(f));
            r = m & is_slide & is_rev;
            h = m & is_hret;
            e = m & strcmp(phase, 'engage');
            q = m & strcmp(phase, 'retract');
            mk(pi_, 2) = max(ts(f)) - t0(pi_);
            if any(e), mk(pi_, 1) = min(ts(e)) - t0(pi_); end
            if any(r)
                mk(pi_, 3) = min(ts(r)) - t0(pi_);
                mk(pi_, 4) = max(ts(r)) - t0(pi_);
            end
            if any(h), mk(pi_, 5) = max(ts(h)) - t0(pi_); end
            if any(q), mk(pi_, 6) = min(ts(q)) - t0(pi_); end
        end
        med = nan(1, 6);
        for j = 1:6
            v = mk(isfinite(mk(:, j)), j);
            if ~isempty(v), med(j) = median(v); end
        end
        t_stop = med(5) + t_post;

        edges   = (-t_pre):bin_s:t_stop;
        centres = edges(1:end - 1) + bin_s / 2;
        nb = numel(centres);

        % --- second sweep: bin each pass ---------------------------------
        per_pass = nan(np, nb, 19);
        for pi_ = 1:np
            if ~isfinite(t0(pi_)), continue; end
            m = seg_rows & (iter == passes(pi_));
            if pi_ > 1
                % the previous pass's return hold = this pass's start hold
                m = m | (seg_rows & (iter == passes(pi_ - 1)) & is_hret);
            end
            rel = ts(m) - t0(pi_);
            cm  = cells(m, :);
            keep = rel >= -t_pre & rel <= t_stop;
            rel = rel(keep); cm = cm(keep, :);
            frac = (cm - repmat(base, size(cm, 1), 1)) ./ repmat(base, size(cm, 1), 1);
            for b = 1:nb
                inb = rel >= edges(b) & rel < edges(b + 1);
                if ~any(inb), continue; end
                per_pass(pi_, b, :) = mean(frac(inb, :), 1);
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

        e = numel(T) + 1;
        T(e).from_label = fr{1};
        T(e).to_label   = seg_to{si};
        T(e).t_centre   = centres;
        T(e).dcc = dcc; T(e).dcc_q1 = q1; T(e).dcc_q3 = q3;
        T(e).n_pass = np;
        T(e).marks = struct('engage', med(1), 'fwd_end', med(2), ...
            'rev_start', med(3), 'rev_end', med(4), 'ret_end', med(5), ...
            'retract', med(6));
        T(e).t_stop = t_stop;
        d = depth(seg_rows & is_slide); s = speed(seg_rows & is_slide);
        T(e).depth_mm = d(1); T(e).speed_mm_s = s(1);
    end
end
