function T = traj_timeline(csv_path, t_pre, t_post, bin_s)
% TRAJ_TIMELINE  Cell response over one pass of a MULTI-POINT trajectory
% (trajectory_displacement.py), median across the repeat passes.
%
% One pass slides through a sequence of pads and back, staying engaged,
% with a hold at every pad:
%
%   [locate | engage | hold_start]  (pass 0 only)
%   slide P1->P2 | hold_mid | ... | slide ->PN | hold_end      (out)
%   slide PN->.. | hold_mid | ... | slide ->P1 | hold_return   (back)
%   [retract]                       (last pass only)
%
% ALIGNMENT. A pass lasts minutes (1 mm/s), so small per-slide timing
% differences would add up and smear a plain time-since-start median.
% Instead every pass is PIECEWISE aligned: each slide/hold block k of a
% pass is mapped onto that block's median start time across passes, so all
% passes line up at every pad. t = 0 is the start of the first slide.
%
% BASELINE. The tip engages once for the whole session, so the session's
% single pre-contact 'locate' mean is the baseline for every pass.
%
% Pre-window (t < 0): pass 0's locate/engage/hold_start, and for later
% passes the previous pass's 'hold_return' (the same state: resting
% engaged on P1). Post-window: the last pass's 'retract'.
%
% Returns T with:
%   path_out     : 1 x (N+1) pads visited on the way out, P1..PN
%   blocks       : struct array, one per core block, canonical times:
%                  phase, from, to, dir, t_start, t_end
%   t_centre, dcc, dcc_q1, dcc_q3 : binned median / quartiles (nb x 19)
%   n_pass, t_stop, t_core_end, depth_mm, speed_mm_s
%   t_engage     : first contact (pass 0), s relative to the first slide
%   holds        : struct array, one per (pass, hold at a pad):
%                  pass, pad, dir, dcc (1x19 mean over the hold)
    if nargin < 2 || isempty(t_pre),  t_pre  = 6.0; end
    if nargin < 3 || isempty(t_post), t_post = 3.0; end
    if nargin < 4 || isempty(bin_s),  bin_s  = 0.5; end

    fid = fopen(csv_path, 'r');
    if fid < 0
        error('traj_timeline:open', 'cannot open %s', csv_path);
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
    n = numel(ts);

    % ---- baseline: the session's pre-contact 'locate' ------------------
    bm = strcmp(phase, 'locate');
    if ~any(bm)
        error('traj_timeline:nolocate', 'no locate samples in %s', csv_path);
    end
    base = mean(cells(bm, :), 1);
    base(abs(base) < 1e-6) = NaN;
    frac_all = (cells - repmat(base, n, 1)) ./ repmat(base, n, 1);

    % ---- runs of identical (phase, from, to, dir, iter) ----------------
    key = strcat(phase, '|', from_l, '|', to_l, '|', dirn);
    chg = [true; ~strcmp(key(2:end), key(1:end - 1)) | diff(iter) ~= 0];
    run_id = cumsum(chg);
    rs = find(chg);                       % first row of each run
    re = [rs(2:end) - 1; n];              % last row of each run
    nr = numel(rs);

    core_ph = {'slide', 'hold_mid', 'hold_end', 'hold_return'};
    is_core = false(nr, 1);
    for r = 1:nr
        is_core(r) = any(strcmp(phase{rs(r)}, core_ph));
    end
    run_iter = iter(rs);
    passes = unique(run_iter(is_core));
    passes = passes(isfinite(passes) & passes >= 0);
    np = numel(passes);

    % core runs of each pass, in order
    core = cell(np, 1);
    for pi_ = 1:np
        core{pi_} = find(is_core & run_iter == passes(pi_));
    end
    nb_ = cellfun(@numel, core);
    if any(nb_ ~= nb_(1))
        error('traj_timeline:blocks', ...
            'passes have different block counts (%s) -- cannot align', mat2str(nb_'));
    end
    nblk = nb_(1);

    % block start offsets (from the pass's first slide) and durations
    st = nan(np, nblk); du = nan(np, nblk); t0 = nan(np, 1);
    for pi_ = 1:np
        r = core{pi_};
        t0(pi_) = ts(rs(r(1)));
        st(pi_, :) = ts(rs(r))' - t0(pi_);
        nxt = [ts(rs(r(2:end)))', ts(re(r(end))) + 0.05];
        du(pi_, :) = nxt - ts(rs(r))';
    end
    cd = median(du, 1);
    cs = [0, cumsum(cd(1:end - 1))];
    t_core_end = cs(end) + cd(end);
    t_stop = t_core_end + t_post;

    % ---- canonical time for every sample of every pass -----------------
    edges   = (-t_pre):bin_s:t_stop;
    centres = edges(1:end - 1) + bin_s / 2;
    nbin    = numel(centres);
    per_pass = nan(np, nbin, 19);
    holds = struct('pass', {}, 'pad', {}, 'dir', {}, 'dcc', {});
    for pi_ = 1:np
        rows = []; tc = [];
        r = core{pi_};
        for j = 1:nblk
            rr = (rs(r(j)):re(r(j)))';
            rows = [rows; rr];                                        %#ok<AGROW>
            tc = [tc; cs(j) + min(ts(rr) - ts(rs(r(j))), cd(j))];     %#ok<AGROW>
            if ~strcmp(phase{rs(r(j))}, 'slide')
                h = numel(holds) + 1;
                holds(h).pass = passes(pi_);
                holds(h).pad  = to_l{rs(r(j))};
                holds(h).dir  = dirn{rs(r(j))};
                holds(h).dcc  = mean(frac_all(rr, :), 1);
            end
        end
        % pre-window
        if pi_ == 1
            pre = find(iter == passes(pi_) & (strcmp(phase, 'locate') | ...
                strcmp(phase, 'engage') | strcmp(phase, 'hold_start')));
        else
            pr = core{pi_ - 1};
            pre = (rs(pr(end)):re(pr(end)))';     % previous hold_return
        end
        rows = [rows; pre];
        tc = [tc; ts(pre) - t0(pi_)];
        % post-window: lift-off after the last pass
        post = find(iter == passes(pi_) & strcmp(phase, 'retract'));
        if ~isempty(post)
            rows = [rows; post];
            tc = [tc; t_core_end + ts(post) - ts(re(r(end)))];
        end

        keep = tc >= -t_pre & tc <= t_stop;
        rows = rows(keep); tc = tc(keep);
        for b = 1:nbin
            inb = tc >= edges(b) & tc < edges(b + 1);
            if ~any(inb), continue; end
            per_pass(pi_, b, :) = mean(frac_all(rows(inb), :), 1);
        end
    end

    dcc = nan(nbin, 19); q1 = nan(nbin, 19); q3 = nan(nbin, 19);
    for b = 1:nbin
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

    % ---- block list on the canonical axis --------------------------------
    r = core{1};
    blocks = struct('phase', {}, 'from', {}, 'to', {}, 'dir', {}, ...
                    't_start', {}, 't_end', {});
    path_out = {from_l{rs(r(1))}};
    for j = 1:nblk
        i0 = rs(r(j));
        blocks(j).phase = phase{i0};
        blocks(j).from  = from_l{i0};
        blocks(j).to    = to_l{i0};
        blocks(j).dir   = dirn{i0};
        blocks(j).t_start = cs(j);
        blocks(j).t_end   = cs(j) + cd(j);
        if strcmp(phase{i0}, 'slide') && strcmp(dirn{i0}, 'fwd')
            path_out{end + 1} = to_l{i0};   %#ok<AGROW>
        end
    end

    T.path_out = path_out;
    T.blocks = blocks;
    T.t_centre = centres;
    T.dcc = dcc; T.dcc_q1 = q1; T.dcc_q3 = q3;
    T.n_pass = np;
    T.t_stop = t_stop;
    T.t_core_end = t_core_end;
    e = find(iter == passes(1) & strcmp(phase, 'engage'), 1);
    if isempty(e), T.t_engage = -t_pre; else, T.t_engage = ts(e) - t0(1); end
    sl = strcmp(phase, 'slide');
    d = depth(sl); s = speed(sl);
    T.depth_mm = d(1); T.speed_mm_s = s(1);
    T.holds = holds;
end
