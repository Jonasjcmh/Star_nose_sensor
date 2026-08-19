function [t_rel, force, t_bin, stats, n_iters_used] = muca_force_vs_time(d, depth_mm, max_round, phases, point_id)
% MUCA_FORCE_VS_TIME  Calibrated, zeroed force (d.force_N -- see
% read_muca_csv.m / ai0_to_force_N.m) vs time-since-press-onset for ONE
% point's press event(s) (point_id) at depth_mm, round_idx in
% [0, max_round-1], phase in `phases` (e.g. {'press','hold','retract'}).
%
% Each event (one per iteration/round) is time-aligned to its own start
% (t=0 at the first matching row), so multiple rounds compare on the same
% ramp-hold-release clock. Also returns 0.1s-binned summary traces (struct
% `stats`, fields .mean/.median/.q1/.q3, each 1 x numel(t_bin)) for clean
% summary lines/bands on top of the raw (noisy, overlapping) points -- with
% a single point_id there are only max_round events to overlay, not 19x.
%
% n_iters_used = number of distinct round_idx actually found (<= max_round;
% can be less if some rounds are missing this depth/point in the data).
    mask = ismember(d.phase, phases) & (d.depth_mm == depth_mm) ...
         & (d.round_idx >= 0) & (d.round_idx <= max_round - 1) ...
         & (d.ur5_point == point_id);

    t_rel = [];
    force = [];
    n_iters_used = 0;
    if ~any(mask)
        t_bin = [];
        stats = struct('mean', [], 'median', [], 'q1', [], 'q3', []);
        return;
    end

    pts    = d.ur5_point(mask);
    rounds = d.round_idx(mask);
    ts     = d.timestamp(mask);
    fz     = d.force_N(mask);

    n_iters_used = numel(unique(rounds));

    keys  = pts * 1000 + rounds;   % unique per (point, round) -- rounds are small ints
    ukeys = unique(keys);
    for i = 1:numel(ukeys)
        gi = (keys == ukeys(i));
        t0 = min(ts(gi));
        t_rel = [t_rel; ts(gi) - t0]; %#ok<AGROW>
        force = [force; fz(gi)];      %#ok<AGROW>
    end

    edges = 0:0.1:(max(t_rel) + 0.1);
    if numel(edges) < 2
        edges = [0, max(t_rel) + 0.1];
    end
    t_bin = edges(1:end-1) + diff(edges) / 2;
    n = numel(t_bin);
    stats.mean   = nan(1, n);
    stats.median = nan(1, n);
    stats.q1     = nan(1, n);
    stats.q3     = nan(1, n);
    for b = 1:n
        bm = t_rel >= edges(b) & t_rel < edges(b + 1);
        if any(bm)
            vals = force(bm);
            stats.mean(b)   = mean(vals);
            stats.median(b) = median(vals);
            stats.q1(b)     = simple_percentile(vals, 25);
            stats.q3(b)     = simple_percentile(vals, 75);
        end
    end
end
