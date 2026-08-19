function round_traces = muca_force_vs_time_rounds(d, depth_mm, max_round, phases, point_id)
% MUCA_FORCE_VS_TIME_ROUNDS  Individual per-iteration force-vs-time traces
% (not pooled/binned like muca_force_vs_time.m) for point_id's press events
% at depth_mm, round_idx in [0, max_round-1], phase in `phases`.
%
% Returns a 1xN cell array (N = number of rounds found), each cell a
% struct with fields .t (time since that round's own press onset, sorted)
% and .f (calibrated, zeroed force -- d.force_N -- at those times), for
% plotting each iteration as its own line.
    mask = ismember(d.phase, phases) & (d.depth_mm == depth_mm) ...
         & (d.round_idx >= 0) & (d.round_idx <= max_round - 1) ...
         & (d.ur5_point == point_id);

    round_traces = {};
    if ~any(mask)
        return;
    end

    rounds = d.round_idx(mask);
    ts     = d.timestamp(mask);
    fz     = d.force_N(mask);

    ukeys = unique(rounds);
    for i = 1:numel(ukeys)
        gi = (rounds == ukeys(i));
        t0 = min(ts(gi));
        trel = ts(gi) - t0;
        f = fz(gi);
        [trel_sorted, order] = sort(trel);
        round_traces{end + 1} = struct('t', trel_sorted, 'f', f(order)); %#ok<AGROW>
    end
end
