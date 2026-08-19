function [t_press_end, t_hold_end, t_total_end] = muca_phase_boundaries(d, depth_mm, max_round, point_id)
% MUCA_PHASE_BOUNDARIES  Mean time-since-press-onset at which each phase
% ends, pooled over the same (point, round) events used by
% muca_force_vs_time.m -- for shading the press/hold/retract regions on the
% force-vs-time plots. Phase durations are robot-commanded (ramp_s/hold_s),
% not surface-dependent, so these boundaries are meant to be computed once
% per point and reused across surfaces (the caller can average across
% whichever surfaces have data for a slightly more robust estimate).
    mask = ismember(d.phase, {'press', 'hold', 'retract'}) & (d.depth_mm == depth_mm) ...
         & (d.round_idx >= 0) & (d.round_idx <= max_round - 1) ...
         & (d.ur5_point == point_id);

    t_press_end = NaN;
    t_hold_end  = NaN;
    t_total_end = NaN;
    if ~any(mask)
        return;
    end

    rounds = d.round_idx(mask);
    ts     = d.timestamp(mask);
    ph     = d.phase(mask);

    ukeys = unique(rounds);
    press_ends  = [];
    hold_ends   = [];
    total_ends  = [];
    for i = 1:numel(ukeys)
        gi = (rounds == ukeys(i));
        t0   = min(ts(gi));
        trel = ts(gi) - t0;
        phi  = ph(gi);
        pm = strcmp(phi, 'press');
        hm = strcmp(phi, 'hold');
        rm = strcmp(phi, 'retract');
        if any(pm)
            press_ends(end + 1) = max(trel(pm)); %#ok<AGROW>
        end
        if any(hm)
            hold_ends(end + 1) = max(trel(hm)); %#ok<AGROW>
        end
        if any(rm)
            total_ends(end + 1) = max(trel(rm)); %#ok<AGROW>
        end
    end
    if ~isempty(press_ends);  t_press_end = mean(press_ends);  end
    if ~isempty(hold_ends);   t_hold_end  = mean(hold_ends);   end
    if ~isempty(total_ends);  t_total_end = mean(total_ends);  end
end
