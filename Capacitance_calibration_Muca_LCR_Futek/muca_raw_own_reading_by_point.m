function rows = muca_raw_own_reading_by_point(d, true_to_code, n_iterations, sensitivity, gamma)
% MUCA_RAW_OWN_READING_BY_POINT  For every TRUE point 1-19: mean hold-phase
% (force_N, raw_own, V_own) at this file's single depth, pooling round_idx
% 0..n_iterations-1. Returns 19x4: [true_point, force_N, raw_own, V_own]
% (NaN row if no hold data).
    rows = nan(19, 4);
    for true_pt = 1:19
        code_id = true_to_code(true_pt);
        mask = (d.ur5_point == code_id) & strcmp(d.phase, 'hold') & ...
               (d.round_idx >= 0) & (d.round_idx <= n_iterations - 1);
        if ~any(mask)
            continue;
        end
        raw_own = mean(d.raw_cells(mask, code_id), 'omitnan');
        force   = mean(d.load_cell_N(mask), 'omitnan');
        calib   = d.calib_raw(code_id);
        ratio = min(max((raw_own - calib) / sensitivity, 0), 1);
        v_own = ratio ^ gamma;
        rows(true_pt, :) = [true_pt, force, raw_own, v_own];
    end
end
