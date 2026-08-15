function matched = match_lcr_muca_by_depth(lcr_rows, muca_rows, depth_offset)
% MATCH_LCR_MUCA_BY_DEPTH  Pair up LCR and mucaboard per-(point,depth) stats
% by exact (point, depth) match, after adding depth_offset to the LCR
% depth. depth_offset=0 for flat/solid (same depth labels in both sessions);
% depth_offset=5 for hollow (the LCR hollow session's depth_mm=0 turns out
% to correspond to the mucaboard session's depth_mm=5 -- confirmed by the
% two sessions' load_cell_N values matching closely at that offset).
%
% lcr_rows, muca_rows: Nx4 [point, depth_mm, mean_force_N, value] (value is
% Cp_pF for lcr_rows, own-cell normalized reading for muca_rows).
%
% Returns Mx6: [point, depth_mm_muca, lcr_force_N, muca_force_N, Cp_pF, muca_value].
    matched = zeros(0, 6);
    for i = 1:size(lcr_rows, 1)
        pt = lcr_rows(i, 1);
        d_target = lcr_rows(i, 2) + depth_offset;
        j = find(muca_rows(:, 1) == pt & abs(muca_rows(:, 2) - d_target) < 1e-6, 1);
        if ~isempty(j)
            matched(end + 1, :) = [pt, d_target, lcr_rows(i, 3), muca_rows(j, 3), ...
                lcr_rows(i, 4), muca_rows(j, 4)]; %#ok<AGROW>
        end
    end
end
