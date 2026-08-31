function [cp_est, extrapolated] = interp_force_to_cp(force_cp_pairs, target_force)
% INTERP_FORCE_TO_CP  force_cp_pairs: Mx2 [force_N, Cp_pF] (one point's LCR
% depth sweep). Sorts by force and linearly interpolates/extrapolates
% Cp_pF at target_force. extrapolated=true if target_force fell outside
% the sweep's observed force range.
    if size(force_cp_pairs, 1) < 2
        cp_est = NaN;
        extrapolated = true;
        return;
    end
    [f_sorted, order] = sort(force_cp_pairs(:, 1));
    cp_sorted = force_cp_pairs(order, 2);
    % de-duplicate identical force values (interp1 needs strictly
    % increasing sample points) by averaging their Cp
    [f_unique, ~, ic] = unique(f_sorted);
    cp_unique = accumarray(ic, cp_sorted, [], @mean);
    if numel(f_unique) < 2
        cp_est = NaN;
        extrapolated = true;
        return;
    end
    cp_est = interp1(f_unique, cp_unique, target_force, 'linear', 'extrap');
    extrapolated = target_force < min(f_unique) || target_force > max(f_unique);
end
