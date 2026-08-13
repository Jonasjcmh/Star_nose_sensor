function [groups, counts, totals] = count_by_phase(group_vals, phase_vals, phases)
% COUNT_BY_PHASE  Row counts of group_vals broken down by phase.
%
%   group_vals : numeric column vector OR cellstr column, one entry per row
%   phase_vals : cellstr column of phase names, same length
%   phases     : cellstr of phase names to count, e.g. {'press','hold','retract'}
%
% Returns:
%   groups : unique group values (sorted), Nx1 (numeric or cellstr, matches
%            group_vals)
%   counts : N x numel(phases) count matrix
%   totals : N x 1, sum across phases
    mask = ismember(phase_vals, phases);
    g = group_vals(mask);
    p = phase_vals(mask);

    groups = unique(g);
    n = numel(groups);
    m = numel(phases);

    phase_masks = cell(1, m);
    for j = 1:m
        phase_masks{j} = strcmp(p, phases{j});
    end

    counts = zeros(n, m);
    for i = 1:n
        if iscell(groups)
            gi_mask = strcmp(g, groups{i});
        else
            gi_mask = (g == groups(i));
        end
        for j = 1:m
            counts(i, j) = sum(gi_mask & phase_masks{j});
        end
    end
    totals = sum(counts, 2);
end
