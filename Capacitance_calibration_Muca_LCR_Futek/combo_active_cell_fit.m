function combo = combo_active_cell_fit(csv_path)
% COMBO_ACTIVE_CELL_FIT  Approach A: parses one
% capacitance_combination_calibration session CSV. For every (combination,
% point_label): finds the "active" cell as the one whose muca-phase mean
% deviates most from that SAME cell's mean across the OTHER point_labels in
% the SAME combination (the capacitor moves by hand between readings, so
% only the currently-wired cell should differ from its own quiescent
% level). Pairs that active cell's raw value with the combination's own
% LCR-phase mean Cp_pF (ground truth, measured once per combination,
% independent of which muca point it's later logged as).
%
% Returns a struct: .raw (Nx1), .Cp_pF (Nx1), .combo_label (Nx1 cellstr),
% .point_label (Nx1 cellstr), .active_cell (Nx1), .n_saturated (how many
% of the N raw readings hit the RAW_VALID_MAX-style ceiling, i.e. >1000).
    fid = fopen(csv_path, 'r');
    if fid < 0
        error('combo_active_cell_fit:notfound', 'CSV not found: %s', csv_path);
    end
    header_line = fgetl(fid);
    header = strsplit(header_line, ',', 'CollapseDelimiters', false);
    ncols = numel(header);
    fmt = repmat('%s', 1, ncols);
    C = textscan(fid, fmt, 'Delimiter', ',', 'Whitespace', '');
    fclose(fid);
    col = @(name) C{find(strcmp(header, name), 1)};

    phase   = col('phase');
    combo_i = col('combination_index');
    combo_l = col('combination_label');
    pt_l    = col('point_label');
    Cp_pF   = str2double(col('Cp_pF'));
    cells = nan(numel(phase), 19);
    for k = 1:19
        cells(:, k) = str2double(col(sprintf('cell_%d', k)));
    end

    % ---- one ground-truth Cp_pF per combination (mean of its lcr rows) --
    is_lcr = strcmp(phase, 'lcr');
    combo_ids = unique(combo_i(is_lcr));
    combo_cp = containers.Map();
    combo_label_map = containers.Map();
    for i = 1:numel(combo_ids)
        cid = combo_ids{i};
        m = is_lcr & strcmp(combo_i, cid);
        combo_cp(cid) = mean(Cp_pF(m), 'omitnan');
        lbls = combo_l(m);
        combo_label_map(cid) = lbls{1};
    end

    % ---- per (combo, point_label) mean cell vector, muca phase only ----
    % Subset everything to the muca-phase rows FIRST so every array below
    % shares the same length/indexing.
    is_muca    = strcmp(phase, 'muca');
    m_combo_i  = combo_i(is_muca);
    m_pt_l     = pt_l(is_muca);
    m_cells    = cells(is_muca, :);
    pair_id    = strcat(m_combo_i, {'||'}, m_pt_l);
    upairs     = unique(pair_id);

    n = numel(upairs);
    raw_out = nan(n, 1);
    cp_out  = nan(n, 1);
    combo_label_out = cell(n, 1);
    point_label_out = cell(n, 1);
    active_cell_out = nan(n, 1);

    for i = 1:n
        m = strcmp(pair_id, upairs{i});
        cid = m_combo_i(find(m, 1));
        cid = cid{1};
        mean_cells = mean(m_cells(m, :), 1, 'omitnan');

        % other point_labels in the SAME combination
        m_other = strcmp(m_combo_i, cid) & ~strcmp(pair_id, upairs{i});
        if any(m_other)
            other_mean = mean(m_cells(m_other, :), 1, 'omitnan');
        else
            other_mean = zeros(1, 19);
        end
        dev = abs(mean_cells - other_mean);
        [~, active_k] = max(dev);

        raw_out(i) = mean_cells(active_k);
        if isKey(combo_cp, cid)
            cp_out(i) = combo_cp(cid);
            combo_label_out{i} = combo_label_map(cid);
        end
        pl = m_pt_l(find(m, 1));
        point_label_out{i} = pl{1};
        active_cell_out(i) = active_k;
    end

    combo.raw          = raw_out;
    combo.Cp_pF         = cp_out;
    combo.combo_label   = combo_label_out;
    combo.point_label   = point_label_out;
    combo.active_cell   = active_cell_out;
    combo.n_saturated   = sum(raw_out > 1000);
end
