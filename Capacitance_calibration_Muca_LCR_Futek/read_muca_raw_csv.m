function d = read_muca_raw_csv(csv_path, raw_valid_max)
% READ_MUCA_RAW_CSV  Parses one mucaboard_data_raw session CSV (59 cols,
% same format used by simple_19points_analysis.m). Returns raw_cells/
% calib_raw in RAW COLUMN order (1..19), untranslated -- caller applies
% TRUE_TO_CODE.
    fid = fopen(csv_path, 'r');
    if fid < 0
        error('read_muca_raw_csv:notfound', 'CSV not found: %s', csv_path);
    end
    header_line  = fgetl(fid);
    column_names = strsplit(header_line, ',', 'CollapseDelimiters', false);
    text_rows = {};
    while true
        line = fgetl(fid);
        if ~ischar(line)
            break;
        end
        text_rows{end + 1} = strsplit(line, ',', 'CollapseDelimiters', false); %#ok<AGROW>
    end
    fclose(fid);
    n_rows = numel(text_rows);
    data_text = vertcat(text_rows{:});

    column_index = @(name) find(strcmp(column_names, name));
    get_numbers  = @(name) str2double(data_text(:, column_index(name)));

    d.ur5_point   = get_numbers('ur5_point');
    d.round_idx   = get_numbers('round_idx');
    d.phase       = data_text(:, column_index('phase'));
    d.load_cell_N = get_numbers('load_cell_N');
    d.tcp_x       = get_numbers('tcp_x');
    d.tcp_y       = get_numbers('tcp_y');
    d.tcp_z       = get_numbers('tcp_z');

    d.raw_cells = nan(n_rows, 19);
    for k = 1:19
        d.raw_cells(:, k) = get_numbers(sprintf('cell_%d', k));
    end
    d.calib_raw = nan(1, 19);
    for k = 1:19
        one_col = get_numbers(sprintf('calib_%d', k));
        d.calib_raw(k) = one_col(1);
    end
    n_bad = sum(d.raw_cells(:) > raw_valid_max);
    d.raw_cells(d.raw_cells > raw_valid_max) = NaN;
    d.calib_raw(d.calib_raw > raw_valid_max) = NaN;
    d.n_rows = n_rows;
    d.n_bad  = n_bad;
end
