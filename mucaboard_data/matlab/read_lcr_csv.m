function d = read_lcr_csv(csv_path)
% READ_LCR_CSV  Parse an LCR-meter-connected capacitance session CSV from
% Capacitance_measurement/logs (produced by capacitance_ramp_collector.py /
% capacitance_two_point_iterations_collector.py). Two slightly different
% schemas exist in that folder (ramp_collector_*.csv has columns
% ...,round_idx,sample_idx,point,depth_mm,phase,...; two_point_iterations_*.csv
% has ...,point_idx,point,depth_mm,iter_idx,phase,...) -- column POSITIONS
% differ between them, so this reads every column as a string first and
% looks up the ones we need BY NAME by header, rather than assuming a fixed
% position (unlike read_muca_csv.m, where the schema is single/fixed and a
% positional textscan format is fine).
%
% Returns a struct with fields: .point (numeric), .depth_mm, .phase (cellstr),
% .load_cell_N, .Cp_pF, .n_rows.
    fid = fopen(csv_path, 'r');
    if fid < 0
        error('read_lcr_csv:notfound', 'CSV not found: %s', csv_path);
    end
    header_line = fgetl(fid);
    header = strsplit(header_line, ',');
    ncols = numel(header);

    fmt = repmat('%s', 1, ncols);
    C = textscan(fid, fmt, 'Delimiter', ',', 'Whitespace', '');
    fclose(fid);

    col_idx = @(name) find(strcmp(header, name), 1);

    d.point         = str2double(C{col_idx('point')});
    d.depth_mm      = str2double(C{col_idx('depth_mm')});
    d.phase         = C{col_idx('phase')};
    d.load_cell_N   = str2double(C{col_idx('load_cell_N')});
    d.Cp_pF         = str2double(C{col_idx('Cp_pF')});
    d.n_rows        = numel(d.point);
end
