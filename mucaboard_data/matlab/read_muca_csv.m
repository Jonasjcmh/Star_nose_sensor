function d = read_muca_csv(csv_path)
% READ_MUCA_CSV  Parse a mucaboard_ramp_collector.py session CSV with
% textscan (no readtable/io-package dependency -- works in stock MATLAB and
% in Octave without the io forge package, which fails to build in some
% environments). Column layout is fixed/known (see header check below).
%
% Returns a struct with one field per column of interest (each Nx1, or Nx19
% for .cells), plus .n_rows and a derived, calibrated, zeroed force channel
% .force_N (see ai0_to_force_N.m + the zeroing note below).
    fid = fopen(csv_path, 'r');
    if fid < 0
        error('read_muca_csv:notfound', 'CSV not found: %s', csv_path);
    end
    header_line = fgetl(fid);
    header = strsplit(header_line, ',');
    if numel(header) ~= 39
        fprintf('  [warn] expected 39 CSV columns, found %d in %s -- column layout may differ\n', ...
            numel(header), csv_path);
    end

    % 1:timestamp 2:datetime(skip) 3-15:ur5_point..ai0 (13 numeric)
    % 16-34:cell_1..cell_19 (19) 35:depth_mm 36:phase(str) 37-39:round_idx,sample_idx,load_cell_N
    fmt = ['%f' '%*s' repmat('%f', 1, 13) repmat('%f', 1, 19) '%f' '%s' repmat('%f', 1, 3)];
    C = textscan(fid, fmt, 'Delimiter', ',', 'Whitespace', '');
    fclose(fid);

    d.timestamp    = C{1};
    d.ur5_point    = C{2};
    d.ur5_pressing = C{3};
    d.ai0          = C{14};
    d.cells        = [C{15:33}];
    d.depth_mm     = C{34};
    d.phase        = C{35};
    d.round_idx    = C{36};
    d.load_cell_N  = C{38};   % as logged by the collector (same formula, unzeroed)
    d.n_rows       = numel(d.timestamp);

    % Recompute from ai0 via the deployed FUTEK calibration (ai0_to_force_N.m,
    % same formula as Integration_2/analyze_session.py's ai0_to_newtons()),
    % then zero so 0 N = no load -- same convention analyze_session.py uses
    % in plot_loadcell_vs_robot() (lc_zero = min of the converted signal over
    % the whole session).
    force_raw  = ai0_to_force_N(d.ai0);
    d.force_zero_N = min(force_raw);
    d.force_N  = force_raw - d.force_zero_N;
end
