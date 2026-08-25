function d = read_muca_csv_central(csv_path)
% READ_MUCA_CSV_CENTRAL  Parse a "raw data central point" session CSV --
% a DIFFERENT, richer column layout than read_muca_csv.m's format:
%
%   timestamp,datetime,ur5_point,ur5_label,ur5_pressing,ur5_done,
%   tcp_x,tcp_y,tcp_z,fx,fy,fz,tx,ty,tz,ai0,
%   cell_1..cell_19,          <- RAW touch-controller counts (NOT [0,1]!)
%   calib_1..calib_19,        <- per-cell baseline, constant across the
%                                 whole session (captured once at connect)
%   depth_mm,phase,round_idx,sample_idx,load_cell_N
%
% Confirmed empirically (not assumed): cell_i values sit in the same
% small-integer range as calib_i (teens-to-thirties), and rise sharply
% during 'hold' vs 'locate' for the pressed point's own column -- this is
% pre-normalization raw data, unlike read_muca_csv.m's cell_i (which is
% already the sensor.py-normalized [0,1] V). Combined with calib_i (the
% baseline that's normally NEVER logged -- see README_signal_normalization.md
% section 5), this file lets you apply the sensitivity/gamma formula
% yourself and check it against a real baseline instead of only the
% partial reverse-calculation.
%
% Returns a struct:
%   .timestamp, .ur5_point, .ur5_pressing, .ai0   (Nx1 each)
%   .cells        Nx19, RAW CSV column order (remap via muca_layout.m's
%                 own_cell_col to get physical-point order, same as
%                 read_muca_csv.m's convention)
%   .calib        1x19, RAW CSV column order, session-constant (taken
%                 from row 1 -- verify assumption holds via
%                 unique(d.calib_all) if paranoid)
%   .depth_mm, .phase, .round_idx, .sample_idx, .load_cell_N
%   .force_N      calibrated + zeroed force channel (ai0_to_force_N.m),
%                 same convention as read_muca_csv.m
%   .n_rows
    fid = fopen(csv_path, 'r');
    if fid < 0
        error('read_muca_csv_central:notfound', 'CSV not found: %s', csv_path);
    end
    header_line = fgetl(fid);
    header = strsplit(header_line, ',');
    if numel(header) ~= 59
        fprintf('  [warn] expected 59 CSV columns, found %d in %s -- column layout may differ\n', ...
            numel(header), csv_path);
    end

    % Raw CSV column -> captured-cell index (skipped columns, marked with
    % %*s, consume no C{} slot):
    %   1:timestamp->C{1}  2:datetime(skip)  3:ur5_point->C{2}
    %   4:ur5_label(skip)  5:ur5_pressing->C{3}  6:ur5_done->C{4}
    %   7-15:tcp_x,tcp_y,tcp_z,fx,fy,fz,tx,ty,tz (9) -> C{5..13}
    %   16:ai0->C{14}  17-35:cell_1..19 (19) -> C{15..33}
    %   36-54:calib_1..19 (19) -> C{34..52}  55:depth_mm->C{53}
    %   56:phase(str)->C{54}  57:round_idx->C{55}  58:sample_idx->C{56}
    %   59:load_cell_N->C{57}
    fmt = ['%f' '%*s' '%f' '%*s' '%f' '%f' repmat('%f', 1, 9) '%f' ...
           repmat('%f', 1, 19) repmat('%f', 1, 19) '%f' '%s' repmat('%f', 1, 3)];
    C = textscan(fid, fmt, 'Delimiter', ',', 'Whitespace', '');
    fclose(fid);

    d.timestamp    = C{1};
    d.ur5_point    = C{2};
    d.ur5_pressing = C{3};
    d.ai0          = C{14};
    d.cells        = [C{15:33}];
    d.calib        = cellfun(@(c) c(1), C(34:52));   % 1x19, session-constant -> take row 1
    d.depth_mm     = C{53};
    d.phase        = C{54};
    d.round_idx    = C{55};
    d.load_cell_N  = C{57};
    d.n_rows       = numel(d.timestamp);

    force_raw  = ai0_to_force_N(d.ai0);
    d.force_zero_N = min(force_raw);
    d.force_N  = force_raw - d.force_zero_N;
end
