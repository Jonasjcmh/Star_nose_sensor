function [timestamp_col, loaded_col, ai0_col, fz_col] = read_measurement_columns(csv_path)
% READ_MEASUREMENT_COLUMNS  Read just the four columns Step 2 needs
% ("timestamp", "loaded", "ai0", "fz") from one recording CSV. Column
% positions are taken from the header line, so column order in the file
% does not matter.
%
% Kept as its own small file (instead of a local function inside
% step2_ur_force_vs_time.m) so the script also runs on GNU Octave, which
% resolves script-local functions differently than MATLAB -- same
% pattern as read_loaded_and_ai0.m for Step 1.

    fid = fopen(csv_path, 'r');
    header = strsplit(fgetl(fid), ',');
    fmt = repmat({'%f'}, 1, numel(header));
    fmt{strcmp(header, 'datetime')} = '%s';   % the only non-numeric column
    cols = textscan(fid, strjoin(fmt, ''), 'Delimiter', ',');
    fclose(fid);

    timestamp_col = cols{strcmp(header, 'timestamp')};
    loaded_col    = cols{strcmp(header, 'loaded')};
    ai0_col       = cols{strcmp(header, 'ai0')};
    fz_col        = cols{strcmp(header, 'fz')};
end
