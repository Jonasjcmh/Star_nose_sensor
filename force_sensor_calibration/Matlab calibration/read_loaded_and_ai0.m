function [loaded_col, ai0_col] = read_loaded_and_ai0(csv_path)
% READ_LOADED_AND_AI0  Read just the two columns Step 1 needs ("loaded"
% and "ai0") from one recording CSV. Column positions are taken from
% the header line, so column order in the file does not matter.
%
% Kept as its own small file (instead of a local function inside
% step1_loadcell_calibration.m) so the script also runs on GNU Octave,
% which resolves script-local functions differently than MATLAB.

    fid = fopen(csv_path, 'r');
    header = strsplit(fgetl(fid), ',');
    fmt = repmat({'%f'}, 1, numel(header));
    fmt{strcmp(header, 'datetime')} = '%s';   % the only non-numeric column
    cols = textscan(fid, strjoin(fmt, ''), 'Delimiter', ',');
    fclose(fid);

    loaded_col = cols{strcmp(header, 'loaded')};
    ai0_col    = cols{strcmp(header, 'ai0')};
end
