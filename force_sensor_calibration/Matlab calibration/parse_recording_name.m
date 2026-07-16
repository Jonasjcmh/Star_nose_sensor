function info = parse_recording_name(fname)
% PARSE_RECORDING_NAME  Pull direction / weight / version / timestamp out
% of a futek_direct recording filename, using plain string splitting (no
% regular expressions -- MATLAB's regexp engine silently fails to match
% optional tagged groups, which twice made this discovery find nothing).
%
% Accepted forms:
%   fzcal_futek_direct_posz_100g_v2_20260715_153459.csv   (direction in name)
%   fzcal_futek_direct_100g_v2_20260715_160629.csv        (no direction)
%   fzcal_futek_direct_negz_100g_20260703_174340.csv      (no version tag -> 'v1')
%
% Returns a struct with fields direction ('' when not in the name),
% weight_g, version, ts -- or [] if the filename is not a recording.

    info = [];
    prefix = 'fzcal_futek_direct_';
    if length(fname) <= length(prefix) + 4 || ~strncmp(fname, prefix, length(prefix)) ...
            || ~strcmpi(fname(end-3:end), '.csv')
        return
    end
    parts = strsplit(fname(length(prefix)+1 : end-4), '_');

    % 1) direction, if present in the name
    direction = '';
    if ~isempty(parts) && (strcmp(parts{1}, 'posz') || strcmp(parts{1}, 'negz'))
        direction = parts{1};
        parts(1) = [];
    end

    % 2) weight: '<number>g'
    if isempty(parts)
        return
    end
    w = parts{1};
    if length(w) < 2 || w(end) ~= 'g'
        return
    end
    weight_g = str2double(w(1:end-1));
    if isnan(weight_g)
        return
    end
    parts(1) = [];

    % 3) version tag 'v<digits>', if present; files without one are 'v1'
    version = 'v1';
    if ~isempty(parts) && length(parts{1}) >= 2 && parts{1}(1) == 'v' ...
            && all(isstrprop(parts{1}(2:end), 'digit'))
        version = parts{1};
        parts(1) = [];
    end

    % 4) timestamp: 'YYYYMMDD' then 'HHMMSS'
    if numel(parts) ~= 2 ...
            || length(parts{1}) ~= 8 || ~all(isstrprop(parts{1}, 'digit')) ...
            || length(parts{2}) ~= 6 || ~all(isstrprop(parts{2}, 'digit'))
        return
    end

    info.direction = direction;
    info.weight_g  = weight_g;
    info.version   = version;
    info.ts        = [parts{1} '_' parts{2}];
end
