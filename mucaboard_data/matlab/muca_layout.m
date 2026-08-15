function [point_ids, point_xy, own_cell_col] = muca_layout()
% MUCA_LAYOUT  Physical hex-grid layout and own-cell mapping for the
% mucaboard 19-point sensor, mirrored from:
%   - mucaboard_data/mucaboard_ramp_collector.py: POINTS (x,y mm per point)
%   - Integration_2/analyze_session.py: UR5_TO_IDX (point -> cell array index)
%
% Unlike Interdome_touch, this board has no rotation/JSON calibration file --
% point IDs already line up 1:1 with hexagon position and there are only the
% 19 main points (no interpolated diagonal/triangle/horizontal points).
%
% Returns:
%   point_ids    : 19x1, [1;2;...;19]
%   point_xy     : 19x2, (x_mm, y_mm) per point, row k = point_ids(k)
%   own_cell_col : 19x1, CSV column number (cell_<own_cell_col(k)>) holding
%                  point_ids(k)'s own capacitive reading
    point_ids = (1:19)';

    point_xy = [
        -8,  14;    0,  14;    8,  14;
        -12,  7;   -4,   7;    4,   7;   12,  7;
        -16,  0;   -8,   0;    0,   0;    8,  0;   16,  0;
        -12, -7;   -4,  -7;    4,  -7;   12, -7;
         -8,-14;    0, -14;    8, -14;
    ];

    own_cell_col = [17,13,8,18,14,9,4,19,15,10,5,1,16,11,6,2,12,7,3]';
end
