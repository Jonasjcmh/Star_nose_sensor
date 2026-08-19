function seg = muca_extract_point_segments(d, point_id, own_cell_col, max_round, depth_mm, pressed_phases, released_phases)
% MUCA_EXTRACT_POINT_SEGMENTS  Raw, UN-REDUCED 19-cell readings for one
% point's press cycle, split into the two states you asked for -- no mean/
% min/max computed here, that part is yours.
%
%   .pressed  -- rows from the pressed_phases (default {'hold'}: pressing=
%                True, dwelling at depth_mm) -- the board's response while
%                point_id is loaded.
%   .released -- rows from the released_phases (default {'locate'}:
%                pressing=False, at the surface) -- the board's response
%                with point_id NOT loaded, i.e. "total release" for this
%                point's cycle.
%
% Both groups are pooled across ALL rounds 0..max_round-1 into one matrix
% each. Every row has all 19 cells, remapped through own_cell_col so
% column j is always physical point j's own reading (raw CSV column order
% is NOT point order -- see muca_layout.m). This is the same "full-board
% snapshot" shape muca_point_response.m uses for cross-talk plots, so
% seg.pressed.values(:, point_id) is that point's own signal, while
% seg.pressed.values(:, k) for k ~= point_id is what point k "feels"
% while point_id is being pressed.
%
% Inputs
%   d              : struct from read_muca_csv.m (one surface's session)
%   point_id       : which point's press cycle to pull (1-19)
%   own_cell_col   : 19x1 CSV-column-per-point map, from muca_layout.m
%   max_round      : use round_idx 0..max_round-1
%   depth_mm       : filter to this depth (use [] to skip filtering --
%                    needed for the *_3points_session_* logs, which only
%                    have one depth per round and no separate depth sweep)
%   pressed_phases : cell array of phase strings counting as "pressed"
%                    (default {'hold'}; try {'press','hold'} to include
%                    the ramp, or {'hold','retract'} etc.)
%   released_phases: cell array of phase strings counting as "released"
%                    (default {'locate'}; try {'locate','retract','post'}
%                    for a broader "not currently loaded" definition)
%
% Returns seg, a struct:
%   seg.point_id            = point_id
%   seg.pressed.values      = Rx19 matrix (R = total pressed-phase rows found)
%   seg.pressed.round_idx   = Rx1 (which round each row came from)
%   seg.released.values     = Sx19 matrix (S = total released-phase rows found)
%   seg.released.round_idx  = Sx1
%   seg.n_rounds_pressed / seg.n_rounds_released = number of DISTINCT
%       rounds that contributed >=1 row to that group (<= max_round)
%
% Example
%   [~, ~, own_cell_col] = muca_layout();
%   d = read_muca_csv('flat_3points_session_....csv');
%   seg = muca_extract_point_segments(d, 3, own_cell_col, 10, 10);
%   my_v0   = mean(seg.released.values(:, 3));   % point 3's own release level
%   my_hold = mean(seg.pressed.values(:, 3));    % point 3's own loaded level
%   crosstalk_while_p3_pressed = mean(seg.pressed.values, 1);  % 1x19, all cells
    if nargin < 5
        depth_mm = [];
    end
    if nargin < 6 || isempty(pressed_phases)
        pressed_phases = {'hold'};
    end
    if nargin < 7 || isempty(released_phases)
        released_phases = {'locate'};
    end

    mask_point = (d.ur5_point == point_id) & (d.round_idx >= 0) & (d.round_idx <= max_round - 1);
    if ~isempty(depth_mm)
        mask_point = mask_point & (d.depth_mm == depth_mm);
    end

    mask_pressed  = mask_point & ismember(d.phase, pressed_phases);
    mask_released = mask_point & ismember(d.phase, released_phases);

    seg.point_id = point_id;
    seg.pressed.values      = d.cells(mask_pressed, own_cell_col);
    seg.pressed.round_idx   = d.round_idx(mask_pressed);
    seg.released.values     = d.cells(mask_released, own_cell_col);
    seg.released.round_idx  = d.round_idx(mask_released);
    seg.n_rounds_pressed  = numel(unique(seg.pressed.round_idx));
    seg.n_rounds_released = numel(unique(seg.released.round_idx));
end
