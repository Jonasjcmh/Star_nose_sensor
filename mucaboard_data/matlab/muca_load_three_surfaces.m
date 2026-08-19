function data = muca_load_three_surfaces(flat_csv, solid_csv, hollow_csv)
% MUCA_LOAD_THREE_SURFACES  Thin convenience wrapper around read_muca_csv.m
% for all 3 surfaces at once.
%
% Returns data.flat / data.solid / data.hollow, each the raw struct from
% read_muca_csv.m (timestamps, ur5_point, depth_mm, phase, round_idx,
% .cells in raw CSV column order, etc). Combine with muca_layout.m's
% own_cell_col + muca_extract_point_segments.m to pull a specific point's
% pressed/released groups.
    data.flat   = read_muca_csv(flat_csv);
    data.solid  = read_muca_csv(solid_csv);
    data.hollow = read_muca_csv(hollow_csv);
end
