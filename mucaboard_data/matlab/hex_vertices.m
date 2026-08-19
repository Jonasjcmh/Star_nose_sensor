function [vx, vy] = hex_vertices(x, y, r)
% HEX_VERTICES  6 vertices of a pointy-top regular hexagon centered (x,y).
% Matches matplotlib's RegularPolygon(orientation=0): first vertex at 90 deg,
% one vertex every 60 deg, so hexagons drawn here line up with the Python
% (Interdome_touch/analyze_interdome.py) trajectory/hex-schematic figures.
    angles = deg2rad(90 + 60 * (0:5));
    vx = x + r * cos(angles);
    vy = y + r * sin(angles);
end
