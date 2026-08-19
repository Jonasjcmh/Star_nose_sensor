function save_fig(fig, out_path_no_ext)
% SAVE_FIG  Save fig as both PNG (raster, -r150) and SVG (vector) using the
% same base path (no extension). Prints one "saved:" line per file.
    png_path = [out_path_no_ext '.png'];
    svg_path = [out_path_no_ext '.svg'];
    print(fig, png_path, '-dpng', '-r150');
    fprintf('  saved: %s\n', png_path);
    print(fig, svg_path, '-dsvg');
    fprintf('  saved: %s\n', svg_path);
end
