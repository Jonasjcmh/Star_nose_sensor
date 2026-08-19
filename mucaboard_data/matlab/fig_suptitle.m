function fig_suptitle(fig, txt)
% FIG_SUPTITLE  Figure-wide title, working on both MATLAB (sgtitle, R2018b+)
% and older Octave (no sgtitle) via a manual invisible-axes textbox fallback.
    try
        sgtitle(fig, txt, 'FontWeight', 'bold', 'Interpreter', 'none');
    catch
        ax = axes('Parent', fig, 'Position', [0 0 1 1], 'Visible', 'off');
        text(ax, 0.5, 0.98, txt, 'Units', 'normalized', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
            'FontWeight', 'bold', 'FontSize', 13, 'Interpreter', 'none');
    end
end
