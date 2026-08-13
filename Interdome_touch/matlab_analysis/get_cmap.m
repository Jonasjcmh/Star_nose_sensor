function cmap = get_cmap(kind, n)
% GET_CMAP  Colormap matrix (n x 3) for hex-schematic plots.
%
% 'viridis' exists in Octave but not stock MATLAB; 'parula' is the reverse.
% Try both so this runs unmodified in either, falling back to 'jet' (present
% in both) if neither is available.
    if nargin < 2
        n = 256;
    end
    switch kind
        case 'capacitive'
            try
                cmap = viridis(n);
            catch
                try
                    cmap = parula(n);
                catch
                    cmap = jet(n);
                end
            end
        case 'force'
            cmap = hot(n);
        otherwise
            cmap = jet(n);
    end
end
