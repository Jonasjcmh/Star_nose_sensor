function hex_map(ax, values, target_idx, title_str, vmax)

POINTS = [
    -8 14; 0 14; 8 14;
    -12 7; -4 7; 4 7; 12 7;
    -16 0; -8 0; 0 0; 8 0; 16 0;
    -12 -7; -4 -7; 4 -7; 12 -7;
    -8 -14; 0 -14; 8 -14];

cla(ax); hold(ax,'on');

theta = linspace(0,2*pi,7);

r = 3.8;        % smaller hex
scale = 1.15;   % more spacing

% Python-like colormap
cmap = [
    0.16 0.71 0.63
    0.20 0.90 0.40
    1.00 0.90 0.10
    1.00 0.45 0.00
    0.86 0.00 0.00
];

for i=1:19

    x = POINTS(i,1) * scale;
    y = POINTS(i,2) * scale;

    vx = x + r*cos(theta);
    vy = y + r*sin(theta);

    v = values(i)/max(vmax,1e-6);
    v = max(0,min(1,v));

    idx = max(1, min(5, round(v*4)+1));
    col = cmap(idx,:);

    patch(ax,vx,vy,col,'EdgeColor','w','LineWidth',0.5);

    if abs(values(i)) > 0.01
        text(ax,x,y,sprintf('%.2f',values(i)), ...
            'HorizontalAlignment','center','FontSize',6);
    end
end

axis(ax,'equal');
axis(ax,[-25 25 -23 23]);
axis(ax,'off');
title(ax,title_str);

end