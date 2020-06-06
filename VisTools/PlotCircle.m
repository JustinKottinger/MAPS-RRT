% source: https://www.mathworks.com/matlabcentral/answers/3058-plotting-circles
function PlotCircle(Goal, NumVehicles, Dim, r, Colors)
%x and y are the coordinates of the center of the circle
%r is the radius of the circle
%0.01 is the angle step, bigger values will draw the circle faster but
%you might notice imperfections (not very smooth)
ang=0:0.01:2*pi; 
xp=r*cos(ang);
yp=r*sin(ang);
hold on;
for j = 1 : NumVehicles
    x = Goal((Dim * (j - 1)) + 1);
    y = Goal((Dim * (j - 1)) + 2);
    fill(x + xp, y + yp, Colors{j});
    plot(x+xp,y+yp, 'w');
end

