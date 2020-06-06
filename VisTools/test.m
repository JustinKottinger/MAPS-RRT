clear all; close all; clc;

xmax = 14;
ymax = 10;

x = linspace(0, 14, 15);
y = linspace(0, 10, 11);

figure(1);
for i = 0:1:xmax
    hold on;
    xs = i .* ones(length(y), 1);
    plot(xs, y, "black");
end
for j = 0:1:ymax
    hold on;
    ys = j .* ones(length(x), 1);
    plot(x, ys, "black");
end

rectangle("Position", [4, 0, 4, 2], 'FaceColor', "black")
rectangle("Position", [4, 4, 4, 6], 'FaceColor', "black")
rectangle("Position", [10, 2, 2, 2], 'FaceColor', "black")
rectangle("Position", [10, 6, 2, 2], 'FaceColor', "black")

set(gca,'XTick',[], 'YTick', [])
axis([0 xmax 0 ymax])