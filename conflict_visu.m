data = readtable('conflicts.csv');

figure
quiver3(data.xf, data.yf, data.zf, data.xi, data.yi, data.zi)
xlabel("X")
ylabel("Y")
zlabel("Z")
axis equal