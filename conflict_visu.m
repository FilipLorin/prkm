points = readtable("pointcloud.csv");
conf = readtable('conflicts.csv');

figure
scatter3(points.x, points.y, points.z, ".")
xlabel("X")
ylabel("Y")
zlabel("Z")
axis equal
hold on

scatter3(conf.xf, conf.yf, conf.zf, '.y')
scatter3(conf.xi, conf.yi, conf.zi, '.r')
quiver3(conf.xi, conf.yi, conf.zi, conf.xf-conf.xi, conf.yf-conf.yi, conf.zf-conf.zi, 0, 'r')
