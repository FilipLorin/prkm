points = readtable("pointcloud.csv");
conf = readtable('conflicts.csv');
err = readtable('errors.csv');

figure
scatter3(points.x, points.y, points.z, ".g")
xlabel("X")
ylabel("Y")
zlabel("Z")
axis equal
hold on


scatter3(err.x, err.y, err.z, '.b')

scatter3(conf.xf, conf.yf, conf.zf, '.y')
scatter3(conf.xi, conf.yi, conf.zi, '.r')
quiver3(conf.xi, conf.yi, conf.zi, conf.xf-conf.xi, conf.yf-conf.yi, conf.zf-conf.zi, 0, 'r')
