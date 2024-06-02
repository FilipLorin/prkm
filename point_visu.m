clear; clc; close all;
data = readtable("pointcloud.csv");

figure
scatter3(data.x, data.y, data.z)
xlabel("X")
ylabel("Y")
zlabel("Z")
axis equal
