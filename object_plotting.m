%%
clear all; clc; % clear old variables

%delete(instrfind({'Port'},{'COM5'}));
port = "COM7"; % change if different port

% find the arduino and get the baudrate
arduinoObj = serialport(port, 250000);

fopen(arduinoObj);
points_sent = 50*50;

% arrays to hold points
x_distances = zeros(0, points_sent);
y_distances = zeros(0, points_sent);
z_distances = zeros(0, points_sent);

count = 1;
% 2-D visualization
while count < points_sent
   
    distances = fscanf(arduinoObj, "%s");
    temp_dist_array = str2num(distances);
    x_distances(count) = temp_dist_array(1);
    y_distances(count) = temp_dist_array(2);
    z_distances(count) = temp_dist_array(3);
    count = count + 1;

end

figure(1); hold on; grid on; clf;

scatter(y_distances, z_distances, 30, x_distances, "filled");
colorbar
title("3-D Scan of a Number 2");
xlabel("x distance from scanner [m]");
ylabel("y distance from scanner [m]");
