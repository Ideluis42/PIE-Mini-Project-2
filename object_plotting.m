clear all; clc; % clear old variables

%delete(instrfind({'Port'},{'COM5'}));
port = "COM5"; % change if different port

% find the arduino and get the baudrate
arduinoObj = serialport(port, 9600);

fopen(arduinoObj);
x_distances = zeros(1, 10);
y_distances = zeros(1, 10);
theta = 0; % to be replaced with servo angle

count = 1;
while count < 11
    distance = fscanf(arduinoObj, "%f");
    x_distances(count) = distance*cosd(theta);
    y_distances(count) = distance*sind(theta);
    count = count + 1;
end