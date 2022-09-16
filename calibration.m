%% Data Collection

% Run for each distance d
clear;
arduino = arduino(); % define our arduino
sensorReadings = zeros(50);

for i=1:50

    % Save the voltage reading from A0
    sensorReadings(i) = readVoltage(arduino, "A0");
    pause(.5); % wait half a second before gathering next reading
end

% find mean value
total = 0.0;

for i=1:50
    total = total + sensorReadings(i);
end

mean = total/50

%% Plotting Data
clear;
distances = [0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];
voltages = [2.4577 1.8949 1.4791 1.2036 1.0198 0.8938 0.7889 0.7114 0.6463];
clf;
figure(1); hold on;
plot(distances, voltages, "r.", "LineWidth", 2, "MarkerSize", 20);
xlabel("Distance [m]");
ylabel("Voltage Reading [V]")


x = 0.2:0.01:1;
fitted_curve = 4*exp(-3.5*x) + .5;
plot(x, fitted_curve, "b", "LineWidth", 2);