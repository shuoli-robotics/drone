id_az = 7;
id_thrust = 21;
data = flight_data_body_coor;
n = size(data,1);

figure();
plot(smooth(data(:, id_az), 100));
hold on;
plot(smooth(data(:, id_thrust), 10) / -10);


data(:, id_thrust) = data(:, id_thrust) / 100.0;
%params = [1; 0.5; -9.8];
data(:, id_az) = smooth(data(:, id_az), 50);
%params(1) * data(:, id_thrust).^2 + params(2) * data(:, id_thrust) + params(3) + 0.2*randn(n,1);
A = [data(:, id_thrust).^2, data(:, id_thrust), ones(n,1)];
targets = data(:, id_az);
params = A \ targets;

estimates = params(1) * data(:, id_thrust).^2 + params(2) * data(:, id_thrust) + params(3);
figure();
plot(smooth(data(:, id_az), 500));
hold on;
plot(smooth(estimates, 500));
legend({'Actual acceleration', 'Estimated acceleration'});