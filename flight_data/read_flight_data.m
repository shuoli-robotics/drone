function [ output_args ] = read_flight_data( )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
flight_data = csvread('pitch_3.csv');

matrix_x = [1 0 0;0 cos(pi) -sin(pi);0 sin(pi) cos(pi)];

alpha = -flight_data(3,14);
matrix_z = [cos(alpha) -sin(alpha) 0;...
    sin(alpha) cos(alpha) 0 ; 0 0 1];

for i = 1:size(flight_data,1)
    flight_data(i,15:17) = (matrix_z*flight_data(i,15:17)')';
    flight_data(i,18:20) = (matrix_z*flight_data(i,18:20)')';
    flight_data(i,15:17) = (matrix_x*flight_data(i,15:17)')';
    flight_data(i,18:20) = (matrix_x*flight_data(i,18:20)')';
     
end
time = flight_data(:,2);

p = flight_data(:,3);
q = flight_data(:,4);
r = flight_data(:,5);

accel_x = flight_data(:,6);
accel_y = flight_data(:,7);
accel_z = flight_data(:,8);

phi =  flight_data(:,12);
theta =  flight_data(:,13);
psi =  flight_data(:,14);

x = flight_data(:,15);
y = flight_data(:,16);
z = flight_data(:,17);

v_x = flight_data(:,18);
v_y = flight_data(:,19);
v_z = flight_data(:,20);

thrust = flight_data(:,21);

phi_command = flight_data(:,22);
theta_command = flight_data(:,23);
psi_command = flight_data(:,24);

figure(1)
hold on;
plot3(x(2),y(2),z(2),'o')
plot3(x,y,z);
xlabel('x[m]');
ylabel('y[m]');
zlabel('z[m]');
axis auto;
grid on;

figure(2)
hold on;
subplot(1,3,1)
plot(time,v_x);
xlabel('time[s]');
ylabel('v_x[m/s]');
subplot(1,3,2)
plot(time,v_y);
xlabel('time[s]');
ylabel('v_y[m/s]');
subplot(1,3,3)
plot(time,v_z);
xlabel('time[s]');
ylabel('v_z[m/s]');
end

