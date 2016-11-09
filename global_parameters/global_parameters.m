function [] = global_parameters()
%GLOBAL_PARAMETERS ????????????
%   ????????
global m I k_F k_M L g
global simulation_time step pointer;
global drone_states actuator_states time desired_omega desired_angular_velocity desired_angle desired_velocity_body
global position_error desired_position angle_error velocity_body_error M_last_step current_angular_velocity_accel
%% model parameters
k_F = 6.11*10^-8;
k_M = 1.5*10^-9;
L = 0.175;
I_xx = 2.32*10^-3;
I_yy = 2.32*10^-3;
I_zz = 4.00*10^-3;
m = 0.5;
g = 9.8;
I = [I_xx 0 0; 0 I_yy 0; 0 0 I_zz];

%% simulation parameters

simulation_time = 10;
step = 1/512;
pointer = 1;

%% states
drone_states = zeros(12,simulation_time/step);
drone_states(:,1) = [0 0 0 0 0 0 0 0 0 0 0 0]';
desired_omega = zeros(4,simulation_time/step);
actuator_states = zeros(4,simulation_time/step);
actuator_states(:,1) = 4474;
desired_angular_velocity = zeros(3,simulation_time/step);
desired_angle = zeros(3,simulation_time/step);
desired_velocity_body = zeros(3,simulation_time/step);
time = zeros(1,simulation_time/step);
position_error  = zeros(3,simulation_time/step);
angle_error =  zeros(3,simulation_time/step);
velocity_body_error =  zeros(3,simulation_time/step);
desired_position = zeros(3,simulation_time/step);

%% INDI
M_last_step = 0;
current_angular_velocity_accel = 0;
end

