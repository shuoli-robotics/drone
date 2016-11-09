function [ desired_angular_velocity ] = controller_angle_PID( desired_angle )
%CONTROLLER_ANGLE Summary of this function goes here
%   Detailed explanation goes here
global drone_states pointer angle_error step

k_p_angular_velocity_phi = 10;
k_p_angular_velocity_theta = 10;
k_p_angular_velocity_psi = 3;

k_d_angular_velocity_phi = 1;
k_d_angular_velocity_theta = 1;
k_d_angular_velocity_psi = 0.2;

% k_d_angular_velocity_phi = 0;
% k_d_angular_velocity_theta = 0;
% k_d_angular_velocity_psi = 0;

phi = drone_states(7,pointer);
theta = drone_states(8,pointer);

R_d_angle = [1 tan(theta)*sin(phi) tan(theta)*cos(phi);...
    0 cos(phi) -sin(phi);...
    0 sin(phi)/cos(theta) cos(phi)/cos(theta)];

angle_error(:,pointer) = desired_angle - ...
    [drone_states(7,pointer) drone_states(8,pointer) drone_states(9,pointer)]';

if pointer == 1
    d_angle_error = 0;
else
    d_angle_error = (angle_error(:,pointer) - angle_error(:,pointer-1))/step;
end

d_angle_needed = [k_p_angular_velocity_phi k_p_angular_velocity_theta k_p_angular_velocity_psi]'...
    .*angle_error(:,pointer) + ...
    [k_d_angular_velocity_phi k_d_angular_velocity_theta k_d_angular_velocity_psi]' ...
    .* d_angle_error;

%desired_angular_velocity = inv(R_d_angle)*d_angle_needed; 
desired_angular_velocity =d_angle_needed;
%temp = inv(R_d_angle); 
end

