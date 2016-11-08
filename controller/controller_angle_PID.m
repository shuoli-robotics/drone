function [ desired_angular_velocity ] = controller_angle_PID( desired_angle )
%CONTROLLER_ANGLE Summary of this function goes here
%   Detailed explanation goes here
global drone_states pointer

k_p_angular_velocity_phi = 2.5;
k_p_angular_velocity_theta = 2.1;
k_p_angular_velocity_psi = 3;

phi = drone_states(7);
theta = drone_states(8);

R_d_angle = [1 tan(theta)*sin(phi) tan(theta)*cos(phi);...
    0 cos(phi) -sin(phi);...
    0 sin(phi)/cos(theta) cos(phi)/cos(theta)];

error_angle = desired_angle - ...
    [drone_states(7,pointer) drone_states(8,pointer) drone_states(9,pointer)]';

d_angle_needed = [k_p_angular_velocity_phi k_p_angular_velocity_theta k_p_angular_velocity_psi]'...
    .*error_angle;

desired_angular_velocity = inv(R_d_angle)*d_angle_needed; 

end

