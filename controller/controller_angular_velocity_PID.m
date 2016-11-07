function [ F,M ] = controller_angular_velocity_PID( desired_angular_velocity,desired_velocity_body_z )
%CONTROLLER_ANGULAR_VELOCITY_PID Summary of this function goes here
%   Detailed explanation goes here
global drone_states pointer g m

k_p_angular_velocity_p = 0.016;
k_p_angular_velocity_q = 0.016;
k_p_angular_velocity_r = 0.028;
k_p_v_z_body = 5;
current_angular_velocity = drone_states(10:12,pointer);
error_angular_veloctity = desired_angular_velocity-current_angular_velocity;
M = [k_p_angular_velocity_p k_p_angular_velocity_q k_p_angular_velocity_r]'...
    .* error_angular_veloctity;
F = (-g + k_p_v_z_body*(desired_velocity_body_z - drone_states(6,pointer)))*m; 
end

