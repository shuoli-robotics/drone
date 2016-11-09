function [ M ] = controller_angular_velocity_INDI( desired_angular_velocity )
%CONTROLLER_ANGULAR_VELOCITY_INDI Summary of this function goes here
%   Detailed explanation goes here
global drone_states pointer M_last_step current_angular_velocity_accel I

k_p_angular_velocity_p = 1;
k_p_angular_velocity_q = 1;
k_p_angular_velocity_r = 1;

current_angular_velocity = drone_states(10:12,pointer);

error_angular_velocity = desired_angular_velocity - current_angular_velocity;

delta_M = I*((error_angular_velocity).*[k_p_angular_velocity_p...
    k_p_angular_velocity_q k_p_angular_velocity_r]'-current_angular_velocity_accel);

M = M_last_step + delta_M;
M_last_step = M;

end

