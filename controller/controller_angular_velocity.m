function [ M ] = controller_angular_velocity( desired_angular_velocity )
%CONTROLLER_ANGULAR_VELOCITY_PID Summary of this function goes here
%   Detailed explanation goes here
global pointer controller M_last_step I current_angular_velocity_accel sensor_states

switch controller
    case 'PID'
        
        k_p_angular_velocity_p = 0.016;
        k_p_angular_velocity_q = 0.016;
        k_p_angular_velocity_r = 0.028;
        current_angular_velocity = sensor_states(10:12,pointer);
        error_angular_veloctity = desired_angular_velocity-current_angular_velocity;
        M = [k_p_angular_velocity_p k_p_angular_velocity_q k_p_angular_velocity_r]'...
            .* error_angular_veloctity;
    case 'INDI'
        k_p_angular_velocity_p = 8;
        k_p_angular_velocity_q = 8;
        k_p_angular_velocity_r = 4;

        current_angular_velocity = sensor_states(10:12,pointer);

        error_angular_velocity = desired_angular_velocity - current_angular_velocity;

        delta_M = I*((error_angular_velocity).*[k_p_angular_velocity_p...
            k_p_angular_velocity_q k_p_angular_velocity_r]'-current_angular_velocity_accel);

        M = M_last_step + delta_M;
        M_last_step = M;
end

end

