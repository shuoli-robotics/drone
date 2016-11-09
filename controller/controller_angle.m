function [ desired_angular_velocity ] = controller_angle( desired_angle )
%CONTROLLER_ANGLE Summary of this function goes here
%   Detailed explanation goes here
global drone_states pointer angle_error step controller

switch controller
    case 'PID'
        k_p_angular_velocity_phi = 10;
        k_p_angular_velocity_theta = 10;
        k_p_angular_velocity_psi = 3;

        k_d_angular_velocity_phi = 1;
        k_d_angular_velocity_theta = 1;
        k_d_angular_velocity_psi = 0.2;

        % k_d_angular_velocity_phi = 0;
        % k_d_angular_velocity_theta = 0;
        % k_d_angular_velocity_psi = 0;

        
    case 'INDI'
        k_p_angular_velocity_phi = 4;
        k_p_angular_velocity_theta = 4;
        k_p_angular_velocity_psi = 1.5;
        
        k_d_angular_velocity_phi = 0.2;
        k_d_angular_velocity_theta = 0.3;
        k_d_angular_velocity_psi = 0;
end


phi = drone_states(7,pointer);
theta = drone_states(8,pointer);

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

desired_angular_velocity = [1 0 -sin(theta);0 cos(phi) cos(theta)*sin(phi);0 -sin(phi) cos(theta)*cos(phi)]*d_angle_needed;


end

