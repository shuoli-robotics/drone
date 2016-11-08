function [ desired_body_velocity ] = controller_position_PID( desired_position )
%CONTROLLER_POSITION ????????????
%   ????????
global drone_states pointer

k_p_angular_x = 0.8;
k_p_angular_y = 0.8;
k_p_angular_z = 0.8;

phi = drone_states(7);
theta = drone_states(8);
psi= drone_states(9);

R_E_B = [cos(theta)*cos(psi) cos(theta)*sin(phi) -sin(theta);...
     sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
     sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
     cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
     cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
 
current_position = [drone_states(1,pointer) drone_states(2,pointer) drone_states(3,pointer)]';
desired_velocity_earth =[k_p_angular_x k_p_angular_y k_p_angular_z]' .* (desired_position - current_position);
desired_body_velocity = R_E_B*desired_velocity_earth;
end

