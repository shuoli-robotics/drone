function [ desired_body_velocity ] = controller_position_PID( desired_position )
%CONTROLLER_POSITION ????????????
%   ????????
global drone_states  position_error pointer step

k_p_x = 2;
k_p_y = 2;
k_p_z = 1.5;

k_d_x = 0;
k_d_y = 0;
k_d_z = 0;

phi = drone_states(7,pointer);
theta = drone_states(8,pointer);
psi= drone_states(9,pointer);

R_E_B = [cos(theta)*cos(psi) cos(theta)*sin(phi) -sin(theta);...
     sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
     sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
     cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
     cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
 
current_position = [drone_states(1,pointer) drone_states(2,pointer) drone_states(3,pointer)]';
position_error(:,pointer) = desired_position - current_position;
if (pointer == 1)
    d_position_error = 0;
else
    d_position_error = (position_error(:,pointer)-position_error(:,pointer-1))/step;
end

desired_velocity_earth =[k_p_x k_p_y k_p_z]' .* (position_error(:,pointer)) ...
    +[k_d_x k_d_y k_d_z]' .* (d_position_error);
desired_body_velocity = R_E_B*desired_velocity_earth;
end

