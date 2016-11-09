function [ desired_angle,F ] = controller_velocity_body_PID( desired_velocity_body )
%CONTROLLER_VELOCITY_BODY ????????????
%   ????????
global drone_states pointer g m velocity_body_error step i

k_p_v_x_body = 5;
k_p_v_y_body = 5;
k_p_v_z_body = 3;

k_d_v_x_body = 0;
k_d_v_y_body = 0;
k_d_v_z_body = 0;

u = drone_states(4,pointer);
v = drone_states(5,pointer);
w = drone_states(6,pointer);
p = drone_states(10,pointer);
q = drone_states(11,pointer);
r = drone_states(12,pointer);



velocity_body_error(:,pointer) = [(desired_velocity_body(1)-u) ...
    (desired_velocity_body(2)-v) (desired_velocity_body(3)- w)]';

if pointer == 1
    d_velocity_error = zeros(2,1);
else
    d_velocity_error = (velocity_body_error(1:2,pointer)-velocity_body_error(1:2,pointer-1))/step;
end

desired_theta = (k_p_v_x_body*(velocity_body_error(1,pointer) + ...
    k_d_v_x_body*d_velocity_error(1)))/(-g);

desired_phi = (k_p_v_y_body*(velocity_body_error(2,pointer)+...
    k_d_v_y_body*d_velocity_error(2)))/(g);
F = (-g + k_p_v_z_body*velocity_body_error(3,pointer))*m; 
desired_angle = [desired_phi desired_theta]';
end

