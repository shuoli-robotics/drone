function [ desired_angle,F ] = controller_velocity_body_PID( desired_velocity_body )
%CONTROLLER_VELOCITY_BODY ????????????
%   ????????
global drone_states pointer g m

k_p_v_x_body = 1.5;
k_p_v_y_body = 1.5;
k_p_v_z_body = 5;

u = drone_states(4,pointer);
v = drone_states(5,pointer);
w = drone_states(6,pointer);
p = drone_states(10,pointer);
q = drone_states(11,pointer);
r = drone_states(12,pointer);

desired_theta = (k_p_v_x_body*(desired_velocity_body(1)-u)+(q*w-r*v))/(-g);
desired_phi = (k_p_v_y_body*(desired_velocity_body(2)-v)+(r*u-p*w))/(g);
F = (-g + k_p_v_z_body*(desired_velocity_body(3)- w))*m; 
desired_angle = [desired_phi desired_theta]';
end

