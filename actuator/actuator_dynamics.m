function [ d_omega ] = actuator_dynamics( desired_omega,current_omega )
%ACTUATOR_DYNAMICS ????????????
%   ????????

one_order_constant_actuator = 20;
error_actuator_states = desired_omega - current_omega;
d_omega = one_order_constant_actuator * error_actuator_states;
end

