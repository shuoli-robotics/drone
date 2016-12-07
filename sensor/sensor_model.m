function [] = sensor_model()
%SENSOR_MODEL Summary of this function goes here
%   Detailed explanation goes here
global drone_states pointer sensor_states

sensor_states(:,pointer) = drone_states(:,pointer); 
% sensor_states(7,pointer) = sensor_states(7,pointer) + 1/180*pi;  % phi
%sensor_states(8,pointer) = sensor_states(8,pointer) + 1/180*pi; % theta

end

