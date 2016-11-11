function [] = sensor_model()
%SENSOR_MODEL Summary of this function goes here
%   Detailed explanation goes here
global drone_states pointer sensor_states

sensor_states(:,pointer) = drone_states(:,pointer); 


end

