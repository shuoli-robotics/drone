function [] = sensor_model()
%SENSOR_MODEL Summary of this function goes here
%   Detailed explanation goes here
global drone_states pointer sensor_states

R_pqr = diag([0.3^2 0.3^2 0.3^2]);
mu_pqr = [0 0 0];

sensor_states(:,pointer) = drone_states(:,pointer); 
sensor_states(10:12,pointer) = drone_states(10:12,pointer)+[mvnrnd(mu_pqr,R_pqr,1)]' ;  % p q r

end

