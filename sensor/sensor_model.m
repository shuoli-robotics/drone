function [] = sensor_model()
%SENSOR_MODEL Summary of this function goes here
%   Detailed explanation goes here
global drone_states pointer sensor_states actuator_states sensor_states_raw

R_pqr = diag([0.8^2 0.8^2 0.8^2]);
mu_pqr = [0 0 0];

sensor_states_raw(:,pointer) = drone_states(:,pointer); 
sensor_states_raw(10:12,pointer) = drone_states(10:12,pointer)+[mvnrnd(mu_pqr,R_pqr,1)]' ;  % p q r
sensor_states(:,pointer) = sensor_states_raw(:,pointer);
sensor_states(10:12,pointer) = EKF_pqr(sensor_states_raw(10:12,pointer),actuator_states(:,pointer));

end

