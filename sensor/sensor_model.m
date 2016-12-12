function [] = sensor_model()
%SENSOR_MODEL Summary of this function goes here
%   Detailed explanation goes here
global drone_states pointer sensor_states actuator_states sensor_states_raw flag_sensor_uncertainty
if flag_sensor_uncertainty  == 1
    R_pqr = [diag(0.02*[1 1 1])].^2;
mu_pqr = [0 0 0];
else
    R_pqr = zeros(3,3);
    mu_pqr = [0 0 0];
end


sensor_states_raw(:,pointer) = drone_states(:,pointer); 
sensor_states_raw(10:12,pointer) = drone_states(10:12,pointer)+[mvnrnd(mu_pqr,R_pqr,1)]' ;  % p q r
sensor_states(:,pointer) = sensor_states_raw(:,pointer);
sensor_states(10:12,pointer) = EKF_pqr(sensor_states_raw(10:12,pointer),actuator_states(:,pointer));

end

