function [next_omega] = runge_kutta_actuator( current_omega,desired_omega )
%RUNGE_KUTTA_ACTUATOR ????????????
%   ????????
global step

k1 = actuator_dynamics(desired_omega,current_omega);
k2 = actuator_dynamics(desired_omega,current_omega+step/2*k1);
k3 = actuator_dynamics(desired_omega,current_omega+step/2*k2);
k4 = actuator_dynamics(desired_omega,current_omega+step*k3);
next_omega = current_omega+step/6*(k1+2*k2+2*k3+k4);
for j = 1:4
    if next_omega(j) < 0
        next_omega(j) = 0;
    elseif next_omega(j) > 9000
        next_omega(j) = 0;
    end
end
end

