function [ new_states ] = runge_kutta( current_states)
%RUNGE_KUTTA ????????????
%   ????????
global  step omega
k1 = drone_dynamics( current_states,omega);
k2 = drone_dynamics( current_states+step/2*k1,omega);
k3 = drone_dynamics( current_states+step/2*k2,omega);
k4 = drone_dynamics( current_states+step*k3,omega);

new_states =  current_states + step/6*(k1+2*k2+2*k3+k4);

end

