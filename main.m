function [] = main()
%MAIN ????????????
%   ????????
global simulation_time step pointer m g;
global drone_states omega ;
global_parameters();

for i = 1:simulation_time/step:simulation_time
    
    omega = [m*g m*g m*g m*g]'/4+1; 
    current_states = drone_states(1:12,pointer);
    drone_states(:,pointer+1) = runge_kutta(current_states);
    pointer = pointer+1;
end
end

