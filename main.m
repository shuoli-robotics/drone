function [] = main()
%MAIN ????????????
%   ????????
global simulation_time step pointer m g;
global drone_states omega actuator_states;
global i time;
global_parameters();

for i = 0:step:simulation_time
    
    omega = [m*g m*g m*g m*g]'/4+1; 
    current_states = drone_states(1:12,pointer);
    desired_omega = [5000 5000 5000 5000]';
    actuator_states(:,pointer+1) = runge_kutta_actuator(actuator_states(:,pointer),desired_omega);
    drone_states(:,pointer+1) = runge_kutta(current_states);
    pointer = pointer+1;
    time(pointer) = i;
end

plot_actuator();
end

