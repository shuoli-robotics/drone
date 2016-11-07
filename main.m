function [] = main()
%MAIN ????????????
%   ????????
global simulation_time step pointer m g;
global drone_states actuator_states;
global i time desired_omega
global_parameters();

for i = 0:step:simulation_time

    thrust = m*g+0.1;
    moment = [0 0 0]';
    desired_omega(:,pointer) = convert_force_2_omega(thrust,moment);
    %% actuator dynamics
    actuator_states(:,pointer+1) = runge_kutta_actuator(actuator_states(:,pointer),desired_omega(:,pointer));
    
    %% drone dynamics
    drone_states(:,pointer+1) = runge_kutta_drone(drone_states(:,pointer),actuator_states(:,pointer+1));
    
   
    
    %%
    pointer = pointer+1;
    time(pointer) = i;
end

plot_actuator();
end

