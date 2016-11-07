function [] = main()
%MAIN ????????????
%   ????????
global simulation_time step pointer m g;
global drone_states actuator_states;
global i time desired_omega desired_angular_velocity
global_parameters();

for i = 0:step:simulation_time

    desired_angle = [0.3 0.3 0]';
    desired_velocity_body_z = 0;
    
    
 %% angle controller
    [ desired_angular_velocity(:,pointer)] = controller_angle_PID( desired_angle);
 %% angular velocity controller   
    [thrust,moment] = controller_angular_velocity_PID( desired_angular_velocity(:,pointer),desired_velocity_body_z );
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

