function [] = main()
%MAIN ????????????
%   ????????
global simulation_time step pointer;
global drone_states actuator_states;
global i time desired_omega desired_angular_velocity
global desired_angle
global_parameters();

for i = 0:step:simulation_time

    desired_velocity_body = [1 1 -1]';
    desired_psi = 0;
   %% velocity controller
    [ desired_angle(1:2,pointer),F ] = controller_velocity_body( desired_velocity_body );
    desired_angle(3,pointer) = desired_psi;
 %% angle controller
    desired_angular_velocity(:,pointer) = controller_angle_PID( desired_angle(:,pointer));
 %% angular velocity controller   
    M = controller_angular_velocity_PID( desired_angular_velocity(:,pointer));
    desired_omega(:,pointer) = convert_force_2_omega(F,M);
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

