function [] = control_guidance_loop(guidance_metod,ini_states,target_states)
%MAIN ????????????
%   ????????
global simulation_time step pointer;
global drone_states actuator_states;
global i time desired_omega desired_angular_velocity
global desired_angle desired_velocity_body desired_position 
global controller 

controller = 'PID';  % INDI or PID

global_parameters(ini_states);



for i = 0:step:simulation_time
    
    [ desired_position(:,pointer),desired_angle(3,pointer) ] = guidance_drone(guidance_metod,target_states);
    %desired_position(:,pointer)=[2 2 2]';
    
 %% sensor model
     sensor_model();
 %% position controller
    [ desired_velocity_body(:,pointer) ] = controller_position(desired_position(:,pointer));

   %% velocity controller
   %desired_velocity_body(:,pointer) = [1 1 -1]';
    [ desired_angle(1:2,pointer),F ] = controller_velocity_body( desired_velocity_body(:,pointer) );

 %% angle controller

  %desired_angle(:,pointer) = [5/180*pi 0 0]';

    desired_angular_velocity(:,pointer) = controller_angle( desired_angle(:,pointer));
    %temp = desired_angular_velocity(:,pointer)
 %% angular velocity controller 
 %desired_angular_velocity(:,pointer) = [0 0 0.5];

    M = controller_angular_velocity( desired_angular_velocity(:,pointer));

    desired_omega(:,pointer) = convert_force_2_omega(F,M);
    %% actuator dynamics
    actuator_states(:,pointer+1) = runge_kutta_actuator(actuator_states(:,pointer),desired_omega(:,pointer));
    
    %% drone dynamics
    drone_states(:,pointer+1) = runge_kutta_drone(drone_states(:,pointer),actuator_states(:,pointer+1));
    %%
    pointer = pointer+1;
    time(pointer) = i;
    
    if strcmp(guidance_method, 'CL_RRT')
        if norm(drone_states(1:3,pointer)-target_states(1)) < 0.01
            break;
        end
    end
    
end


plot_actuator();
end

