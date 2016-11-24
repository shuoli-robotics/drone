function [done] = control_guidance_loop(ini_states,target_states)
%MAIN ????????????
%   ????????
global simulation_time step pointer;
global drone_states actuator_states;
global i time desired_omega desired_angular_velocity
global desired_angle desired_velocity_body desired_position 
global controller guidance_method

controller = 'PID';  % INDI or PID

global_parameters(ini_states);



for i = 0:step:simulation_time
    
    [ desired_position(:,pointer),desired_angle(3,pointer) ] = guidance_drone(target_states);
    %desired_position(:,pointer)=[2 2 2]';
    
 %% sensor model
     sensor_model();
 %% position controller
    [ desired_velocity_body(:,pointer) ] = controller_position(desired_position(:,pointer));

   %% velocity controller
   %desired_velocity_body(:,pointer) = [1 1 -1]';
    if i<5 || i > 11
        desired_velocity_body(:,pointer) = [0 0 0]';
    end
    [ desired_angle(1:2,pointer),F ] = controller_velocity_body( desired_velocity_body(:,pointer) );

 %% angle controller

  %desired_angle(:,pointer) = [5/180*pi 0 0]';
    if i<8 && i>=5 
        desired_angle(:,pointer) = [5/180*pi 0 0]';
    elseif i>=8 && i<11
        desired_angle(:,pointer) = [-5/180*pi 0 0]';
    end
    desired_angular_velocity(:,pointer) = controller_angle( desired_angle(:,pointer));
    %temp = desired_angular_velocity(:,pointer)
 %% angular velocity controller 
 %desired_angular_velocity(:,pointer) = [0 0 0.5];

    M = controller_angular_velocity( desired_angular_velocity(:,pointer));

    desired_omega(:,pointer) = convert_force_2_omega(F,M);
    if min(desired_omega(:,pointer)) < 0
        done = 0;
        return;
    end
    %% actuator dynamics
    actuator_states(:,pointer+1) = runge_kutta_actuator(actuator_states(:,pointer),desired_omega(:,pointer));
    
    %% drone dynamics
    drone_states(:,pointer+1) = runge_kutta_drone(drone_states(:,pointer),actuator_states(:,pointer+1));
    %%
    pointer = pointer+1;
    time(pointer) = i;
    
    if stop_simulation(guidance_method,target_states)
        break;
    end
    
end

done = 1;
plot_actuator();
end



function [flag_stop] = stop_simulation(guidance_mode,target_states)
global stop_condition_replay_rrt drone_states pointer
flag_stop = 0;
switch guidance_mode
    
    case 'CL_RRT'
        if norm(drone_states(1:3,pointer)-target_states) < 0.01 ...
                && norm(drone_states(4:6,pointer)) < 0.1
            flag_stop = 1;
        end
    case 'REPLAT_RRT'
        if norm(drone_states(1:3,pointer)-stop_condition_replay_rrt) < 0.01
            flag_stop = 1;
        end
end
                

end