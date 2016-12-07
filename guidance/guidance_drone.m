function [ desired_position,desired_psi ] = guidance_drone(target_states)
%GUIDANCE_DRONE Summary of this function goes here
%   Detailed explanation goes here
global time  pointer reference_x reference_y reference_z   guidance_method
step_distance = 1;

switch guidance_method
    
    case 'STEP'
        if time(pointer)<5
            desired_position = step_distance*[1 0 -1*time(pointer)]';
        elseif time(pointer) < 10
            desired_position = step_distance*[0 1 -1*time(pointer)]';
        elseif time(pointer) < 15
            desired_position = step_distance*[-1 0 -1*time(pointer)]';
        elseif time(pointer)<20
            desired_position = step_distance*[0 -1 -1*time(pointer)]';
        else
            desired_position = step_distance*[1 0 -1*time(pointer)]';
        end
% desired_position = [1 1 1]';
        desired_psi = 0;
        
    case 'SET_POINT'
        current_reference = [reference_x(:,pointer);reference_y(:,pointer);reference_z(:,pointer)];
        [reference_x(:,pointer+1),reference_y(:,pointer+1),reference_z(:,pointer+1)]=runge_kutta_trans_fun(current_reference);
        desired_position = [reference_x(1,pointer+1);reference_y(1,pointer+1);reference_z(1,pointer+1)];
         desired_psi = 0;
         
    case 'CL_RRT'
        desired_position = target_states;
        desired_psi = 0;
        
    case 'REPLAT_RRT'
        desired_position = target_states;
        desired_psi = 0;
        
    case 'VELOCITY'
        desired_position = target_states;   % here target_states is desired_velocity
        desired_psi = 0;
        
    case 'ATTITUDE'
        desired_position = target_states;
        desired_psi = 0;
end

end



%% subfunctions for setpoint mode (generate smooth reference trajectory)
function [reference_x,reference_y,reference_z] = runge_kutta_trans_fun(current_states)
% current_states = [reference_x(:,pointer);reference_y(:,pointer);reference_z(:,pointer)]
global  step setpoint_x setpoint_y setpoint_z 
u_x = setpoint_x-current_states(1);
k1 = transter_function_model_x(current_states(1:2),u_x);
k2 = transter_function_model_x(current_states(1:2)+step/2*k1,u_x);
k3 = transter_function_model_x(current_states(1:2)+step/2*k2,u_x);
k4 = transter_function_model_x(current_states(1:2)+step*k3,u_x);
reference_x =  current_states(1:2) + step/6*(k1+2*k2+2*k3+k4);

u_y = setpoint_y-current_states(3);
k1 = transter_function_model_y(current_states(3:4),u_y);
k2 = transter_function_model_y(current_states(3:4)+step/2*k1,u_y);
k3 = transter_function_model_y(current_states(3:4)+step/2*k2,u_y);
k4 = transter_function_model_y(current_states(3:4)+step*k3,u_y);
reference_y =  current_states(3:4)+step/6*(k1+2*k2+2*k3+k4);

u_z = setpoint_z-current_states(5);
k1 = transter_function_model_z(current_states(5:6),u_z);
k2 = transter_function_model_z(current_states(5:6)+step/2*k1,u_z);
k3 = transter_function_model_z(current_states(5:6)+step/2*k2,u_z);
k4 = transter_function_model_z(current_states(5:6)+step*k3,u_z);
reference_z =  current_states(5:6) + step/6*(k1+2*k2+2*k3+k4);
end

function [dx] = transter_function_model_x(current_x,u)

dx = [0 1;-1.5^2 -3]*current_x + [0 1.5^2]'*u;
end

function [dy] = transter_function_model_y(current_y,u)

dy = [0 1;-1.5^2 -3]*current_y + [0 1.5^2]'*u;
end

function [dz] = transter_function_model_z(current_y,u)

dz = [0 1;-1.5^2 -3]*current_y + [0 1.5^2]'*u;
end
