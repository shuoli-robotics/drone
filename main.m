function [] = main()
%MAIN ????????????
%   ????????
global simulation_time step pointer m g;
global drone_states;
global_parameters();

temp = 1;

for i = 1:simulation_time/step:simulation_time
    
    omega = [m*g m*g m*g m*g]'/4; 
    current_states = drone_states(1:12,end);
    [d_states] = drone_dynamics(current_states,omega);
end
end

