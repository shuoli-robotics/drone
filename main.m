function [ ] = main(  )
%MAIN ????????????
%   ????????
global guidance_method sensor_states sensor_states_raw drone_states
clf;
guidance_method = 'STEP';
ini_states = [0 0 -1.5 0 0 0 0 0 0 0 0 0]';
target = [2.5 5 -1.5];
%[optimal_trajectory] = cl_rrt(target,ini_states);
 [done] = control_guidance_loop(ini_states,target)
plot_sensor();
end

