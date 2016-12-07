function [ ] = main(  )
%MAIN ????????????
%   ????????
global guidance_method
clf;
guidance_method = 'CL_RRT';
ini_states = [0 0 -1.5 0 0 0 0 0 0 0 0 0]';
target = [2.5 5 -1.5];
[optimal_trajectory] = cl_rrt(target,ini_states);
 [done] = control_guidance_loop(ini_states,target)

end

