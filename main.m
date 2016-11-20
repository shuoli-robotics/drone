function [ ] = main(  )
%MAIN ????????????
%   ????????
global guidance_method
clf;
guidance_method = 'CL_RRT';
ini_states = [1 1 -1 1 0 0 0 0 0 0 0 0]';
target_states = [5 5 -1]';

control_guidance_loop(ini_states,target_states)

end

