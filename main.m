function [ ] = main(  )
%MAIN ????????????
%   ????????
global guidance_method
clf;
guidance_method = 'STEP';
ini_states = [0 0 0 0 0 0 0 0 0 0 0 0]';
target_states = [0 0 0]';

control_guidance_loop(ini_states,target_states)

end

