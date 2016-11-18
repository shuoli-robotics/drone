clear
clc

ini_states = [0 0 0 0 0 0 0 0 0 0 0 0];
target_states = [1 1 -3];

[done] = control_guidance_loop(ini_states,target_states);