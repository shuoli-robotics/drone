function [ desired_position,desired_psi ] = guidance_drone()
%GUIDANCE_DRONE Summary of this function goes here
%   Detailed explanation goes here
global time guidance pointer

switch guidance
    case 'STEP'
        if time(pointer)<5
            desired_position = [0.5 0 -0.5*time(pointer)]';
        elseif time(pointer) < 10
            desired_position = [0 0.5 -0.5*time(pointer)]';
        elseif time(pointer) < 15
            desired_position = [-0.5 0 -0.5*time(pointer)]';
        elseif time(pointer)<20
            desired_position = [0 -0.5 -0.5*time(pointer)]';
        else
            desired_position = [0.5 0 -0.5*time(pointer)]';
        end
        desired_psi = 0;
end

end

