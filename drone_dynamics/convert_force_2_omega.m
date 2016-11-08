function [ omega_needed ] = convert_force_2_omega(F,M)
%CONVERT_FORCE_2_OMEGA ????????????
%   ????????
global k_F L k_M i pointer
matrix_omega_2_force = [-k_F -k_F -k_F -k_F; 0 -k_F*L 0 k_F*L;...
    k_F*L 0 -k_F*L 0; -k_M k_M -k_M k_M];
matrix_force_2_omega = inv(matrix_omega_2_force);
omega2_needed = matrix_force_2_omega * [F;M];
for j = 1:4
    if omega2_needed(j) <0
        temp = 1;
    end
end
omega_needed = sqrt(omega2_needed);
end

