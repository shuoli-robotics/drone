function [ filtered_attitude ] = EKF_attitude( measured_attitude,estimated_pqr )
%EKF_ATTITUDE 此处显示有关此函数的摘要
%   此处显示详细说明
global estimated_attitude pointer estimated_P_attitude flag_model_uncertainty step

if flag_model_uncertainty == 1
   % Q_guess = diag(0.02*[1 1 1]).^2;
    Q_guess = zeros(3,3); % no uncertainty in angle model
else
    Q_guess = zeros(3,3);
end

R_guess = [diag(3/180*pi*[1 1 1])].^2;

H = diag([1 1 1]);
if pointer == 1
     phi_estimated_k_1 =0;
     theta_estimated_k_1 = 0;
     psi_estimated_k_1 = 0;
     estimated_P_attitude = zeros(3,3);
     estimated_attitude_last_step = [0 0 0]';
      
else
    %% estimated p q r in last step
    phi_estimated_k_1 = estimated_attitude(1,pointer-1);
    theta_estimated_k_1 = estimated_attitude(2,pointer-1);
    psi_estimated_k_1 = estimated_attitude(3,pointer-1);
    estimated_attitude_last_step =  estimated_attitude(:,pointer-1);
end
    R_d_angle = [1 tan(theta_estimated_k_1)*sin(phi_estimated_k_1) tan(theta_estimated_k_1)*cos(phi_estimated_k_1);...
    0 cos(phi_estimated_k_1) -sin(phi_estimated_k_1);...
    0 sin(phi_estimated_k_1)/cos(theta_estimated_k_1) cos(phi_estimated_k_1)/cos(theta_estimated_k_1)];

    P_k_1 = estimated_P_attitude;
  
    predicted_attitude = estimated_attitude_last_step +  R_d_angle*estimated_pqr*step;
    
    F = [tan(theta_estimated_k_1)*cos(phi_estimated_k_1)-tan(theta_estimated_k_1)*sin(phi_estimated_k_1) ...
        1/cos(theta_estimated_k_1)^2*sin(phi_estimated_k_1) + 1/cos(theta_estimated_k_1)^2*cos(phi_estimated_k_1) 0;...
        -sin(phi_estimated_k_1)-cos(phi_estimated_k_1) 0 0;...
        cos(phi_estimated_k_1)/cos(theta_estimated_k_1)-sin(phi_estimated_k_1)/cos(theta_estimated_k_1) ...
        -sin(phi_estimated_k_1)/cos(theta_estimated_k_1)^2-cos(phi_estimated_k_1)/cos(theta_estimated_k_1)^2 0];
    Phi_k_k_1 = diag([1 1 1])+F*step;
    
    P_k_k_1 = Phi_k_k_1*P_k_1*Phi_k_k_1'+Q_guess;
    
    K_k = P_k_k_1*H'*inv(H*P_k_k_1*H'+R_guess);
    
    estimated_delta_x = K_k*(measured_attitude-predicted_attitude);
    
    filtered_attitude = predicted_attitude+estimated_delta_x;
    
    estimated_attitude(:,pointer) = filtered_attitude;
    P_k = (diag([1 1 1])-K_k*diag([1 1 1]))*P_k_k_1*(diag([1 1 1])-K_k*H)'+K_k*R_guess*K_k';
    estimated_P_attitude = P_k;

end

