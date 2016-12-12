function [ filtered_pqr ] = EKF_pqr( measured_pqr,omega )
%EKF_PQR Summary of this function goes here
%   Detailed explanation goes here
global pointer estimated_pqr step I estimated_P k_F L k_M flag_model_uncertainty
I_xx= I(1,1);
I_yy =  I(2,2);
I_zz =  I(3,3);
M = [(-omega(2)^2+omega(4)^2)*k_F*L (omega(1)^2-omega(3)^2)*k_F*L ...
    (-k_M*omega(1)^2+k_M*omega(2)^2-k_M*omega(3)^2+k_M*omega(4)^2)]';
if flag_model_uncertainty == 1
    Q_guess = diag(0.02*[1 1 1]).^2;
else
    Q_guess = zeros(3,3);
end

R_guess = [diag(0.02*[1 1 1])].^2;
%R_guess = zeros(3,3);
H = diag([1 1 1]);
if pointer == 1
     p_estimated_k_1 =0;
     q_estimated_k_1 = 0;
     r_estimated_k_1 = 0;
     estimated_P = zeros(3,3);
     estimated_pqr_last_step = [0 0 0]';
      
else
    %% estimated p q r in last step
    p_estimated_k_1 = estimated_pqr(1,pointer-1);
    q_estimated_k_1 = estimated_pqr(2,pointer-1);
    r_estimated_k_1 = estimated_pqr(3,pointer-1);
    estimated_pqr_last_step =  estimated_pqr(:,pointer-1);
end
    
    P_k_1 = estimated_P;
  
    predicted_pqr = estimated_pqr_last_step +  inv(I)*(M-cross(estimated_pqr_last_step,I*estimated_pqr_last_step))*step;
    F = -inv(I)*[0  (I_zz-I_yy)*r_estimated_k_1  (I_zz-I_yy)*q_estimated_k_1;...
        (I_xx-I_zz)*r_estimated_k_1  0  (I_xx-I_zz)*p_estimated_k_1;...
        (I_yy-I_xx)*q_estimated_k_1 (I_yy-I_xx)*p_estimated_k_1 0];
    Phi_k_k_1 = diag([1 1 1])+F*step;
    
    P_k_k_1 = Phi_k_k_1*P_k_1*Phi_k_k_1'+Q_guess;
    
    K_k = P_k_k_1*H'*inv(H*P_k_k_1*H'+R_guess);
    
    estimated_delta_x = K_k*(measured_pqr-predicted_pqr);
    
    filtered_pqr = predicted_pqr+estimated_delta_x;
    
    estimated_pqr(:,pointer) = filtered_pqr;
    P_k = (diag([1 1 1])-K_k*diag([1 1 1]))*P_k_k_1*(diag([1 1 1])-K_k*H)'+K_k*R_guess*K_k';
    estimated_P = P_k;
end

