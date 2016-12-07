%% 
close all
clear all 
VarName = csvread('yellow_set_theta_3.csv');
%VarName = csvread('New_data_set\long_hand_1.csv');

%Process data from test data sets
%Straight flight
% full bat 12.0
% High speed

%rename used variables
counter = VarName(:,1); time_stamp = VarName(:,2);

%gyro scale 0.0139882 for deg/sec
gyro_scale = deg2rad(0.0139882);

%acc scale 0.0009766 for m/s2
acc_scale = 0.0009766;

gyro_p = VarName(:,3)*gyro_scale; gyro_q = VarName(:,4)*gyro_scale; gyro_r = VarName(:,5)*gyro_scale;
acc_x = VarName(:,6)*acc_scale; acc_y = VarName(:,7)*acc_scale; acc_z = VarName(:,8)*acc_scale;
phi = VarName(:,12); theta = VarName(:,13); psi = VarName(:,14);
pos_x = VarName(:,15); pos_y =VarName(:,16); pos_z = VarName(:,17);
vel_x = VarName(:,18); vel_y = VarName(:,19); vel_z = VarName(:,20);

[data_size dummy ] = size(counter);

data_size

psi_offset = deg2rad(0);
%initialization
prev_time  = time_stamp(1);
speed_nav = [0 0 0];
pos_nav = [0 0 0];

% prev_speed = speed_nav;
prev_pos = pos_nav;

%calibration pqr and acc
end_init = 300;

%initial bias estimation based on avaraging when drone is on ground
gyro_bias = [ mean(gyro_p(1:end_init)) mean(gyro_q(1:end_init)) mean(gyro_r(1:end_init))];
%acc_bias  = [ mean(acc_x(1:end_init)) mean(acc_y(1:end_init)) mean(acc_z(1:end_init))]';

%EKF init
EKF_count = 0;
HOLD_count = 0;
prev_EKF_time = 0;
EKF_time_vec = [0];
count = 0;

bias_acc  = [0 0 0]';
bias_gyro = [-0.02 -0.02 0]';

%initialize state with biases.
X_int = [0.5 0 -1.2 0.4 0 0 0 0 0 0 0 0 0 0 0];
X_int_prev = X_int;

P_k_1 = zeros(15,15);
P_k_1_k_1 = eye(15,15)*1;%initialize with large variances?


Q = diag([1.0 1.0 5.0 0.01 0.01 0.05]);

%position errorvariances
R_k = diag([0.1 0.1 0.1]);%for optitrack withput noise
%R_k = diag([1.5 1.5 1.5]);%for optitrack with noise

% extract position from state prediction
H_k = [eye(3,3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3)];

Jac_mat_f = kf_calc_jacobian();%
Jac_mat_h = kf_calc_jacobian_h();

for k = 2:data_size
   
   %sample dt
   dt = time_stamp(k)-time_stamp(k-1);
   
   %angular rate vextor
   omega = [gyro_p(k) gyro_q(k) gyro_r(k)];

   %input vector for kalman filter prediction
   U_k = [acc_x(k) acc_y(k) acc_z(k) omega];

   %prediction is basic integration of kinematics 
   X_int = [X_int; X_int_prev + kf_calc_f_nl(dt,X_int_prev,U_k)'*dt];
   
   %position in nav frame
   pos_nav = [pos_nav;X_int(1:3)];
   
   %EKF update at 20 hz
   if(EKF_count < 25)
       EKF_count = EKF_count + 1;
   else
        count = count + 1;
        EKF_dt = time_stamp(k)- prev_EKF_time;
        prev_EKF_time = time_stamp(k);
%        %EKF update here:

        %discretization 
        x_kk_1 = X_int(k,:);
        G = kf_calc_G_nl(x_kk_1);
        DFx = kf_calc_Fx_nl(Jac_mat_f, x_kk_1, U_k);
        [Phi, Gamma] = c2d(DFx, G,EKF_dt);
        
        % P(k+1|k) (prediction covariance matrix)
        P_k_1 = Phi*P_k_1_k_1*Phi' + Gamma*Q*Gamma';
        
        DHx = kf_calc_Hx_nl(Jac_mat_h, x_kk_1, U_k);
        
        %kalman gain
        K = P_k_1 * DHx' / (DHx*P_k_1 * DHx' + R_k);

        %measurement vector of xyz position
        z_k = [pos_x(k) pos_y(k) pos_z(k)];
        
        %optimal EKF state estimate
        X_opt = x_kk_1' + K * (z_k - X_int(k,1:3))';
        
        %update predicted state with optimal EKF state
        X_int(k,:) = X_opt;
        
        P_k_1_k_1 = (eye(15) - K*DHx) * P_k_1 * (eye(15) - K*DHx)' + K*R_k*K';
        
       EKF_count = 0;
   end
   
   %speed in nav frame
   speed_nav = [speed_nav;(C_b_n(X_int(k,7),X_int(k,8),X_int(k,9))*X_int(k,4:6)')'];
   
   X_int_prev = X_int(k,:);

end

% % %gyro int phi
% % figure
% % plot(time_stamp,rad2deg(angle_int(:,1)),time_stamp,rad2deg(phi(:)))
% % title('phi')
% % 
% % %gyro int theta
% % figure
% % plot(time_stamp,rad2deg(angle_int(:,2)),time_stamp,rad2deg(theta(:)))
% % title('theta')
% % 
% % %gyro int psi
% % figure
% % plot(time_stamp,rad2deg(angle_int(:,3)),time_stamp,rad2deg(psi(:)))
% % title('psi')
% % 
% % % %acc x nav frame
% % % figure
% % % plot(time_stamp,acc_nav(:,1))%,time_stamp,rad2deg(psi(:)))
% % % title('acc x nav frame')
% % % 
% % % %acc y nav frame
% % % figure
% % % plot(time_stamp,acc_nav(:,2))%,time_stamp,rad2deg(psi(:)))
% % % title('acc y nav frame')
% % % 
% % % %acc z nav frame
% % % figure
% % % plot(time_stamp,acc_nav(:,3))%,time_stamp,rad2deg(psi(:)))
% % % title('acc z nav frame')
% % 
% % %raw acc x
% % figure
% % plot(time_stamp,acc_x)
% % title('raw acc x')
% % 
% % %raw acc y
% % figure
% % plot(time_stamp,acc_y)
% % title('raw acc y')
% % 
% % %raw acc z
% % figure
% % plot(time_stamp,acc_z)
% % title('raw acc z')
% % 
% % %raw acc x
% % figure
% % plot(time_stamp,gyro_p)
% % title('raw gyro p')
% % 
% % %raw acc y
% % figure
% % plot(time_stamp,gyro_q)
% % title('raw gyro q')
% % 
% % %raw acc z
% % figure
% % plot(time_stamp,gyro_r)
% % title('raw gyro r')
% % 
% % %speed x
% % figure
% % plot(time_stamp,speed_nav(:,1),time_stamp,vel_x)
% % title('speed x')
% % 
% % %speed y
% % figure
% % plot(time_stamp,speed_nav(:,2),time_stamp,vel_y)
% % title('speed y')
% % 
% % %speed z
% % figure
% % plot(time_stamp,speed_nav(:,3),time_stamp,vel_z)
% % title('speed z')
% 
% % speed x
% figure
% plot(time_stamp,vel_x,time_stamp,speed_nav(:,1))
% title('speed x')
% 
% %speed y
% figure
% plot(time_stamp,vel_y,time_stamp,speed_nav(:,2))
% title('speed y')
% 
% %speed z
% figure
% plot(time_stamp,vel_z,time_stamp,speed_nav(:,3))
% title('speed z')
% 
% % bias p
% figure
% plot(time_stamp,X_int(:,13))
% title('bias p')
% 
% %bias q
% figure
% plot(time_stamp,X_int(:,14))
% title('bias q')
% 
% %bias r
% figure
% plot(time_stamp,X_int(:,15))
% title('bias r')
% 
% %gyro int phi
% figure
% plot(time_stamp,rad2deg(X_int(:,7)),time_stamp,rad2deg(phi(:)))
% title('phi')
% 
% %gyro int theta
% figure
% plot(time_stamp,rad2deg(X_int(:,8)),time_stamp,rad2deg(theta(:)))
% title('theta')
% 
% %gyro int psi
% figure
% plot(time_stamp,rad2deg(X_int(:,9)),time_stamp,rad2deg(psi(:)))
% title('psi')
% 
% %3d plot
 figure(22)

plot3(X_int(:,1),X_int(:,2),-X_int(:,3),pos_x,pos_y,-pos_z)
title('position xyz')
%axis([-1 7, -2 6, 0 3])
axis([0 8 0 0.3 0 2]);
grid on

figure (21)
subplot(2,3,1)
plot(time_stamp,vel_x,time_stamp,speed_nav(:,1));
xlabel('t[s]');
ylabel('v_x[m/s]');
subplot(2,3,2)
plot(time_stamp,vel_y,time_stamp,speed_nav(:,2));
xlabel('t[s]');
ylabel('v_y[m/s]');
subplot(2,3,3)
plot(time_stamp,vel_y,time_stamp,speed_nav(:,3))
xlabel('t[s]');
ylabel('v_z[m/s]');
subplot(2,3,4)
plot(time_stamp,rad2deg(X_int(:,7)),time_stamp,rad2deg(phi(:)))
xlabel('t[s]');
ylabel('phi[degree]');
subplot(2,3,5)
plot(time_stamp,rad2deg(X_int(:,8)),time_stamp,rad2deg(theta(:)))
xlabel('t[s]');
ylabel('theta[degree]');
subplot(2,3,6)
plot(time_stamp,rad2deg(X_int(:,9)),time_stamp,rad2deg(psi(:)))
xlabel('t[s]');
ylabel('psi[degree]');
