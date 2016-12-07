function [ output_args ] = data_process( input_args )
%DATA_PROCESS 此处显示有关此函数的摘要
%   此处显示详细说明
flight_raw_data = csvread('yellow_set_theta_3.csv');
[m,n] = size(flight_raw_data);
m = 2500;  %!!!!!!!!!!!!!!!!!
flight_data = zeros(m,n+6);
id_counter = 1;
id_time = 2;

id_p = 3;
id_q = 4;
id_r = 5;

id_ax_body = 6;
id_ay_body = 7;
id_az_body = 8;

id_phi = 12;
id_theta = 13;
id_psi = 14;

id_x = 15;
id_y = 16;
id_z = 17;

id_vx = 18;
id_vy = 19;
id_vz = 20;

id_command_trust = 21;

id_sp_phi = 22;
id_sp_theta = 23;
id_sp_psi = 24;

id_rpm_ref1 = 27;
id_rpm_ref2 = 28;
id_rpm_ref3 = 29;
id_rpm_ref4 = 30;

id_rpm_obs1 = 31;
id_rpm_obs2 = 32;
id_rpm_obs3 = 33;
id_rpm_obs4 = 34;

id_vx_body = 35;
id_vy_body = 36;
id_vz_body = 37;

id_ax = 38;
id_ay = 39;
id_az = 40;

% scale p q r and ax ay az
flight_data(1:m,1:n) = flight_raw_data(1:m,1:n);
flight_data(1:m,id_p:id_r) = flight_raw_data(1:m,id_p:id_r)*0.0139882;
flight_data(1:m,id_ax_body:id_ay_body) = flight_raw_data(1:m,id_ax_body:id_ay_body)*0.0009766;

p = 1;
for i = 1:m
   phi = flight_raw_data(i,id_phi);
   theta = flight_raw_data(i,id_theta);
   psi = flight_raw_data(i,id_psi);
    R_E_B = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
     sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
     sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
     cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
     cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
 
%  flight_data(i,id_vx_body:id_vz_body) = [R_E_B*[flight_raw_data(i,id_vx:id_vz)]']';
%  flight_data(i,id_ax_body:id_az_body) = [R_E_B*[flight_raw_data(i,id_ax:id_az)]']';
if i ~= 1 && rem(i,20) == 0
    theta(p) = flight_data(i,id_theta);
    v_x(p) = flight_data(i,id_vx);
    if p == 1
        a_x(p) = 0;
    else
        a_x(p) = (v_x(p)-v_x(p-1))*512/20;
    end  
    p = p+1;
end
end

figure(1);
subplot(4,3,1)
plot(flight_data(1:m,id_time),flight_data(1:m,id_x));
xlabel('t/s');
ylabel('x[m]');
subplot(4,3,2)
plot(flight_data(1:m,id_time),flight_data(1:m,id_y));
xlabel('t/s');
ylabel('y[m]');
subplot(4,3,3)
plot(flight_data(1:m,id_time),flight_data(1:m,id_z));
xlabel('t/s');
ylabel('z[m]');
subplot(4,3,4)
plot(flight_data(1:m,id_time),flight_data(1:m,id_vx));
xlabel('t/s');
ylabel('vx[m/s]');
subplot(4,3,5)
plot(flight_data(1:m,id_time),flight_data(1:m,id_vy));
xlabel('t/s');
ylabel('vy[m/s]');
subplot(4,3,6)
plot(flight_data(1:m,id_time),flight_data(1:m,id_vz));
xlabel('t/s');
ylabel('vz[m/s]');
subplot(4,3,7)
% plot(flight_data(1:m,id_time),flight_data(1:m,id_sp_phi)/pi*180,...
%     flight_data(1:m,id_time),flight_data(1:m,id_phi)/pi*180);
plot(flight_data(1:m,id_time),flight_data(1:m,id_phi)/pi*180);
xlabel('t/s');
ylabel('phi[degree]');
%legend('phi setpoint','phi measured');
subplot(4,3,8)
% plot(flight_data(1:m,id_time),flight_data(1:m,id_sp_theta)/pi*180,...
%     flight_data(1:m,id_time),flight_data(1:m,id_theta)/pi*180);
plot(flight_data(1:m,id_time),flight_data(1:m,id_theta)/pi*180);
xlabel('t/s');
ylabel('theta[degree]');
% legend('theta setpoint','theta measured');
subplot(4,3,9)
% plot(flight_data(1:m,id_time),flight_data(1:m,id_sp_psi)/pi*180,...
%     flight_data(1:m,id_time),flight_data(1:m,id_psi)/pi*180);
plot(flight_data(1:m,id_time),flight_data(1:m,id_psi)/pi*180);
xlabel('t/s');
ylabel('psi[degree]');
%legend('psi setpoint','psi measured');

subplot(4,3,10)
plot(flight_data(1:m,id_time),flight_data(1:m,id_p));
xlabel('t/s');
ylabel('p[deg/s]');

subplot(4,3,11)
plot(flight_data(1:m,id_time),flight_data(1:m,id_q));
xlabel('t/s');
ylabel('q[deg/s]');

subplot(4,3,12)
plot(flight_data(1:m,id_time),flight_data(1:m,id_r));
xlabel('t/s');
ylabel('r[deg/s]');
a_11 = 0;
a_12 = 0;
a_21 = 0;
a_22 = 0;
b_11 = 0;
b_21 = 0;
g = 9.8;
mass = 0.414;
% for i = 1:m
%    a_11 = a_11+flight_data(i,id_vx)^2;
%    a_12 = a_12+abs(flight_data(i,id_vx))*flight_data(i,id_vx)^2;
%    a_21 = a_21+abs(flight_data(i,id_vx))*flight_data(i,id_vx)^2;
%    a_22 = flight_data(i,id_vx)^4;
%    b_11 = b_11+flight_data(i,id_vx)*(flight_data(i,id_ax)-g*-flight_data(i,id_theta));
%    b_21 = b_21+abs(flight_data(i,id_vx))*flight_data(i,id_vx)*(flight_data(i,id_ax)-g*-flight_data(i,id_theta));
% end

% A = [a_11 a_12;a_21 a_22];
% B = [b_11;b_21];
% 
% parameter = inv(A)*B;
% a_bla = parameter(1)/g;
% k_par = parameter(2)*mass;
drag_measured = zeros(p-1,1);
for i = 1:p-1
    drag_measured(i) =a_x(i)-g*-theta(i);
end
end

