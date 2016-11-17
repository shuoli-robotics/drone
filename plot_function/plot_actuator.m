function [ ] = plot_actuator()
%PLOT_ACTUATOR ????????????
%   ????????
global actuator_states time drone_states desired_omega pointer desired_angular_velocity
global desired_angle desired_velocity_body desired_position 
global guidance_method

% if strcmp(guidance_method,'CL_RRT')
%     return;
% end



figure(1);
desired_omega(:,pointer) = desired_omega(:,pointer-1);
subplot(2,2,1)
plot(time(1:pointer),actuator_states(1,1:pointer),time(1:pointer),desired_omega(1,1:pointer));
xlabel('t/s');
ylabel('rotor1 speed[rpm]');
legend('real','ref');
subplot(2,2,2)
plot(time(1:pointer),actuator_states(2,1:pointer),time(1:pointer),desired_omega(2,1:pointer));
xlabel('t/s');
ylabel('rotor2 speed[rpm]');
legend('real','ref');
subplot(2,2,3)
plot(time(1:pointer),actuator_states(3,1:pointer),time(1:pointer),desired_omega(3,1:pointer));
xlabel('t/s');
ylabel('rotor3 speed[rpm]');
legend('real','ref');
subplot(2,2,4)
plot(time(1:pointer),actuator_states(4,1:pointer),time(1:pointer),desired_omega(4,1:pointer));
xlabel('t/s');
ylabel('rotor4 speed[rpm]');
legend('real','ref');


figure(2)
desired_angular_velocity(:,pointer) = desired_angular_velocity(:,pointer-1);
desired_angle(:,pointer) = desired_angle(:,pointer-1);
desired_velocity_body(:,pointer) = desired_angle(:,pointer-1);
desired_position(:,pointer) =desired_position(:,pointer-1);
subplot(4,3,1)
plot(time(1:pointer),drone_states(1,1:pointer),time(1:pointer),desired_position(1,1:pointer));
xlabel('t/s');
ylabel('x earth[m]');
legend('real','ref');
subplot(4,3,2)
plot(time(1:pointer),drone_states(2,1:pointer),time(1:pointer),desired_position(2,1:pointer));
xlabel('t/s');
ylabel('y earth[m]');
legend('real','ref');
subplot(4,3,3)
plot(time(1:pointer),drone_states(3,1:pointer),time(1:pointer),desired_position(3,1:pointer));
xlabel('t/s');
ylabel('z earth[m]');
legend('real','ref');
subplot(4,3,4)
plot(time(1:pointer),drone_states(4,1:pointer),time(1:pointer),desired_velocity_body(1,1:pointer));
xlabel('t/s');
ylabel('x body[m/s]');
legend('real','ref');
subplot(4,3,5)
plot(time(1:pointer),drone_states(5,1:pointer),time(1:pointer),desired_velocity_body(2,1:pointer));
xlabel('t/s');
ylabel('y body[m/s]');
legend('real','ref');
subplot(4,3,6)
plot(time(1:pointer),drone_states(6,1:pointer),time(1:pointer),desired_velocity_body(3,1:pointer));
xlabel('t/s');
ylabel('z body[m/s]');
legend('real','ref');
subplot(4,3,7)
plot(time(1:pointer),drone_states(7,1:pointer)/pi*180,time(1:pointer),desired_angle(1,1:pointer)/pi*180);
xlabel('t/s');
ylabel('Phi[degree]');
legend('real','ref');
subplot(4,3,8)
plot(time(1:pointer),drone_states(8,1:pointer)/pi*180,time(1:pointer),desired_angle(2,1:pointer)/pi*180);
xlabel('t/s');
ylabel('Theta[degree]');
legend('real','ref');
subplot(4,3,9)
plot(time(1:pointer),drone_states(9,1:pointer)/pi*180,time(1:pointer),desired_angle(3,1:pointer)/pi*180);
xlabel('t/s');
ylabel('Psi[degree]');
legend('real','ref');
subplot(4,3,10)
plot(time(1:pointer),drone_states(10,1:pointer)/pi*180,time(1:pointer),desired_angular_velocity(1,1:pointer)/pi*180);
xlabel('t/s');
ylabel('p[degree/s]');
legend('real','ref');
subplot(4,3,11)
plot(time(1:pointer),drone_states(11,1:pointer)/pi*180,time(1:pointer),desired_angular_velocity(2,1:pointer)/pi*180);
xlabel('t/s');
ylabel('q[degree/s]');
legend('real','ref');
subplot(4,3,12)
plot(time(1:pointer),drone_states(12,1:pointer)/pi*180,time(1:pointer),desired_angular_velocity(3,1:pointer)/pi*180);
xlabel('t/s');
ylabel('r[degree/s]');
legend('real','ref');

figure(3)
plot3(drone_states(1,1:pointer),drone_states(2,1:pointer),-drone_states(3,1:pointer),'b','LineWidth',1);
hold on;
plot3(desired_position(1,1:pointer),desired_position(2,1:pointer),-desired_position(3,1:pointer),'g','LineWidth',2);
xlabel('x[m]');
ylabel('y[m]');
zlabel('z[m]');
legend('real','ref');
grid on;


