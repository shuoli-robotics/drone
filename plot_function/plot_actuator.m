function [ ] = plot_actuator()
%PLOT_ACTUATOR ????????????
%   ????????
global actuator_states time drone_states desired_omega pointer desired_angular_velocity
global desired_angle desired_velocity_body desired_position
figure(1);
desired_omega(:,pointer) = desired_omega(:,end);
subplot(2,2,1)
plot(time,actuator_states(1,:),time,desired_omega(1,:));

subplot(2,2,2)
plot(time,actuator_states(2,:),time,desired_omega(2,:));
subplot(2,2,3)
plot(time,actuator_states(3,:),time,desired_omega(3,:));
subplot(2,2,4)
plot(time,actuator_states(4,:),time,desired_omega(4,:));


figure(2)
desired_angular_velocity(:,pointer) = desired_angular_velocity(:,end);
desired_angle(:,pointer) = desired_angle(:,end);
desired_velocity_body(:,pointer) = desired_angle(:,end);
desired_position(:,pointer) =desired_position(:,end);
subplot(4,3,1)
plot(time,drone_states(1,:),time,desired_position(1,:));
subplot(4,3,2)
plot(time,drone_states(2,:),time,desired_position(2,:));
subplot(4,3,3)
plot(time,drone_states(3,:),time,desired_position(3,:));
subplot(4,3,4)
plot(time,drone_states(4,:),time,desired_velocity_body(1,:));
subplot(4,3,5)
plot(time,drone_states(5,:),time,desired_velocity_body(2,:));
subplot(4,3,6)
plot(time,drone_states(6,:),time,desired_velocity_body(3,:));
subplot(4,3,7)
plot(time,drone_states(7,:),time,desired_angle(1,:));
subplot(4,3,8)
plot(time,drone_states(8,:),time,desired_angle(2,:));
subplot(4,3,9)
plot(time,drone_states(9,:),time,desired_angle(3,:));
subplot(4,3,10)
plot(time,drone_states(10,:),time,desired_angular_velocity(1,:));
subplot(4,3,11)
plot(time,drone_states(11,:),time,desired_angular_velocity(2,:));
subplot(4,3,12)
plot(time,drone_states(12,:),time,desired_angular_velocity(3,:));


end

