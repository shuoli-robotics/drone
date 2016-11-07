function [ ] = plot_actuator()
%PLOT_ACTUATOR ????????????
%   ????????
global actuator_states time drone_states
figure(1);
subplot(2,2,1)
plot(time,actuator_states(1,:));
subplot(2,2,2)
plot(time,actuator_states(2,:));
subplot(2,2,3)
plot(time,actuator_states(3,:));
subplot(2,2,4)
plot(time,actuator_states(4,:));
% figure(1);
% plot(actuator_states(1,:));

figure(2)
subplot(4,3,1)
plot(time,drone_states(1,:));
subplot(4,3,2)
plot(time,drone_states(2,:));
subplot(4,3,3)
plot(time,drone_states(3,:));

end

