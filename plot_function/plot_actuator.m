function [ ] = plot_actuator()
%PLOT_ACTUATOR ????????????
%   ????????
global actuator_states time
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

end

