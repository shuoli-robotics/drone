function [] = plot_sensor()
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
global sensor_states sensor_states_raw drone_states time pointer
figure(4)
subplot(3,1,1)
plot(time(1:pointer-1),drone_states(10,1:pointer-1),'r--','LineWidth',2);
hold on
plot(time(1:pointer-1),sensor_states_raw(10,1:pointer-1),'b:','LineWidth',0.2);
plot(time(1:pointer-1),sensor_states(10,1:pointer-1),'g:','LineWidth',2);
xlabel('t[s]');
ylabel('p[rad/s^2]');
legend('real states','measured stetes','filtered states');

subplot(3,1,2)
plot(time(1:pointer-1),drone_states(11,1:pointer-1),'r--','LineWidth',2);
hold on
plot(time(1:pointer-1),sensor_states_raw(11,1:pointer-1),'b:','LineWidth',0.2);
plot(time(1:pointer-1),sensor_states(11,1:pointer-1),'g:','LineWidth',2);
xlabel('t[s]');
ylabel('q[rad/s^2]');
legend('real states','measured stetes','filtered states');

subplot(3,1,3)
plot(time(1:pointer-1),drone_states(12,1:pointer-1),'r--','LineWidth',2);
hold on
plot(time(1:pointer-1),sensor_states_raw(12,1:pointer-1),'b:','LineWidth',0.2);
plot(time(1:pointer-1),sensor_states(12,1:pointer-1),'g:','LineWidth',2);
xlabel('t[s]');
ylabel('q[rad/s^2]');
legend('real states','measured stetes','filtered states');
end

