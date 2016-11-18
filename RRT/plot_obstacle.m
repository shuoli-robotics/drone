function [  ] = plot_obstacle( center,edge )
%PLOT_OBSTACLE Summary of this function goes here
%   Detailed explanation goes here
point = zeros(3,5);
point(:,1) = [center(1)-edge/2 center(2) center(3)-edge/2];
point(:,2) = [center(1)+edge/2 center(2) center(3)-edge/2];
point(:,3) = [center(1)+edge/2 center(2) center(3)+edge/2];
point(:,4) = [center(1)-edge/2 center(2) center(3)+edge/2];
point(:,5) = point(:,1);
plot3(point(1,:),point(2,:),point(3,:),'b','LineWidth',5);
% for i = 1:4
%    plot3(point(:,i),point(:,i)); 
% end

end
