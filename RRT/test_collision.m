clear
clc


for i = 1: 10000
    p_rand = [5*rand() 5*rand() -4*rand()];
    p_rand = [0 0 0];
    if ~collision_check( p_rand )
        p_rand(3) = -p_rand(3);
        plot3(p_rand(1),p_rand(2),p_rand(3),'*');
        hold on;
    end
    
end

plot_obstacle( [2.5 3 1.5],1 );
grid on;
