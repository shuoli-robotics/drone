function [] = cl_rrt(  )
%CL_RRT 此处显示有关此函数的摘要
%   此处显示详细说明
global tree_pointer  rrt_tree guidance_method pointer drone_states

guidance_method = 'CL_RRT';
max_num = 1000;
goal = [3 5 -1.5];
tree_pointer = 1;
ini_states = zeros(12,1);
ini_states(3,1) = -1.5; 
p_rand_collision = 0;
control_loop_fail = 0;
trajectory_fail = 0;
%% init rrt tree
% line 1-12 are states of the drone at the node.
% line 13 if father of this node
% line 14 is the cost from root to this node
% line 15 is whether this node is the father of the goal 
rrt_tree = zeros(15,1000);
rrt_tree(1:12,1) = ini_states;
rrt_tree(14,1) = 1000000;


figure(6)
plot_obstacle( [2.5 3 1.5],1 );
grid on;
hold on;

for i = 1:max_num
   p_rand = [rand()*5 rand()*7 rand()*-2]';
   
   if collision_check( p_rand )
       p_rand_collision = p_rand_collision + 1;
    continue;
   end
    
   %% sort node based on distance
   nearest_nodes = sort_node(p_rand,rrt_tree);
   
   %% try to connect p_rand with nearest_node
   for j = 1:tree_pointer
       ini_states = rrt_tree(1:12,nearest_nodes(j));
       target_states = p_rand;
       if ~control_guidance_loop(ini_states,target_states)
           control_loop_fail = control_loop_fail + 1;
           continue; % sometimes step is too much for the system
       end

       if check_collision_of_trajectory()
           trajectory_fail =  trajectory_fail + 1;
           continue;
       end
       plot3(drone_states(1,1:pointer),drone_states(2,1:pointer),-drone_states(3,1:pointer));
       
   %% add p_rand and inter nodes to the tree
   add_nodes_to_rrt(nearest_nodes(j));
   
    break;
   end
   
end

end


function origin_position = sort_node(p_rand,rrt_tree)
global tree_pointer
distance_rand_point_node = zeros(tree_pointer,1);

for i = 1:tree_pointer
   distance_rand_point_node(i) = norm(rrt_tree(1:3,i)-p_rand);
end

[~,origin_position] = sort(distance_rand_point_node);

end

function collision = check_collision_of_trajectory()
global drone_states pointer
for k = 1:5:pointer
    if collision_check(drone_states(1:3,k))
               collision = 1;
               return;
     end
 end      
collision = 0;
end

function []=add_nodes_to_rrt(father_node)
% this function is used to add inter node to the tree
global drone_states rrt_tree tree_pointer pointer
inter_nodes_num = 3;
inter_nodes = zeros(15,inter_nodes_num);
for i = 1:inter_nodes_num
    if i ~= inter_nodes_num
        inter_nodes(1:12,i) = drone_states(1:12,ceil(pointer*0.3*rand()));
    else
        inter_nodes(1:12,i) = drone_states(1:12,pointer);
    end
   % father 
   if i == 1
       inter_nodes(13,i) = father_node;
       plot3(inter_nodes(1,i),inter_nodes(2,i),-inter_nodes(3,i),'o');
   else
      inter_nodes(13,i) = tree_pointer+i-1; 
        plot3(inter_nodes(1,i),inter_nodes(2,i),-inter_nodes(3,i),'*');
   end   
end
rrt_tree(:,tree_pointer+1:tree_pointer+inter_nodes_num) = inter_nodes;
tree_pointer = tree_pointer + inter_nodes_num;


end
