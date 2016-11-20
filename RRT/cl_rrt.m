function [optimal_trajectory] = cl_rrt(  )
%CL_RRT ????????????????????????
%   ????????????????
global tree_pointer  rrt_tree guidance_method pointer drone_states goal

guidance_method = 'CL_RRT';
max_num = 200;
goal = [2.5 5 -1.5];
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
% line 16 is the flag shows that if this node is end node or internode
rrt_tree = zeros(16,10000);
rrt_tree(1:12,1) = ini_states;
fail_to_connect = 0;

figure(6)
plot_obstacle( [2.5 3 1.5],1 );
grid on;
hold on;

for i = 1:max_num
    if rand()<0.05
        p_rand = generate_random_point_in_safe_zone();
    else
        p_rand = [rand()*5 rand()*7 rand()*-2]';
    end
    
   if collision_check( p_rand )
       p_rand_collision = p_rand_collision + 1;
    continue;
   end
    
   %% sort node based on distance
   nearest_nodes = sort_node(p_rand,rrt_tree);
   
   %% try to connect p_rand with nearest_node
   for j = 1:tree_pointer
       if (rrt_tree(16,nearest_nodes(j)) == 1)  % we don't want to connect to end points
          if rand()>0.01
              continue;
          end
       end
       ini_states = rrt_tree(1:12,nearest_nodes(j));
       target_states = p_rand;
       if ~control_guidance_loop(ini_states,target_states)
           control_loop_fail = control_loop_fail + 1;
           continue; % sometimes step is too much for the system
       end

       if check_collision_of_trajectory()
           trajectory_fail =  trajectory_fail + 1;
           if j == tree_pointer
               fail_to_connect = 1;     % each nodes on the tree cannot connect with p_rand successfully
           end
           continue;
       end
       plot3(drone_states(1,1:pointer),drone_states(2,1:pointer),-drone_states(3,1:pointer),'--');
       
   %% add p_rand and inter nodes to the tree
   [inter_nodes,inter_nodes_num] = add_nodes_to_rrt(nearest_nodes(j));
   
    break;
   end
   if fail_to_connect == 1
      fail_to_connect =0;
      continue;
   end
   %% connect nodes to goal
   connect_new_nodes_to_goal(inter_nodes,inter_nodes_num);
end

%% find optimal trajectory
 [optimal_trajectory]=find_optimal_trajectory();
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
for k = 1:50:pointer
    if collision_check(drone_states(1:3,k))
               collision = 1;
               return;
     end
 end      
collision = 0;
end

function [inter_nodes,inter_nodes_num]=add_nodes_to_rrt(father_node)
% this function is used to add inter node to the tree
global drone_states rrt_tree tree_pointer pointer
inter_nodes_num = 3;
inter_nodes = zeros(16,inter_nodes_num);
trajectory_step = (drone_states(1,pointer) - drone_states(1,1))/inter_nodes_num;
for i = 1:inter_nodes_num
    for j = 1:pointer   % can be improved
        if abs(drone_states(1,j)-(drone_states(1,1)+trajectory_step*i)) < 0.1
            if i ~= inter_nodes_num
                
                inter_nodes(1:12,i) = drone_states(1:12,j);
                if i == 1   % first node
                    inter_nodes(13,i) = father_node;
                    inter_nodes(14,i) = rrt_tree(14,father_node)+norm(inter_nodes(1:3,i)-rrt_tree(1:3,father_node));
                else % second node and second last node
                    inter_nodes(13,i) = tree_pointer+i-1;
                    inter_nodes(14,i) =rrt_tree(14,tree_pointer+i-1) + norm(inter_nodes(1:3,i)-rrt_tree(1:3,tree_pointer+i-1));
                end
                plot3(inter_nodes(1,i),inter_nodes(2,i),-inter_nodes(3,i),'.');
            else  % last node
                inter_nodes(1:12,i) = drone_states(1:12,pointer);
                inter_nodes(13,i) = tree_pointer+i-1;
                % since we don't want to connect end node, we add 1 to the
                % distance
                inter_nodes(14,i) = rrt_tree(14,tree_pointer+i-1)+norm(inter_nodes(1:3,i)-rrt_tree(1:3,tree_pointer+i-1))+1;
                inter_nodes(16,i) = 1;
                plot3(inter_nodes(1,i),inter_nodes(2,i),-inter_nodes(3,i),'o');
            end
            break;
        end
    
    end
    
     
end
rrt_tree(:,tree_pointer+1:tree_pointer+inter_nodes_num) = inter_nodes;
tree_pointer = tree_pointer + inter_nodes_num;
end


function [p_rand] = generate_random_point_in_safe_zone()
global x_gate y_gate z_gate y_safe_zone l_gate

p_rand = [x_gate + (-1)^round(rand()) * rand()*0.5*l_gate ... 
    y_gate + (-1)^round(rand()) * rand()*0.5*y_safe_zone ...
    z_gate + (-1)^round(rand()) * rand()*0.5*l_gate]';
end


function [] = connect_new_nodes_to_goal(inter_nodes,inter_nodes_num)
global goal rrt_tree tree_pointer pointer drone_states

for i = 1:inter_nodes_num
    if ( norm(inter_nodes(1:3,i)-goal') > 2 || inter_nodes(16,i) == 1)
        continue;
    end
    ini_states = inter_nodes(1:12,i);
       target_states = goal;
       if ~control_guidance_loop(ini_states,target_states)
           continue; % sometimes step is too much for the system
       end
       if check_collision_of_trajectory()
           continue;
       else
           rrt_tree(15,tree_pointer-inter_nodes_num+i) = 1;
           plot3(drone_states(1,1:pointer),drone_states(2,1:pointer),-drone_states(3,1:pointer),'--');
       end
       
end
end


function [optimal_trajectory]=find_optimal_trajectory()
global rrt_tree tree_pointer goal

minimum_dis = 10000;
trajectory_pointer = 1;
optimal_trajectory_temp = zeros(3,tree_pointer);


for i = 1:tree_pointer
   if (rrt_tree(15,i) == 1)
      rrt_tree(14,i) = rrt_tree(14,i)+norm(goal-rrt_tree(1:3,i));
      if  rrt_tree(14,i) < minimum_dis
          minimum_dis = rrt_tree(14,i);
          minimum_node = i;
      end
   end
end

while (minimum_node ~= 1)
   optimal_trajectory_temp(1:3,trajectory_pointer) = rrt_tree(1:3,minimum_node);
   minimum_node = rrt_tree(13,minimum_node);
   trajectory_pointer = trajectory_pointer + 1;
end
optimal_trajectory = zeros(3,trajectory_pointer+1);
optimal_trajectory(:,1) = goal';
optimal_trajectory(:,2:end-1) =  optimal_trajectory_temp(:,1:trajectory_pointer-1);
optimal_trajectory(:,end) = rrt_tree(1:3,1);
%[goal' rrt_tree(1:3,1) optimal_trajectory_temp(:,1:trajectory_pointer-1) rrt_tree(1:3,1)];
plot3(optimal_trajectory(1,:),optimal_trajectory(2,:),-optimal_trajectory(3,:),'r','LineWidth',2);
end