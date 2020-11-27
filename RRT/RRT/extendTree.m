function [new_tree,flag] = extendTree(tree,end_node,segmentLength,world,dim)

flag = 0;
% select a random point
randomPoint = zeros(1,dim);
if rand < 0.5 
    for i = 1:dim % random sample
        randomPoint(1,i) = (world.endcorner(i) - world.origincorner(i))*rand;
    end 
else
    randomPoint(1,1:2) = end_node(1,1:2); % sample taken as goal to bias tree generation to goal
end

% find leaf on node that is closest to randomPoint
tmp = tree(:,1:dim) - ones(size(tree,1),1)*randomPoint;
sqrd_dist = sqr_eucl_dist(tmp,dim);
[min_dist,idx] = min(sqrd_dist);
min_parent_idx = idx;
new_point = tree(idx,1:dim);
new_node = tree(idx,:);

% 如果采样的点距离最近的leaf小于一定的阈值，则认为这个叶节点扩展过
% 如果不碰撞，则继续前进 segmentLength，直到碰撞为止
Threshold = 0.25;
pflag = 0;
while norm(new_point - randomPoint) > Threshold && pflag == 0 
    if norm(new_point - randomPoint) < segmentLength
        pflag = collision(randomPoint,tree(min_parent_idx,:),world,dim);
        if pflag == 0
            new_point = randomPoint;
            min_cost = cost_np(tree(min_parent_idx,:),new_point,dim);
            new_node = [new_point,0,min_cost,min_parent_idx];
            tree = [tree;new_node];
            pflag = 1;
            goal_flag = is_goal(new_node,end_node,segmentLength,world,dim);
            if goal_flag == 1
                tree(end,dim+1)=1;
                flag = 1;
            end
        end
    else
        new_point = (randomPoint - tree(min_parent_idx,1:dim));
        % 每次前进一个 segmentLength
        new_point = tree(min_parent_idx,1:dim) + (new_point/norm(new_point))*segmentLength;
        
        min_cost  = cost_np(tree(min_parent_idx,:),new_point,dim);
        new_node  = [new_point, 0, min_cost, min_parent_idx];

        pflag = collision(new_node,tree(min_parent_idx,:),world,dim);
        if pflag == 0
            tree = [tree; new_node];
            min_parent_idx = size(tree,1);
           
            goal_flag = is_goal(new_node,end_node,segmentLength,world,dim);
            if goal_flag == 1
                tree(end,dim+1) = 1;  % mark node as connecting to end.
                pflag = 1;
                flag = 1;
            end
        end
    end
end

new_tree = tree;

end
