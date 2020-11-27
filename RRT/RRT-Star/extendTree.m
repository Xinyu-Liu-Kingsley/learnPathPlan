function [new_tree,flag] = extendTree(tree,end_node,segmentLength,r,world,flag_chk,dim)

flag1 = 0;
while flag1 == 0
    % select a random point
    [randomPoint] = Sample(world, dim, end_node, 0.4);
    % find leaf on node that is closest to randomPoint
    tmp = tree(:,1:dim) - ones(size(tree,1),1)*randomPoint;
    sqrd_dist = sqr_eucl_dist(tmp,dim);
    [min_dist,idx] = min(sqrd_dist);
    min_parent_idx = idx;
    
    new_point = (randomPoint - tree(idx,1:dim));
    new_point = tree(idx,1:dim) + (new_point/norm(new_point))*segmentLength;
    
    min_cost  = cost_np(tree(idx,:),new_point,dim);
    new_node  = [new_point, 0, min_cost, idx]; % 随机生成的新节点，步骤和RRT一样
    
    if collision(new_node, tree(idx,:), world,dim) == 0 % 检查新节点是否碰撞
        tmp_dist = tree(:,1:dim) - (ones(size(tree,1),1)*new_point);
        dist = sqr_eucl_dist(tmp_dist,dim);
        near_idx = find(dist <= r^2); % 以新节点为圆心，找半径 r 内的所有节点
        
        if size(near_idx,1) > 1
            size_near = size(near_idx,1);
            for i = 1:size_near
                cost_near = tree(near_idx(i),dim+2) + line_cost(tree(near_idx(i),:),new_point,dim);
                if (cost_near < min_cost) && (collision(new_node, tree(near_idx(i),:), world,dim) == 0)
                    min_cost = cost_near;
                    min_parent_idx = near_idx(i);
                end
            end
        end
        
        new_node = [new_point, 0 , min_cost, min_parent_idx]; % 更新新节点的最好的爸爸及代价
        new_tree = [tree; new_node]; % 将新节点加入树
        new_node_idx = size(new_tree,1); % 新节点的树索引
        
        if size(near_idx,1) > 1
            reduced_idx = near_idx;
            for j = 1:size(reduced_idx,1)
                near_cost = new_tree(reduced_idx(j),dim+2);
                lcost = line_cost(new_tree(reduced_idx(j),:),new_point,dim); % 重布线过程，也就是新节点找儿子
                if near_cost > min_cost + lcost ...
                        && collision(new_tree(reduced_idx(j),:),new_node,world,dim)
%                     before = new_tree(reduced_idx(j),dim+3);
                    new_tree(reduced_idx(j),dim+3) = new_node_idx; % 将原来的父节点更新为新的节点
%                     after = new_tree(reduced_idx(j),dim+3);
%                     fprintf('before parent %d, after update to new node %d\n',before,after);
                end
            end
        end
        flag1 = 1;
    end
end

if flag_chk == 0
    % check to see if new node connects directly to end_node
    if ((norm(new_node(1:dim) - end_node(1:dim)) < segmentLength )...
            && (collision(new_node,end_node,world,dim) == 0))
        flag = 1;
        new_tree(end,dim+1) = 1;  % mark node as connecting to end.
    else
        flag = 0;
    end
else
    flag = 1;
end

end
