function [cost] = cost_np(from_node,to_point,dim)
% calculate the cost from a node to a point

diff = from_node(:,1:dim) - to_point;
eucl_dist = norm(diff);
cost = from_node(:,dim+2) + eucl_dist;

end