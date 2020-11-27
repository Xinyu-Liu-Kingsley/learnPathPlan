function [cost] = line_cost(from_node,to_point,dim)
% caculate the cost from node to new point use euler distance

diff = from_node(:,1:dim) - to_point;
cost = norm(diff);

end
