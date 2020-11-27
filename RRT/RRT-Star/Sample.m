function [randomPoint] = Sample(world, dim, end_node, prob2goal)
% select a random point
% ������ڵ��� prob2goal �ĸ���ΪĿ���

randomPoint = ones(1,dim);
if rand < prob2goal
    randomPoint(1,1:2) = end_node(1,1:2); % sample taken as goal to bias tree generation to goal
else
    for i = 1:dim % random sample
        randomPoint(1,i) = (world.endcorner(i) - world.origincorner(i))*rand;
    end 
end

end

