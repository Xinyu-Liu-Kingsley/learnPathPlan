function goal_flag = is_goal(node,end_node,segmentLength,world,dim)

goal_flag = 0;
if (norm(node(1:dim)-end_node(1:dim)) < segmentLength )...
        && (collision(node,end_node,world,dim) == 0)
    goal_flag = 1;
end

end
