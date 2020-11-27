function collision_flag = collision(node, parent, world, dim)

collision_flag = 0;
for i = 1:dim
    if (node(i) > world.endcorner(i)) || (node(i) < world.origincorner(i))
        collision_flag = 1;
        return;
    end
end

if dim == 2
    for sigma = 0:.2:1
        p = sigma*node(1:dim) + (1-sigma)*parent(1:dim);
        % check each obstacle
        for i = 1:world.NumObstacles
            if (norm([p(1);p(2)] - [world.cx(i);world.cy(i)]) <= 1*world.radius(i))
                collision_flag = 1;
                return;
            end
        end
    end
elseif dim == 3
    for sigma = 0:.2:1
        p = sigma*node(1:dim) + (1-sigma)*parent(1:dim);
        % check each obstacle
        for i = 1:world.NumObstacles
            if (norm([p(1);p(2);p(3)] - [world.cx(i);world.cy(i); world.cz(i)]) <= 1*world.radius(i))
                collision_flag = 1;
                return;
            end
        end
    end
end

end
