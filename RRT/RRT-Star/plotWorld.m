function plotWorld(world,path,dim)
% the first element is the north coordinate
% the second element is the south coordinate

if dim == 2
    axis([world.origincorner(1),world.endcorner(1),...
        world.origincorner(2), world.endcorner(2)]);
    hold on
    
    N = 10;
    th = 0:2*pi/N:2*pi;
    for i = 1:world.NumObstacles
        X = world.radius(i)*sin(th) + world.cx(i);
        Y = world.radius(i)*cos(th) + world.cy(i);
        fill(X,Y,'blue');
    end
    
    X = path(:,1);
    Y = path(:,2);
    p = plot(X,Y);
    
elseif dim == 3
    axis([world.origincorner(1),world.endcorner(1),...
        world.origincorner(2), world.endcorner(2),...
        world.origincorner(3), world.endcorner(3)]);
    hold on
    
    for i = 1:world.NumObstacles
        [X, Y, Z] = sphere(10);
        X = (X*world.radius(i));
        Y = (Y*world.radius(i));
        Z = (Z*world.radius(i));
        surf(X+world.cx(i),Y+world.cy(i),Z+world.cz(i));
        colormap([0.5 0.2 0.3]);
    end
    
    X = path(:,1);
    Y = path(:,2);
    Z = path(:,3);
    p = plot3(X,Y,Z);
end

set(p,'Color','black','LineWidth',3)
xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');
title('RRT Star Algorithm');

end
