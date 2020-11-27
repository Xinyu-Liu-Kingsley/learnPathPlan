function [its,sizePath,run_time] =  RRTstar3D(dim,segmentLength,radius,random_world,show_output,samples)

% if samples < 1000
%     disp('ERROR! SPECIFY ATLEAST 1000 SAMPLES')
%     return
% end

if dim == 2
    start_cord = [5,5];
    goal_cord = [95,95];
elseif dim == 3
    start_cord = [5,5,5];
    goal_cord = [95,95,95];
else
    disp('Error! dim larger than 3 !')
    return;
end

% create random world
Size = 100;
if random_world == 1
    NumObstacles = 10;
    world = createWorld(NumObstacles,ones(1,dim)*Size,zeros(1,dim),dim);
else
    [world NumObstacles] = createKnownWorld(ones(1,dim)*Size,[0;0;0],dim);
end
% randomly select start and end nodes
%start_node = generateRandomNode(world,dim)
%end_node   = generateRandomNode(world,dim)
start_node = [start_cord,0,0,0];
end_node = [goal_cord,0,0,0];
% establish tree starting with the start node
tree = start_node;
numPaths = 0;

a = clock;

% check to see if start_node connects directly to end_node
if ((norm(start_node(1:dim) - end_node(1:dim)) < segmentLength )...
        &&(collision(start_node,end_node,world,dim) == 0))
    disp('CASE 1')
    path = [start_node; end_node];
else
    disp('CASE 2')
    if samples > 0
        draw = floor(samples / 8);
        its = 0;
        for i = 1:samples
            flag = 0;
            [tree,flag] = extendTree(tree,end_node,segmentLength,radius,world,flag,dim);
            numPaths = numPaths + flag;
            its = its + 1;
            
            if its == draw
                tree_1 = tree;
            elseif its == draw*2
                tree_2 = tree;
            elseif its == draw*3
                tree_3 = tree;
            elseif its == draw*4
                tree_4 = tree;
            elseif its == draw*5
                tree_5 = tree;
            elseif its == draw*6
                tree_6 = tree;
            elseif its == draw*7
                tree_7 = tree;
            elseif its == samples
                tree_8 = tree;
            end
        end
    else % sample < 0, then set default parameter 
        its = 0;
        numPaths = 0;
        flag = 0;
        while numPaths < 1
            [tree,flag] = extendTree(tree,end_node,segmentLength,radius,world,flag,dim);
            numPaths = numPaths + flag;
            its = its + 1;
        end
    end
end

% find path with minimum cost to end_node
path = findMinimumPath(tree,end_node,dim);

b = clock;

run_time = 3600*(b(4)-a(4)) + 60 * (b(5)-a(5)) + (b(6) - a(6));

if samples > 0
    path_1 = findMinimumPath(tree_1,end_node,dim);
    path_2 = findMinimumPath(tree_2,end_node,dim);
    path_3 = findMinimumPath(tree_3,end_node,dim);
    path_4 = findMinimumPath(tree_4,end_node,dim);
    path_5 = findMinimumPath(tree_5,end_node,dim);
    path_6 = findMinimumPath(tree_6,end_node,dim);
    path_7 = findMinimumPath(tree_7,end_node,dim);
    path_8 = findMinimumPath(tree_8,end_node,dim);
end

sizePath = size(path,1);

if show_output == 1
    if samples > 0
        if size(path_1, 1) > 0
            figure;
            plotExpandedTree(tree_1,dim);
            plotWorld(world,path_1,dim);
        else
            disp('Could not find a connecting tree till 1/8th samples, no drawing that path')
        end

        if size(path_2, 1) > 0
            figure;
            plotExpandedTree(tree_2,dim);
            plotWorld(world,path_2,dim);
        else
            disp('Could not find a connecting tree till 2/8th samples, no drawing that path')
        end

        if size(path_3, 1) > 0
        figure;
        plotExpandedTree(tree_3,dim);
        plotWorld(world,path_3,dim);
        else
            disp('Could not find a connecting tree till 3/8th samples, no drawing that path')
        end

        if size(path_4, 1) > 0
        figure;
        plotExpandedTree(tree_4,dim);
        plotWorld(world,path_4,dim);
        else
            disp('Could not find a connecting tree till 4/8th samples, no drawing that path')
        end

        if size(path_5, 1) > 0
        figure;
        plotExpandedTree(tree_5,dim);
        plotWorld(world,path_5,dim);
        else
            disp('Could not find a connecting tree till 5/8th samples, no drawing that path')
        end

        if size(path_6, 1) > 0
        figure;
        plotExpandedTree(tree_6,dim);
        plotWorld(world,path_6,dim);
        else
            disp('Could not find a connecting tree till 6/8th samples, no drawing that path')
        end

        if size(path_7, 1) > 0
        figure;
        plotExpandedTree(tree_7,dim);
        plotWorld(world,path_7,dim);
        else
            disp('Could not find a connecting tree till 7/8th samples, no drawing that path')
        end

        if size(path_8, 1) > 0
        figure;
        plotExpandedTree(tree_8,dim);
        plotWorld(world,path_8,dim);
        else
            disp('Could not find a connecting tree till 8/8th samples, no drawing that path')
        end
    end
    
    if size(path, 1) > 0
    figure;
    plotExpandedTree(tree,dim);
    plotWorld(world,path,dim);
    else
        disp('Could not find a connecting tree for the specified samples.PLEASE increase the number of samples!')
    end
end

end


