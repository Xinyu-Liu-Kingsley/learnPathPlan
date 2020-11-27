function [nIterations,sizePath,run_time] =  RRT3D(dim,segmentLength,random_world,show_output)
% input 输入参数:
% dim: 构型空间维数
% segmentLength: 扩展叶节点的长度
% random_world: %表示是否使用随机障碍物（1）还是事先设定好的障碍物（0）
% show_output: 绘制搜索结图(1) 不绘制(0)
% output 输出参数:
% nIterations: 迭代次数
% sizePath： 路径步数
% run_time： 运行时长[s]


% start_cord - 开始节点坐标
% goal_cord - 目标节点坐标
if dim == 2
    start_cord = [5,5];
    goal_cord = [95,95];
else
    start_cord = [5,5,5];
    goal_cord = [95,95,95];
end

% create world
Size = 100; % 世界的坐标轴尺寸

if random_world == 1
    NumObstacles = 10; % 障碍物的个数
    world = createWorld(NumObstacles,ones(1,dim)*Size,zeros(1,dim),dim);
else
    [world NumObstacles] = createKnownWorld(ones(1,dim)*Size,[0;0;0],dim);
end

start_node = [start_cord,0,0,0];
end_node = [goal_cord,0,0,0];
% establish tree starting with the start node
tree = start_node;
maxIter = 10000; % max iterations times

a = clock;

% check to see if start_node connects directly to end_node
if ((norm(start_node(1:dim) - end_node(1:dim)) < segmentLength )...
        &&(collision(start_node,end_node,world,dim) == 0))
    path = [start_node; end_node];
else
    nIterations = 0;
    numPaths = 0;
    flag = 0;
    while numPaths < 1 && (nIterations < maxIter)
        %每次生成随机节点，从树中最近点拓展到该随机节点，并作为树中的新节点
        [tree,flag] = extendTree(tree,end_node,segmentLength,world,dim);
        numPaths = numPaths + flag;
        nIterations = nIterations + 1;
    end
    
    if flag == 0
        disp("Can not find the path");
    end
end

% find path with minimum cost to end_node
path = findMinimumPath(tree,end_node,dim);
sizePath = size(path,1);

b = clock;

% calculate simulation time [s]
run_time = 3600*(b(4) - a(4)) + 60 * (b(5) - a(5)) + (b(6) - a(6));

if show_output == 1
    figure;
    plotExpandedTree(tree,dim);
    plotWorld(world,path,dim);
end

end
