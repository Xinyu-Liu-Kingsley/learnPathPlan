function [nIterations,sizePath,run_time] =  RRT3D(dim,segmentLength,random_world,show_output)
% input �������:
% dim: ���Ϳռ�ά��
% segmentLength: ��չҶ�ڵ�ĳ���
% random_world: %��ʾ�Ƿ�ʹ������ϰ��1�����������趨�õ��ϰ��0��
% show_output: ����������ͼ(1) ������(0)
% output �������:
% nIterations: ��������
% sizePath�� ·������
% run_time�� ����ʱ��[s]


% start_cord - ��ʼ�ڵ�����
% goal_cord - Ŀ��ڵ�����
if dim == 2
    start_cord = [5,5];
    goal_cord = [95,95];
else
    start_cord = [5,5,5];
    goal_cord = [95,95,95];
end

% create world
Size = 100; % �����������ߴ�

if random_world == 1
    NumObstacles = 10; % �ϰ���ĸ���
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
        %ÿ����������ڵ㣬�������������չ��������ڵ㣬����Ϊ���е��½ڵ�
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
