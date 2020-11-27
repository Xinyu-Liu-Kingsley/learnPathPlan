%  /*RRT-Connect algoritm in 2D with collision avoidance
% *Version: MATLAB 2017b
% *Time: 2020.07.07
% *Author: Joey Zhou */

close all;
clear;
clc;

%% ���ɽ�ͼ
map = ones(500,500);
% map 1
map(1:450,70:120) = 0;
map(50:500,350:400) = 0;
% % map 2
% map(250:450,70:120) = 0;
% map(50:250,350:400) = 0;
imshow(map);
[m,n] = size(map);

%% �����ʼ�������������Ͻ�Ϊԭ������������һ������ʾy���ڶ�������ʾx
qinit = [100,20];          %��ʼ��
qgoal = [480,480];         %Ŀ���
stepsize = 20;             %��������
disTh = 20;                %��ֵ
maxFailAttempts = 2000;    %����ʧ�ܴ�������
counter = 0;               %��������
display = 1;               %��ʾ��������Ϊ��

%% �ж���ʼ���Ŀ����Ƿ����Ҫ��
if feasiblePoint(qinit,map) == 0
	error('��ʼ�㲻���ϵ�ͼҪ��'); %error����ĳ��򲻻ᱻִ��
end

if feasiblePoint(qgoal,map) == 0
	error('Ŀ��㲻���ϵ�ͼҪ��'); %error����ĳ��򲻻ᱻִ��
end

%% ��ʾ��ͼ
if display
	imshow(map);
    rectangle('position',[1 1 size(map)-1],'LineWidth', 2,'edgecolor','k');
    hold on;
    scatter(qinit(2),qinit(1),'sm','filled');
    hold on;
    scatter(qgoal(2),qgoal(1),'sb','filled');
end

%% ����ʼ����ʱ
tic;
RRTree1 = double([qinit 0]);   %��ʼ�������
RRTree2 = double([qgoal -1]);  %��ֹ�������
Tree1ExpansionFail = 0;
Tree2ExpansionFail = 0;
while ~Tree1ExpansionFail || ~Tree2ExpansionFail
    % ��չ��һ����
    if ~Tree1ExpansionFail
        [RRTree1,pathFound,Tree1ExpansionFail] = rrtExtend(RRTree1,RRTree2,qgoal,stepsize,maxFailAttempts,disTh,map);%��չTree1��Tree1ExpansionFail�ķ���ֵ��ԶΪ0
         if ~Tree1ExpansionFail && isempty(pathFound) && display   %�����չ���ɹ���ͬʱû�е������յ�
           % Tree1�½ڵ��븸�ڵ�����
             line([RRTree1(end,2);RRTree1(RRTree1(end,3),2)],[RRTree1(end,1);RRTree1(RRTree1(end,3),1)],'color','b','linewidth',3);
             counter = counter + 1;
             M(counter) = getframe;
         end
    end
    % ��չ�ڶ�����
    if ~Tree2ExpansionFail
        [RRTree2,pathFound,Tree2ExpansionFail] = rrtExtend(RRTree2,RRTree1,qinit,stepsize,maxFailAttempts,disTh,map);
        if ~isempty(pathFound)
            pathFound(3:4) = pathFound(4:-1:3); % ����λ��,����һ�£��Ա����ı��
        end 
    end
    
    % �ҵ�·��
    if ~isempty(pathFound)       
        if display
            line([RRTree1(pathFound(1,3),2);pathFound(1,2);RRTree2(pathFound(1,4),2)],[RRTree1(pathFound(1,3),1);pathFound(1,1);RRTree2(pathFound(1,4),1)],'color','green','linewidth',3);
            counter = counter+1;
            M(counter) = getframe;
        end
        path = [pathFound(1,1:2)]; % compute path
        prev = pathFound(1,3);     % add nodes from RRT 1 first
        while prev > 1
            path = [RRTree1(prev,1:2);path];
            prev = RRTree1(prev,3);
        end
        prev = pathFound(1,4);     % then add nodes from RRT 2
        while prev > 0
            path = [path;RRTree2(prev,1:2)];
            prev = RRTree2(prev,3);
        end
        break;
    end
    
    % ������������
    [RRTree1,RRTree2] = Swap(RRTree1,RRTree2);
end
toc;

line(path(:,2),path(:,1),'linestyle','--','linewidth',2,'color','y');
title('2D RRT-Connect Algorithm');

