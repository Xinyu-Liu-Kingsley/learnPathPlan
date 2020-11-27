function [RRTree,pathFound,extendFail] = rrtExtend(RRTreeA,RRTreeB,qgoal,stepsize,maxFailedAttempts,disTh,map)
% rrtExtend Ϊ���������һ���ڵ�
% ���� extendFail ��Ϊ��־��0��ʾ����չ��ɻ����ҵ������յ�

pathFound = []; 
extendFail = 1;
failedAttempts = 0;
Index = RRTreeA(1,3);

while failedAttempts <= maxFailedAttempts
    %% ************ ��չRRTree1�ķ�ʽ ******************
    if Index == 0 
        % ��ȡ���������
        if rand < 0.2
            sample = rand(1,2) .* size(map);
        else
            sample = qgoal;  % sample taken as goal to bias tree generation to goal
        end
        % �ҵ�������������������Ľڵ�
        [~, I] = min(distanceCost(RRTreeA(:,1:2),sample), [], 1); 
        closestNode = RRTreeA(I(1),:);
        % ��qnearest��qrand��չһ�ξ��뵽��xnew����������ײ���
        theta = atan2((sample(1)-closestNode(1)),(sample(2)-closestNode(2)));  % direction to extend sample to produce new node
        newPoint = double(int32(closestNode(1:2) + stepsize * [sin(theta)  cos(theta)])); % �½ڵ�
        if ~checkPath(closestNode(1:2), newPoint, map) % if extension of closest node in tree to the new point is not feasible
            failedAttempts = failedAttempts + 1;
            continue;
        end
        % ����½ڵ�����һ�����Ľڵ��Ƿ�������ֵ���������˵������ֱ�����ϣ�����һ����ײ���
        [~, I2] = min(distanceCost(RRTreeB(:,1:2),newPoint), [], 1); 
        if distanceCost(RRTreeB(I2(1),1:2),newPoint) < disTh*disTh         
            % ����������������߶�֮���Ƿ�����ϰ���
            if  checkPath(RRTreeB(I2(1),1:2), newPoint, map) % if is feasible, path found 
                pathFound = [newPoint I I2];
                extendFail = 0;
                RRTree = RRTreeA;
                break;
            else
                failedAttempts = failedAttempts + 1;
                continue;
            end
        end
        % ����½ڵ��Ƿ��Ѿ������д���
        [~, I3] = min(distanceCost(RRTreeA(:,1:2),newPoint), [], 1); 
        if distanceCost(newPoint,RRTreeA(I3(1),1:2)) < disTh*disTh 
            failedAttempts = failedAttempts + 1;
            continue; 
        end
        RRTree = [RRTreeA;newPoint I];
        extendFail = 0;
    else 
    %% ************ ��չRRTree2�ķ�ʽ ******************
        % ��ȡ���������
        if rand < 0.2
            sample = rand(1,2) .* size(map);
        else
            sample = qgoal;  % sample taken as goal to bias tree generation to goal
        end
        % �ҵ�������������������Ľڵ�
        [~, I] = min(distanceCost(RRTreeA(:,1:2),sample), [], 1);% find the minimum value of each column
        closestNode = RRTreeA(I(1),:);
        % ��qnearest��qrand��չһ�ξ��뵽��xnew����������ײ���
        theta = atan2((sample(1)-closestNode(1)),(sample(2)-closestNode(2)));  % direction to extend sample to produce new node
        newPoint = double(int32(closestNode(1:2) + stepsize * [sin(theta)  cos(theta)]));
        if ~checkPath(closestNode(1:2), newPoint, map) % if extension of closest node in tree to the new point is not feasible
            failedAttempts = failedAttempts + 1;
            continue;
        end
        % ����½ڵ�����һ�����Ľڵ��Ƿ�������ֵ���������˵������ֱ�����ϣ�����һ����ײ���
        [~, I2] = min(distanceCost(RRTreeB(:,1:2),newPoint), [], 1); 
        if distanceCost(RRTreeB(I2,1:2),newPoint) < disTh*disTh
            % ����������������߶�֮���Ƿ�����ϰ���
            if  checkPath(RRTreeB(I2(1),1:2), newPoint, map) % if is feasible, path found 
                pathFound = [newPoint I I2];
                extendFail = 0;
                RRTree = RRTreeA;
                break;
            else
                failedAttempts = failedAttempts + 1;
                continue;
            end
        end

        % ����½ڵ�����һ�����Ľڵ㲻������ֵ�������ż�����ͬ��������һ�ξ���
        RRTreeA = [RRTreeA;newPoint I]; % �ȴ�������
        line([RRTreeA(end,2);RRTreeA(RRTreeA(end,3),2)],[RRTreeA(end,1);RRTreeA(RRTreeA(end,3),1)],'color','r','linewidth',3);
        flag = 1;
        while flag == 1
            Temp = newPoint;
            newPoint = double(int32(Temp(1:2) + stepsize * [sin(theta)  cos(theta)]));
            I = size(RRTreeA,1);
            % ����½ڵ�����һ�����Ľڵ��Ƿ�������ֵ
            [~, I2] = min(distanceCost(RRTreeB(:,1:2),newPoint), [], 1); 
            if distanceCost(RRTreeB(I2,1:2),newPoint) < disTh*disTh  
                % ����������������߶�֮���Ƿ�����ϰ���
                if  checkPath(RRTreeB(I2,1:2), newPoint, map)
                    pathFound= [newPoint I I2];
                    extendFail = 0;
                    RRTreeA = [RRTreeA;newPoint I];
                    RRTree = RRTreeA;
                    line([RRTree(end,2);RRTree(RRTree(end,3),2)],[RRTree(end,1);RRTree(RRTree(end,3),1)],'color','r','linewidth',3);
                    break;
                else
                    break;
                end
            else
                if  ~checkPath(Temp, newPoint, map)  % �������ײ
                    RRTree = RRTreeA;
                    extendFail = 0;
                    break;
                else
                     RRTree = [RRTreeA;newPoint I];
                     extendFail = 0;
                     line([RRTree(end,2);RRTree(RRTree(end,3),2)],[RRTree(end,1);RRTree(RRTree(end,3),1)],'color','r','linewidth',3);
                end  
            end
        end   
    end
    break;
end

