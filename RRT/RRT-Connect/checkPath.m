function feasible = checkPath(n,newPos,map) 
% checkPath ���extendһ�ξ�����Ƿ����ϰ�����ײ
% ����Ϊqnearst��qextend

%floor������ȡ��
%ceil������ȡ��
feasible = 1;
dir = atan2(newPos(1) - n(1),newPos(2) - n(2));
for r = 0:0.5:sqrt(sum((n - newPos).^2))
    posCheck = n + r.*[sin(dir),cos(dir)];
    if ~(feasiblePoint(ceil(posCheck),map) && feasiblePoint(floor(posCheck),map)&& ...
        feasiblePoint([ceil(posCheck(1)) floor(posCheck(2))],map) && feasiblePoint([floor(posCheck(1)) ceil(posCheck(2))],map))
        feasible = 0;
        break;
    end
    if ~feasiblePoint(newPos,map)
        feasible = 0;
    end
end

end
