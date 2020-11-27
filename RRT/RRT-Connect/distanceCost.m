function h = distanceCost(a,b)
% distanceCost 输入随机树矩阵，求两点之间的最小距离

h = sum((a-b).^2,2); % 对差值的每一行求和，返回的是距离的平方
    
end
