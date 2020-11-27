function [RRTree1,RRTree2] = Swap(RRTreeA,RRTreeB)
    LA = length(RRTreeA(:,1));
    LB = length(RRTreeB(:,1));
    if  LB < LA
        RRTree1 = RRTreeB; 
        RRTree2 = RRTreeA;
    else
        RRTree1 = RRTreeA; 
        RRTree2 = RRTreeB;
    end
end
