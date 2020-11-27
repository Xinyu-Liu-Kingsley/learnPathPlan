function path = findMinimumPath(tree,end_node,dim)
% »ØËİ×î¼ÑÂ·¾¶

% find nodes that connect to end_node
connectingNodes = [];
for i = 1:size(tree,1)
    if tree(i,dim+1) == 1
        connectingNodes = [connectingNodes ; tree(i,:)];
    end
end

if size(connectingNodes, 1) > 0
    % find minimum cost last node
    [tmp,idx] = min(connectingNodes(:,dim+2));

    % construct lowest cost path
    path = [connectingNodes(idx,:); end_node];
    parent_node = connectingNodes(idx, dim+3);
    while parent_node > 1
        parent_node = tree(parent_node,dim+3);
        path = [tree(parent_node,:); path];
    end
else
    path = [];
end

end
