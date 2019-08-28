function cost = BreadthFirstSearch(G,xi,xj)
% Searches for the existance of a path from i to j in graph G
%
% Inputs:
%
% i - initial index
% j - goal index
% G - Graph structure

    connected = 0;
    queue = xi;
    discovered = zeros(1,length(G));
    discovered(xi) = 1;
    
    parent = zeros(1,length(G));
    
    while(~isempty(queue))
        
        v = queue(end);
        queue(end) = [];
        
        if(v == xj)
            connected = 1;
            break;
        end
        
        neighboringNodes = G(v).next;
        
        for i = 1:length(neighboringNodes)
            
            if(~discovered(neighboringNodes(i)))
                
                discovered(neighboringNodes(i)) = 1;
                queue = [neighboringNodes(i) queue];
                parent(neighboringNodes(i)) = v;
                
            end
            
        end
        
    end
    
    if(connected)
        cost = BackTracePath(parent,xi,xj,G);
    else
        cost = Inf;
    end
end

function cost = BackTracePath(parent,xi,xf,G)
    current = xf;
    
    cost = 0;
    while(current ~= xi)
        cost = cost + G(parent(current)).ctg(find(G(parent(current)).next==current));
        current = parent(current);
    end
end
