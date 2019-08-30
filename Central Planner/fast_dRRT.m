function [Tree , decoupled_paths] = fast_dRRT(Qi,Qf,G,H,valid_indices)

    connected = false;
    
    Tree(1) = Node(Qi,ones(size(Qi,1),1));
    
    V_last = 1;
    
    %figure;hold on;
    goal_idx = zeros(size(Qf,1),1);
    
    for i = 1:size(Qf,1)
        goal_idx(i) = Query_State(G{i},Qf(i,:));
    end
    
    Q_goal = Node(Qf,goal_idx);
    
    while not(connected)
        
        [V_last,Tree] = Expand(Tree,V_last,G,valid_indices,H,Q_goal);
        goal_idx = Query_Tree(Tree,Q_goal);
        
        connected = goal_idx > 0;
    end
    
    decoupled_paths = ReturnPaths(Tree,G,goal_idx);

end

%% Expansion Functions
function [V_last,Tree] = Expand(Tree,V_last,G,valid_indices,H,Q_goal)
    

    % If the last expansion node is empty, randomly sample the roadmaps
    if isempty(V_last)
        Q_rand = Random_Sample(G,valid_indices,Q_goal);
        nearIdx = Nearest_Neighbor(Tree,Q_rand,H);
    else
        Q_rand = Q_goal;
        nearIdx = V_last;
    end
    
    % Determine New node to expand from from direction oracle
    Vnew = Informed_Expansion(Tree(nearIdx),Q_rand,G,H,Q_goal);
    
    N = Neighbors_In_Tree(Vnew,G,Tree,Tree(nearIdx));

    % Find the best collision-free parent node to expand from
    [lowest_cost_idx , best_free_idx , lowest_cost , lowest_free_cost] = Best_Expansion_Parent(Vnew,N,Tree,G);
    
    if isempty(best_free_idx)
        
        V_hybrid = Brute_Force_Connector(Tree(lowest_cost_idx),Vnew,G);
        
        in_tree = Query_Tree(Tree,V_hybrid);
        
        if (Collision_Free_Dynamic(Tree(lowest_cost_idx) , V_hybrid , G)) && ~in_tree 
            
            [Tree , hybrid_idx] = Insert(Tree,V_hybrid);
            Tree = Connect(Tree, lowest_cost_idx, hybrid_idx, lowest_cost);
            %Plot_Paths(Tree(lowest_cost_idx),V_hybrid,G);
            %pause(0.001);
            V_last = hybrid_idx;
            return;
        else
            
            V_last = [];
            return;
        end
        
        
    else
		[Tree , new_idx] = Insert(Tree,Vnew);
		Tree = Connect(Tree, best_free_idx, new_idx, lowest_free_cost);
        %Plot_Paths(Tree(best_free_idx),Vnew,G);
        %pause(0.001);
		
		V_last = new_idx;
    end
end

% Returns a random configuration
function Q_rand = Random_Sample(G,valid_indices,Q_goal)

    persistent count;

    if(isempty(count))
        count  = 0;
    end

    % 2% bias towards goal configuration   
    if(count == 50)
        
        Q_rand = Q_goal;
        count = 0;
    
    else
    
        state = zeros(length(G),4);
        individual_rm_idx = zeros(length(G),1);
    
        for i = 1:length(G)
            idx = randi(length(valid_indices{i}));
            j = valid_indices{i}(idx);
            state(i,:) = G{i}(j).state;
            individual_rm_idx(i) = j;
        end
        
        Q_rand = Node(state,individual_rm_idx);
        
    end
	
	count = count + 1;

    
end


function nearestIdx = Nearest_Neighbor(Tree,Q_rand,H)

    R = length(H); % Number of robots
    

    nearestIdx = 0;

    min_distance = inf;
    for i = 1:length(Tree)
        
        d_cost = zeros(1,R);
		
        for j = 1:R
            d_cost(j) = norm(Q_rand.state-Tree(i).state);
        end
		
        current_distance = sum(d_cost);
        
        if(current_distance < min_distance)
            min_distance = current_distance;
            nearestIdx = i;
        end
        
    end
end

function Vnew = Informed_Expansion(Vnear,Q_rand,G,H,Q_goal)

    R = size(Q_rand.state,1);
    new_state = zeros(R,4);
    indices = zeros(R,1);
    for i = 1:R
        
        % Access the neighbors of v_near in the individual roadmap i
        neighbors = [Vnear.rm_idx(i), G{i}(Vnear.rm_idx(i)).next]; 
       
        % If the randomly sampled goal state is the target state, select
        % the individual node with the shortest pre-computed path.
        % Otherwise, randomly select a neighbor
        
        
        
        if isequal(Q_rand.rm_idx,Q_goal.rm_idx)
            
            % Pick the neighbor with the lowest pre-computed path cost
            [~,bestIdx] = min(H{i}(neighbors,Q_goal.rm_idx(i)));
            expansion_neighbor = neighbors(bestIdx);
            
        else
            
            % Pick a neighbor randomly
            expansion_neighbor = neighbors(randi(length(neighbors)));
            
        end
        
        new_state(i,:) = G{i}(expansion_neighbor).state;
        indices(i) = expansion_neighbor;
    end
    
    Vnew = Node(new_state,indices);

end

function N = Neighbors_In_Tree(V_new,G,Tree,V_near)
    R = length(G);
    N = []; % List of Parent Nodes in Tree


    parents = cell(R,1);

    for i = 1:R
        parents{i} = [V_near.rm_idx(i) , G{i}(V_new.rm_idx(i)).prev];
    end

    for i = 1:length(Tree)
    
        isParent = zeros(1,R);
    
        for j = 1:R
            if ismember(Tree(i).rm_idx(j),parents{j})
                isParent(j) = 1;
            end
        end
    
        if(all(isParent))
            N = [N, i];
        end
    end
end

function [lowest_free_cost,best_idx,Vnew] = FindBestExpansionParent(Vnew,N,Tree,G)

    cost = zeros(1,length(N));
    free = zeros(1,length(N));
	
    for i = 1:length(N)
        if Collision_Free_Dynamic(Tree(N(i)),Vnew,G)
			free(i) = 1;
        end
		
		cost(i) = Tree(N(i)).ctc + Cost_to_Go(Tree(N(i)),Vnew,G);
    end
    
    free_neighbors = find(free == 1);
    
    %if isempty(free_neighbors)
    %    [lowest_free_cost,lowest_free_idx] = min(cost);
    %    Vnew = Brute_Force_Connector(Vnew,Tree(N(best_idx)));
        %lowest_free_idx = 0;
        %lowest_free_cost = Inf;
		
    %else
    [lowest_free_cost,lowest_free_idx] = min(cost(free_neighbors));
    %end
    best_idx = N(lowest_free_idx);
    
end

% Returns the best free expansion candidate (if one exists) and the
% expansion candidate with the lowest cost
function [lowest_cost_idx , best_free_idx , lowest_cost , lowest_free_cost] = Best_Expansion_Parent(Vnew,N,Tree,G)

    cost = zeros(1,length(N));
    free = zeros(1,length(N));
    
    
    for i = 1:length(N)
        
        cost(i) = Cost_to_Go(Tree(N(i)),Vnew,G);
        
        if Collision_Free_Dynamic(Tree(N(i)),Vnew,G)
            free(i) = 1;
        else
            free(i) = Inf;
        end
    end
    
    free_neighbors = free .* cost;
    
    [lowest_free_cost,min_free_idx] = min(free_neighbors);
    [lowest_cost,min_index] = min(cost);
    lowest_cost_idx = N(min_index);
    
    if lowest_free_cost == Inf
        best_free_idx = [];
    else
        best_free_idx = N(min_free_idx);
    end
end

function ctg = Cost_to_Go(V1,V2,G)

    R = size(V1.state,1);
    
    ctg = 0;
    
    for i = 1:R
        
        idx1 = V1.rm_idx(i);
        idx2 = V2.rm_idx(i);
        
        if(idx1 == idx2)
            continue;
        else
            cost_idx = find(G{i}(idx1).next == idx2);
            ctg = ctg + G{i}(idx1).ctg(cost_idx);
        end
    end

end
%% Search / Path reconstruction functions

function idx = Query_Tree(Tree,Q)

    idx = 0;
    
    for i = 1:length(Tree)
        
        if(isequal(Tree(i).rm_idx,Q.rm_idx))
            idx = i;
            return
        end
        
    end

end
%% Plotting functions

% Plots paths between composite vertices
function [] = Plot_Paths(V1,V2,G)

    colors = ["r" , "g" , "b" ];
    paths = Get_Paths(V1,V2,G);
    
    for i = 1:length(paths)
        
        plot(paths{i}(1,:),paths{i}(2,:),'Color',colors(mod(i,3) + 1));
        
    end

end
%% Collision checking functions

% Returns boolean indicating if the individual nodes of V collide with each
% other
function free = Collision_Free_Static(V)

    vehicle_body = [-1.7 , -1.7 , 1.7 , 1.7;
                     0.9 , -0.9 , -0.9, 0.9];
   
    num_agents = size(V.state,1);
    
    for i = 1:(num_agents - 1)
       
        Xi_extended = Convert_to_Ref_Frame(V.state(i,:)',vehicle_body);
        vehicle_i_poly = polyshape(Xi_extended(1,:),Xi_extended(2,:));
        
        for j = (i+1):num_agents
            
            Xj_extended = Convert_to_Ref_Frame(V.state(j,:)',vehicle_body);
            vehicle_j_poly = polyshape(Xj_extended(1,:),Xj_extended(2,:));
            
            vehicle_ovlp = intersect(vehicle_i_poly,vehicle_j_poly);
            
            if ~isempty(vehicle_ovlp.Vertices)
                free = false;
                return
            end
            
        end
        
    end

end

% Checks if collision result
function free = Collision_Free_Dynamic(V1,V2,G)
    
    free = true;

    inflated_paths = Get_Inflated_Paths(V1,V2,G);
    
    for i = 1:length(inflated_paths) - 1
        
        path_i = inflated_paths{i};
        
        for j = (i+1):length(inflated_paths)
            
            path_j = inflated_paths{j};
            
            if overlaps(path_i,path_j)
                
                free = false;
                return
				
            end
            
        end
    end
end

function inflated_paths = Get_Inflated_Paths(V1,V2,G)

    vehicle_body = [-1.7 , -1.7 , 1.7 , 1.7;
                     0.9 , -0.9 , -0.9, 0.9];

    for i = 1:size(V1.state,1)
        
        idx_1 = V1.rm_idx(i);
        idx_2 = V2.rm_idx(i);
        
        if idx_1 == idx_2
            % If the two indices are the same, then the vehicle does not
            % move. If this is the case, construct a polygon centered
            % around the state V1
            state = V1.state(i,:)';
            X_extent = Convert_to_Ref_Frame(state,vehicle_body);
            inflated_paths{i} = polyshape(X_extent(1,:),X_extent(2,:));
            
        else
            
            poly_idx = find(G{i}(idx_1).next == idx_2);
            
            inflated_paths{i} = G{i}(idx_1).inflated_path{poly_idx};
            
        end
        
    end

end

% Returns individual paths between nodes
function composite_paths = Get_Paths(V1,V2,G)

    R = size(V1.state,1);
    composite_paths = cell(R,1);

    for i = 1:size(V1.state,1)
        
        idx_1 = V1.rm_idx(i);
        idx_2 = V2.rm_idx(i);
        
        if idx_1 == idx_2
            % If the two indices are the same, then the vehicle does not
            % move. If this is the case, construct a polygon centered
            % around the state V1
            composite_paths{i} = [V1.state(i,:)',V2.state(i,:)'];
            
        else
            
            path_idx = find(G{i}(idx_1).next == idx_2);
            
            composite_paths{i} = G{i}(idx_1).path{path_idx};
            
        end
        
    end

end

function vehicle_poly = Get_State_Polygons(V)
    vehicle_body = [-1.7 , -1.7 , 1.7 , 1.7;
                     0.9 , -0.9 , -0.9, 0.9];
    vehicle_poly = cell(12,1);
    
    for i = 1:size(V.state,1)
        
        X_extended = Convert_to_Ref_Frame( V.state(i,:)' , vehicle_body );
        
        vehicle_poly{i} = polyshape(X_extended(1,:),X_extended(2,:));
        
    end

end

function V_hybrid = Brute_Force_Connector(V1,V2,G)

    R = length(G);
	
    % Three sets to consider for each i-th vehicle : Interacting vehicles with
    % higher priority than i (Hi) , vehicles with lower priority (Lo), and
    % vehicles with ambiguous priority , but one or the other must move
    % (Xor)
    Hi = cell(R,1);
    Lo = cell(R,1);
    Xor = cell(R,1);
    
	vehicle_paths = Get_Inflated_Paths(V1,V2,G);
    Xi = Get_State_Polygons(V1);
    
    for i = 1:(R-1)
	
        for j = (i+1):R
            
            local_priority = Priority_i_j(Xi{i},vehicle_paths{i},Xi{j},vehicle_paths{j});
            
            switch local_priority
                case 1
                    Lo{i} = [Lo{i} , j];
                    Hi{j} = [Hi{j} , i];
                case -1
                    Hi{i} = [Hi{i} , j];
                    Lo{i} = [Lo{j} , i];
                case 2
                    Xor{i} = [Xor{i} , j];
                    Xor{j} = [Xor{j} , i];
            end
            
        end
    end
    
    % Start with an empty set
    S = [];
    not_S = [];
    
    Q_hybrid = zeros(R,4);
    indices = zeros(R,1);
    for i = 1:R
        
        if (isempty(Hi{i})) && (isempty(Xor{i}))
            
            % Case 1: No nodes with higher priority and no ambiguous
            % priorities
            Q_hybrid(i,:) = V2.state(i,:);
            indices(i) = V2.rm_idx(i);
            S = [S , i];
        
        elseif ~isempty(Hi{i})
            
            % Case 2: Nodes that clearly have a higher priority. In this
            % case, do not add i to S and remove any ambigous priorities
            % between i and other nodes
            Q_hybrid(i,:) = V1.state(i,:);
            indices(i) = V1.rm_idx(i);
            not_S = [not_S , i];
            
            for j = 1:length(Xor{i})
               neighbor = Xor{i}(j);
               idx = Xor{neighbor} == i;
               Xor{neighbor}(idx) = [];
            end
            
        elseif isempty(Hi{i}) && ~isempty(Xor{i})
            
            % If there are no vehicles with clear priority over i but
            % some vehicles with ambiguous priority with i, assess the impact of giving priority to
            % i vs. not giving priority to i
            
            % Cost of not choosing i is defined as the number of nodes in
            % the low set of i, i.e., the number 
            cost_i = length(Xor{i});
            
            % The cost is defined as the number of nodes that would be
            % disqualified from S if the ith node was added to S
            cost = zeros(1,length(Xor{i}));
            
            for j = 1:length(cost)
                
                cost(j) = length(Xor{Xor{i}(j)});
                
            end
            
            min_cost = min(cost);
            
            if min_cost < cost_i
                
                Q_hybrid(i,:) = V1.state(i,:);
                indices(i) = V1.rm_idx(i);
                for j = 1:length(Xor{i})
                    neighbor = Xor{i}(j);
                    idx = Xor{neighbor} == i;
                    
                    if not(isempty(idx))
                    
                        Xor{neighbor , idx} = [];
                        
                    end
                end
                
            else
                Q_hybrid(i,:) = V2.state(i,:);
                indices(i) = V2.rm_idx(i);
                S = [S , i];
            end
            
        end
    end
    
    V_hybrid = Node(Q_hybrid,indices);

end

% Returns the priority of vehicle i with respect to vehicle j
function priority = Priority_i_j(Vi_1,path_i,Vj_1,path_j)
    
    path_i_and_path_j_overlap = overlaps(path_i,path_j);
    
    if not(path_i_and_path_j_overlap)
        priority = 0;
        return
    end
    
    Vi_1_and_path_j_overlap = overlaps(Vi_1,path_j);
    path_i_and_Vj_1_overlap = overlaps(path_i,Vj_1);
    
    if Vi_1_and_path_j_overlap
        priority = 1;
    elseif path_i_and_Vj_1_overlap
        priority = -1;
    else
        priority = 2;
    end
    
end

%% Tree management functions
function node = Node(Q,local_idx)

    node = struct('state',Q,'prev',[],'ctc',0,'rm_idx',local_idx);
    
end

function [Tree,new_idx] = Insert(Tree,V)
	Tree = [Tree , V];
	new_idx = length(Tree);
end

function Tree = Connect(Tree,idx1,idx2,cost)

	Tree(idx2).prev = idx1;
	Tree(idx2).ctc = cost;

end

% Returns the decoupled paths from starting index to current indx
function decoupled_path = ReturnPaths(Tree,G,idx)

    sequence = idx;
    current = idx;
    
    N = length(G);
    
    % Back trace the path from idx to starting node
    while Tree(current).prev ~= 0
       
        sequence = [Tree(current).prev , sequence];
        current = Tree(current).prev;
        
    end
    
    T = 0;
    
    decoupled_path = cell(N,1);
    
    for i = 1:length(sequence)
        
        if i == length(sequence)
            
            composite_paths = cell(12,1);
            
        else
            
            composite_paths = Get_Paths(Tree(sequence(i)) , Tree(sequence(i + 1)), G);
            
        end

        for j = 1:N
           
            if i == length(sequence)
                
                path_length = 0;
                
            else 
                idx1 = Tree(sequence(i)).rm_idx(j);
                idx2 = Tree(sequence(i+1)).rm_idx(j);
                path_length_idx = find(G{j}(idx1).next == idx2);
                path_length = G{j}(idx1).ctg(path_length_idx);
            end
            
            wp = struct('state',[ Tree(sequence(i)).state(j,:) , T],'ds',path_length,'ref_path',composite_paths{j});
            decoupled_path{j} = [decoupled_path{j} , wp];
            
        end
        
        T = T + 1;
    end

end
%% Miscellaneous functions
function q = Convert_to_Ref_Frame(qref,q)

    theta = qref(3);
    R = [cos(theta) -sin(theta);
     sin(theta) cos(theta)];

    % Case 1: q is an entire configuration 
    if(size(q,1) == 4)
 
        for i = 1:size(q,2)
            dq = q(1:2,i);
            q(1:2,i) = R*dq + qref(1:2);
            q(3,i) = mod(q(3,i) + theta,2*pi);
        end
    % Case 2: q is a set of euclidean points
    else
        for i = 1:size(q,2)
            dq = q(1:2,i);
            q(1:2,i) = R*dq + qref(1:2);
        end
    end
end