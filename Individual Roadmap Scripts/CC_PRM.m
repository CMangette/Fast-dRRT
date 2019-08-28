function G = CC_PRM(qi,qf,N,r,intention , vehicle_dimensions)

    G(1) = Node([qi,0]);
    G(1).prev = 0;
    
    ref_paths = Sampling_Paths(qi',qf',intention,100);
    figure; hold on;
    scatter(qi(1),qi(2),20,"b",'filled');
    scatter(qf(1),qf(2),20,"b",'filled');
    pause(0.001);
    
    TP = Turning_Param(1, 2);
    while (length(G) < N )
        
        % Sample the configuration space bounds
        q_rand = Random_Sample(qf,ref_paths);
        
        % Check if the random configuration exists in G
        rand_idx = Query_State(G,q_rand);
        
        for i = 1:length(G)
            
            if(Reachable(G(i).state,q_rand,r))
                
                % If reachable, return the path points between the two
                % configurations
                
                path = Steer(G(i).state',q_rand',TP);
                if not(path.is_valid)
                    continue;
                elseif (path.path_param.length > r) || (path.path_param.length >= 1.4 * dist(G(i).state,q_rand))
                    continue;
                else
                    % If not already in the graph, add to graph
                    if(rand_idx == 0)
                    
                        v_new = Node(q_rand);
                        [G,rand_idx] = Insert(G,v_new);
                    
                    end
                    
                    
                    if( ~ismember(rand_idx,G(i).next) )
                        
                        G = Connect(G,i,rand_idx,path , vehicle_dimensions);
                        
                    end
                end
            else
                continue
            end
            
        end
    end

end


%% Graph Management Functions

% Returns standard node structure with the given state
function node = Node(q)
    node = struct('state',q,'prev',[],'next',[],'ctg',[],'path',...
        cell(1),'inflated_path',cell(1));
end

% Inserts new vertex into graph
function [ G , new_idx] = Insert(G,v_new)

    G = [G v_new];
    new_idx = length(G);

end


% Returns G with connection added between indices i and j  
function G = Connect(G,i,j,path , vehicle_dimensions)

    path_length = path.path_param.length;
    G(i).next(end + 1) = j;
    G(i).ctg(end + 1) = path_length;
    
    s = 0:0.1:path_length;
    
    if path.type == STRAIGHT
        
        x = linspace(G(i).state(1) , G(j).state(1) , length(s));
        y = linspace(G(i).state(2) , G(j).state(2) , length(s));
        theta = G(i).state(3) * ones(1,length(s));
        kappa = zeros(1,length(s));
        
        
        path_states = [x ; y ; theta ; kappa ; s];
        
    else
        
        path_states = SCC_Path(path,s);
        path_states = [path_states ; s];
        
    end
    
    inflated_path = Path_Polygon(path,vehicle_dimensions);
    
    G(i).path{end + 1} = path_states;
    G(i).inflated_path{end + 1} = inflated_path;
    
    G(j).prev(end + 1) = i;
    
    plot(path_states(1,:),path_states(2,:),'Color','b');
    pause(0.001);
end

%% PRM Construction Functions

% Returns a random sample. 5% bias towards goal region
function q_rand = Random_Sample(q_goal,sample_paths)

    persistent count;
    
    if(isempty(count))
        count = 0;
    end
    
    if(count == 20)
        q_rand = [q_goal,0];
        count = 0;
    else
    
        smax = sample_paths{1}(4,end);
        x = sample_paths{1}(1,:);
        y = sample_paths{1}(2,:);
        theta = sample_paths{1}(3,:);
        s = sample_paths{1}(4,:); 
        
        s_rand = smax*rand;
        
    
        x_rand = round(interp1(s,x,s_rand),2) + 0.65 * randn;
        y_rand = round(interp1(s,y,s_rand),2)  + 0.65 * randn;
        theta_rand = round(interp1(s,theta,s_rand),2) + 0.05 * randn;
    
        q_rand = [x_rand, y_rand, theta_rand, 0];
    end
    
    count = count + 1;

end

function reachable = Reachable(qi,qf,r)

    dubins_param =  dubins_core(qi,qf,3);
    
    path_length = 3 * sum(dubins_param.seg_param);
    
    exceeds_r = path_length > r;
    
    if(exceeds_r )
        reachable = false;
    else
        reachable = true;
    end

end

% Returns the index at which the state q exists
function idx = Query_State(G,q)
    idx = 0;
    for i = 1:length(G)
        if(Equal_States(G(i).state,q))
            idx = i;
        end
    end
end

% Returns boolean indicating if q1 and q2 are equal states. Using the
% default function isequal() can lead to numerical errors and lead to false
% negatives.
function equal = Equal_States(q1,q2)

    dq = round(norm(q2 - q1),4);
    
    if(dq == 0)
        equal = true;
    else
        equal = false;
    end

end

function d = dist(q1,q2)
    d = norm(q1(1:2) - q2(1:2));
end

%% Macros

function type = STRAIGHT()
    type = 5;
end