clear all
clc
%Q(2)(b)


source={'A','A','A','B','B','C','C','C','D','D','D','D','E','E','E','F','F','F','G','G','G','H','H','I','I','I','I','J','J','J','J','K','K','K','K','L','L','M'};
target={'B','D','E','A','C','B','D','J','A','C','E','F','A','D','F','D','E','G','F','H','I','G','I','G','H','J','K','C','I','K','L','I','J','L','M','J','K','K'};
weights = [10,10,10,10,1,1,15,21,10,15,1,7,10,1,8,7,8,6,6,3,2,3,4,2,4,1,1,21,1,1,6,1,1,5,6,6,5,6];

%for plotting purpose
G = digraph(source,target,weights);
p=plot(G);
[path1,d] = shortestpath(G,'A','M');
highlight(p,path1,'EdgeColor','g')

%given Euclidean distance to estimate
speed=30; %m/s
heuristics= [10,10,10,10,1,1,15,21,10,15,7,8,10,1,8,7,8,6,6,2,3,3,4,2,4,1,1,21,1,1,6,1,1,5,6,6,5,6]*1/speed;
names=['A','B','C','D','E','F','G','H','I','J','K','L','M'];
[path,cost,heuristic,func,iterations] = astar(source,target,weights,heuristics,'A','M')


function [path,cost,heuristic,func,iterations] = astar(source,target,weights,heuristics,names,startNode,goalNode)

    iterations = table;
    
    % Refactor source, target, names, startNode & goalNode vectors to numbers
    
    % If names argument missing 
    if (nargin<7)
        
        % Fifth argument (i.e. names) is starting Node
        ssNode = names;
        
        % Sixth argument (i.e. startNode) is goal Node
        ggNode = startNode;
        
        [s,t,n,sNode,gNode] = refactor(source,target,weights,ssNode,ggNode);
        
    else
        
        % Sixth argument (i.e. startNode) is starting Node
        ssNode = startNode;
        
        % Seventh argument (i.e. goalNode) is goal Node
        ggNode = goalNode;
        
        [s,t,n,sNode,gNode] = refactor(source,target,weights,names,ssNode,ggNode);        
        
    end
    
    % Get all unique nodes from source and target vectors
    uniqueNodes = getNodes(s,t);
    
    % Priority queue
    queue = [];
    
    % Initial path from starting node
    path = struct('Path',sNode,'Cost',0,'Heuristic',heuristics(sNode),'F',heuristics(sNode));
    
    % Add initial path to priority queue
    queue = [queue path];
    
    
    % Local variables to track iteration number
    iteration = 1; 
    
    % Update Iterations table
    iterations = [iterations; updateTable(s,t,n,queue,iteration)];
    
    
    
    % Repeat until queue is empty or goal is reached
    while(isGoalReached(queue,gNode)==0 && length(queue)>0)        
 
        % Put empty table row
        array = {'_____','_________________________________','_____','_____','_____'};
        iterations = [iterations; cell2table(array)];
    
    
        % Get and remove minimum path from priority queue
        [minI,minP] = minPath(queue);
        queue(minI) = [];
        
        %Generate new paths
        newPaths = getNewPaths(s,t,weights,heuristics,minP);
        
        queue = [queue newPaths]; 
        
        % Update Iterations table
        iteration = iteration + 1; 
        iterations = [iterations; updateTable(s,t,n,queue,iteration)];
    
         
    end
    
    if(length(queue)>0)
    
        [minI,minP] = minPath(queue);
        path = n(minP.Path);
        cost = minP.Cost;
        heuristic = minP.Heuristic;
        func = minP.F;
        
                
    else
        
        path = [];
        cost = [];
        heuristic = [];
        func = [];
        
    end
    
    iterations.Properties.VariableNames = {'Iteration' 'PriorityQueue' 'Cost' 'Heuristic' 'F'};
        
end
function [minIndex,path] = minPath(paths)
    minIndex = [];
    path = [];
    
    if(length(paths)>0)
        
        minIndex = 1;
        path = paths(minIndex);
    
        if(length(paths)>1)
           
            for i=2:length(paths)
                
                if(paths(i).F < path.F)
                
                    minIndex = i;
                    path = paths(minIndex);
                
                end
                
            end
            
        end
    
    end
end
function isGoal = isGoalReached(paths,goalNode)
    if(length(paths)==0)
        isGoal = 0;
        return;
    end
    
    [minI,minP] = minPath(paths);
    
    if(minP.Path(length(minP.Path)) == goalNode)
        isGoal = 1;
    else
        isGoal = 0;
    end
end
function weight = getWeight(s,t,weights,nodeA,nodeB)
    
    for i=1:length(s)
       
        if(s(i)==nodeA && t(i)==nodeB)
            weight = weights(i);
        end
        
    end
end
function paths = getNewPaths(s,t,w,h,path)
    paths = [];
    
    uniqueNodes = getNodes(s,t);
    
    if(~isempty(path))
        currentNode = path.Path(length(path.Path));
        childs = getChilds(s,t,currentNode);
        for i=1:length(childs)
            % If path is not a loop
            
            if(length(find(path.Path==childs(i)))==0)
                c = path.Cost + getWeight(s,t,w,currentNode,childs(i));
                heur = h(find(uniqueNodes==childs(i)));
                f = c + heur;
                p = struct('Path',[path.Path childs(i)],'Cost',c,'Heuristic',heur,'F',f);
                paths = [paths p];
            end
        end
    end
end
function childs = getChilds(source,target,node)
    
    childs = sort(target(find(source==node)));
    
end
function nodes = getNodes(s,t)
    nodes = unique(horzcat(s,t));
end
function [s,t,n,sn,gn] = refactor(source,target,weights,names,startNode,goalNode)
    % If names argument missing 
    if (nargin<6)
        
        % Fourth argument (i.e. names) is starting node
        sn = names;
        
        % Fifth argument (i.e. startNode) is goal node
        gn = startNode;
    else
        
        % Fifth argument (i.e. startNode) is starting node
        sn = startNode;  
        
        % Sixth argument (i.e. goalNode) is goal node
        gn = goalNode;
        
    end
    
    % Get all unique nodes
    uNodes = unique(horzcat(source,target));
        
        
    
    % If source and target are cell arrays
    if(iscell(source) && iscell(target))
    
        % If names argument missing
        if(nargin<6)
            n = uNodes;
        else
            n = names;
        end
        
        
        % Get unique nodes cell array
        uNodes = unique(horzcat(source,target));
        s = [];
        t = [];
        % Populate source and target with equivalent numeric values
        for i=1:length(source)
            [sFound,sIndex] = ismember(source(i),uNodes);
            [tFound,tIndex] = ismember(target(i),uNodes);
            s = [s sIndex];
            t = [t tIndex];
        end
            
        
        
        
    else
        
        s = source;
        t = target;
        
        % If names argument missing
        if(nargin<6)    
            
            uNodes = unique(horzcat(source,target));
            n = cell(1,length(uNodes));
            
            
            for i=1:length(uNodes)
                n{i} = num2str(uNodes(i));
            end
            
        else
            n = names;
        end
    end
    
    
    
    % If starting node is not a number
    if(~isnumeric(sn))
        sn = find(ismember(n,sn));
        
    end
    
    % If goal node is not a number
    if(~isnumeric(gn))
        gn = find(ismember(n,gn));
        
    end
end
function tableIteration = updateTable(s,t,n,queue,iteration)
    tempTable = table;
    uniqueNodes = getNodes(s,t);
    
    unsortedF = [];
    sortedF = [];
    for i=1:length(queue)
       unsortedF = [unsortedF queue(i).F]; 
    end
    mx = max(unsortedF);
    while(length(sortedF) ~= length(unsortedF))
        [mins,indices] = min(unsortedF);
        for j=1:length(indices)
            unsortedF(indices(j))=mx+1;
        end
        sortedF = [sortedF indices];
    end
    
    
    % Display current queue
    
    for p = 1:length(queue)
       
        path = queue(sortedF(p));
        
        pathStr = '<';
        for i=length(path.Path):-1:1
            if(i==1)
                pathStr = strcat(pathStr,sprintf('%s',char(n(find(uniqueNodes==path.Path(i))))));
            else
                pathStr = strcat(pathStr,sprintf('%s,',char(n(find(uniqueNodes==path.Path(i))))));
            end
        end
        pathStr = strcat(pathStr,sprintf('>'));
        % Display path cost    
        cost = path.Cost;
        
        % Display path heuristic    
        heuristic = path.Heuristic;
        
        % Display evaluation function    
        f = path.F;
        
        array = {num2str(iteration) pathStr num2str(cost) num2str(heuristic) num2str(f)};
        tempTable = [tempTable; cell2table(array)];
    end
    
    tableIteration = tempTable;
end