function [shortestPath, totalCost] = dijkstra(map, s, d)

n = size(map,1);
for i = 1:n
    % initialize the farthest node to be itself;
    farthestPrevHop(i) = i;
    farthestNextHop(i) = i;
end
%Note all nodes as unvisited
visited(1:n) = false;
%initialize the distance to all points as infinity.
distance(1:n) = inf;   
parent(1:n) = 0;
distance(s) = 0;

%Find the unvisited nodes
%Find with smallest values
%Make the current point and its distance the current distance

for i = 1:(n-1),
    temp = [];
    for h = 1:n,
         if ~visited(h)  
             temp=[temp distance(h)];
         else
             temp=[temp inf];
         end
     end;
     [t, u] = min(temp);      
     visited(u) = true;        
     for v = 1:n,               
         if ( ( map(u, v) + distance(u)) < distance(v) )
             distance(v) = distance(u) + map(u, v);   
             parent(v) = u;    
         end;             
     end;
end;
shortestPath = [];
if parent(d) ~= 0   % if there is a shortestPath!
    t = d;
    shortestPath = [d];
    while t ~= s
        p = parent(t);
        shortestPath = [p shortestPath];
        
        if map(t, farthestPrevHop(t)) < map(t, p)
            farthestPrevHop(t) = p;
        end;
        if map(p, farthestNextHop(p)) < map(p, t)
            farthestNextHop(p) = t;
        end;
        t = p;      
    end;
end;
totalCost = distance(d);
%return;