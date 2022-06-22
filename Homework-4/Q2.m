clear all
clc
%Q(2)(a)

%Start point A , End point m
mapname=['A' 'B' 'C' 'D' 'E' 'F' 'G' 'H' 'I' 'J' 'K' 'L' 'M']; 

%Matrix for distance with respect to A
                       %'A' 'B' 'C' 'D' 'E' 'F' 'G' 'H' 'I' 'J' 'K' 'L' 'M'
map =                   [0	10	Inf	10	10	Inf	Inf	Inf	Inf	Inf	Inf	Inf	Inf;
                        10	0	1	Inf	Inf	Inf	Inf	Inf	Inf	Inf	Inf	Inf	Inf;
                        Inf	1	0	15	Inf	Inf	Inf	Inf	Inf	21	Inf	Inf	Inf;
                        10	Inf	15	0	1	7	Inf	Inf	Inf	Inf	Inf	Inf	Inf;
                        10	Inf	Inf	1	0	8	Inf	Inf	Inf	Inf	Inf	Inf	Inf;
                        Inf	Inf	Inf	7	8	0	6	Inf	Inf	Inf	Inf	Inf	Inf;
                        Inf	Inf	Inf	Inf	Inf	6	0	3	2	Inf	Inf	Inf	Inf;
                        Inf	Inf	Inf	Inf	Inf	Inf	3	0	4	Inf	Inf	Inf	Inf;
                        Inf	Inf	Inf	Inf	Inf	Inf	2	4	0	1	1	Inf	Inf;
                        Inf	Inf	21	Inf	Inf	Inf	Inf	Inf	1	0	1	6	Inf;
                        Inf	Inf	Inf	Inf	Inf	Inf	Inf	Inf	1	1	0	5	6;
                        Inf	Inf	Inf	Inf	Inf	Inf	Inf	Inf	Inf	6	5	0	Inf;
                        Inf	Inf	Inf	Inf	Inf	Inf	Inf	Inf	Inf	Inf	6	Inf	0
                        ];
%Source id
s=1;
%Destination id
d=size(map,1);
%All paths
k = size(map,1);         


%------Call kShortestPath------:
[shortestPaths, totalCosts] = ShortestPath(map, s, d, k);
%------Display results------:

if isempty(shortestPaths)
    fprintf('No path available between these nodes\n\n');
else
    for i = 1: length(shortestPaths)
        fprintf('Path # %d:\n',i);
        disp(shortestPaths{i})
        fprintf('Cost of path %d is %5.2f\n\n',i,totalCosts(i));
    end
end

index=find(min(totalCosts));
fprintf('Shortest path is:')
display(shortestPaths{index},mapname(shortestPaths{index}))
fprintf('And its cost function:')
display(totalCosts(index))
