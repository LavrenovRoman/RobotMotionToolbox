function [trajectory, VfGraphs, PathWithoutPoint, CostWithoutPoint] = rmt_shortest_path(input_variables,seq_obstacles, draw_path)
%RTM_CHORTEST_PATH Summary of this function goes here
%   Detailed explanation goes here

%addpath('.\cdd');

disp('Visibility graph algorithm is running...');

Nxi = input_variables(1);
Nx = input_variables(2);
Nyi = input_variables(3);
Ny = input_variables(4);
Nobstacles = input_variables(5);
%Initial and goal points
xini = input_variables(6);
yini = input_variables(7);
xgoal = input_variables(8);%X-AXIS (0..20)
ygoal = input_variables(9);%Y-AXIS (0..10)
env_bounds=[Nxi,Nx,Nyi,Ny];

%uiwait(msgbox(sprintf('\nFor defining a region:\n\t - left-click = pick a vertex\n\t - right-click = pick last vertex\n\nRegions should be convex and non-overlapping\n'),...
%    'Robot Motion Toolbox','modal'));

%axes(handle);
%axis(env_bounds);
%hold on
%grid on


%reading the evolution domain
x_max = Nx;
y_max = Ny;

reg = seq_obstacles;
poly2 = seq_obstacles;
[aa ba] = size(reg{1});

for i=1:Nobstacles
    %h(i)=fill(reg{i}(1,:),reg{i}(2,:),'c');
    if(aa>ba)
        aux = reg{i}';
        aux_poly = expandPolygon(aux',0.0);
        a = aux_poly{1};
        poly2(i) = {a'};
    else
        aux = reg{i};
        aux_poly = expandPolygon(aux',0.0);
        a = aux_poly{1};
        poly2(i) = {a'};
    end 
    drawPolygon(aux_poly);%,'g','LineWidth',2);    
end

reg = poly2;


%fprintf('Give start and goal point...\n');
x_start = xini;
y_start = yini;
%[x_start,y_start]=ginput(1);
%plot(x_start, y_start, 'pw','Markersize',13, 'Color', 'g');
%[x_goal,y_goal]=ginput(1);
x_goal = xgoal;
y_goal = ygoal;
%plot(x_goal, y_goal,'pw','Markersize',13, 'Color', 'r');

no_of_edges=0;
%find the trajectories from start point to the vertices of the regions
start_point= [x_start,y_start];
n = Nobstacles;
for i=1:n
    %for each region
    l = length(reg{i});
    for j = 1:l
        %for each vertice of that region
        point2=reg{i}(:,j)';% point[x y]
        [exist_flag dist intersction_point] = intersection( start_point, point2 , reg{i});
        if(dist < eps*1e10)
            %exista intersectie doar cu varful regiunii
                %dreapta start-point -> intersectio-point nu trebuie sa intersecteze oricare alta
                %regiune
                flag_inters_regiuni=0; % nu exista intersectii
                point2= intersction_point';
                for k=1:n
                    if(i~=k)%celelalte regiuni
                        [ef d ip] = intersection( start_point, point2 , reg{k});
                        if(ef == 1)
                             flag_inters_regiuni = 1;
                        end
                    end
                end
                if(flag_inters_regiuni ~= 1) 
                    no_of_edges= no_of_edges + 1;
                    edges{no_of_edges}= [start_point' point2'];
                end
           
        end
    end
end

%find the trajectories from stop point to the vertices of the regions
stop_point= [x_goal,y_goal];
for i=1:n
    %for each region
    l = length(reg{i});
    for j = 1:l
        %for each vertice of that region
        point2=reg{i}(:,j)';% point[x y]
        [exist_flag dist intersction_point] = intersection( stop_point, point2 , reg{i});
        if(dist < eps*1e10)
            %exista intersectie doar cu varful regiunii
                %dreapta stop-point -> intersectio-point nu trebuie sa intersecteze oricare alta
                %regiune
                flag_inters_regiuni=0; % nu exista intersectii
                point2= intersction_point';
                for k=1:n
                    if(i~=k)%celelalte regiuni
                        [ef d ip] = intersection( stop_point, point2 , reg{k});
                        if(ef == 1)
                             flag_inters_regiuni = 1;
                        end
                    end
                end
                if(flag_inters_regiuni ~= 1) 
                    no_of_edges= no_of_edges + 1;
                    edges{no_of_edges}= [stop_point' point2'];
                end
           
        end
    end
end


% edges for every region
for i=1:n
    l = length(reg{i});
    for j = 1:l-1
        no_of_edges = no_of_edges + 1;
        edges{no_of_edges}= [reg{i}(:,j) reg{i}(:,j+1)];
    end
    no_of_edges = no_of_edges + 1;
    edges{no_of_edges}= [reg{i}(:,l) reg{i}(:,1)];
end

%supporting  edge
if (n>1)
for i=1:(n-1)
    for j=(i+1):n
        if (i~=j)
            
            temp_edge = supporting_edges(reg{i}, reg{j});
            
            %-----------------------------------------
           
            for nr_temp_edge=1:length(temp_edge)
                point1 = temp_edge{nr_temp_edge}(:,1);
                point2 = temp_edge{nr_temp_edge}(:,2);
                flag_inters_regiuni=0; % nu exista intersectii
                for k=1:n
                    if((i~=k) && (j~=k))%celelalte regiuni
                        [ef d ip] = intersection(point1', point2' , reg{k});
                        if(ef == 1)
                             flag_inters_regiuni = 1;
                        end
                    end
                end 
                if (flag_inters_regiuni ~= 1)
                    edges = [edges temp_edge(nr_temp_edge)];
                end
            end
               
                
            %-----------------------------------------
        end
    end
end
end

%sepparating  edge
if (n>1)
for i=1:(n-1)
    for j=(i+1):n
        if (i~=j)
            temp_edge = sepparating_edges(reg{i}, reg{j});
             %-----------------------------------------
           
            for nr_temp_edge=1:length(temp_edge)
                point1 = temp_edge{nr_temp_edge}(:,1);
                point2 = temp_edge{nr_temp_edge}(:,2);
                flag_inters_regiuni=0; % nu exista intersectii
                for k=1:n
                    if((i~=k) && (j~=k))%celelalte regiuni
                        [ef d ip] = intersection(point1', point2' , reg{k});
                        if(ef == 1)
                             flag_inters_regiuni = 1;
                        end
                    end
                end 
                if (flag_inters_regiuni ~= 1)
                    edges = [edges temp_edge(nr_temp_edge)];
                end
            end
               
                
            %----------------------------------------- 
        end
    end
end
end

%from start to goal
flag_inters_regiuni=0; % nu exista intersectii
for i=1:n
     [ef d ip] = intersection([x_start y_start], [x_goal y_goal] , reg{i});
     if(ef == 1)
         flag_inters_regiuni = 1;
     end

end
if (flag_inters_regiuni ~= 1) % exista drum intre start si stop
    start_goal{1}=[[x_start,y_start]'  [x_goal,y_goal]'];
    edges =[ edges  start_goal{1}];
end

%for i=1:length(edges)
%    plot(edges{i}(1,:), edges{i}(2,:), 'Color',[.8 .8 .8]);
%end


%---------------------------------------------------------------
%    26.05.2013
% --------------------------------------------------------------

pointsInObstacles = zeros(Nobstacles,1);
nrElem=1;
VfGraphs{nrElem}= edges{1}(:,1);
for i=2: length(edges)
    flag=0;
    for j=1:length(VfGraphs)
        if(VfGraphs{j} == edges{i}(:,1))
            flag=1;
            break;
        end
    end
    if(flag == 0)
        nrElem = nrElem+1;
        VfGraphs{nrElem} = edges{i}(:,1);
        findObst = 0;
        for o=1:Nobstacles
            for v=1:size(reg{o},2)
                if VfGraphs{nrElem}(1,1)==reg{o}(1,v) && VfGraphs{nrElem}(2,1)==reg{o}(2,v)
                    pointsInObstacles(nrElem,1) = o;
                    findObst = 1;
                    break;
                end;
            end;
            if findObst == 1;
                break;
            end;
        end;
    end;
end;

%for i=1: length(edges)
%    flag=0;
%    for j=1:length(VfGraphs)
%        if(VfGraphs{j} == edges{i}(:,2))
%            flag=1;
%            break;
%        end
%    end
%    if(flag == 0)
%        nrElem = nrElem+1;
%        VfGraphs{nrElem} = edges{i}(:,2);
%    end
%end


%for i=1:length(VfGraphs)
%plot(VfGraphs{i}(1,:), VfGraphs{i}(2,:), '-yo')
%end

%In vectorul VfGraphs se afla coordonatele tuturor varfurilor din graf.

%determin index start
start_p=start_point';
stop_p=stop_point';
for i=1:length(VfGraphs)
    if(VfGraphs{i}==start_p)
        indexStart = i;
    end
    if(VfGraphs{i}==stop_p)
        indexStop = i;
    end
end

% cost = matricea de adiacenta

len = length (VfGraphs);
PathWithoutPoint = cell(len-2, 1);
CostWithoutPoint = [];
number = 1;

cost = zeros(len);
for i=1:length(edges)
    index_1 = returnIndex(edges{i}(:,1), VfGraphs);
    index_2 = returnIndex(edges{i}(:,2), VfGraphs);
    if(index_1~=0 && index_2~=0)
        cost(index_1, index_2) = norm(edges{i}(:,1)-edges{i}(:,2));
        cost(index_2, index_1) = cost(index_1, index_2); 
    end;
end;
[path, cost_path] = find_path(cost,indexStart,indexStop);
if ~isinf(cost_path)
    PathWithoutPoint{number,1} = path;
    CostWithoutPoint = [CostWithoutPoint, cost_path];
    number = number + 1;
end;

tempCombine1 = zeros(1,length(path)-2);
fprintf('Path with points %5.0f', path(1));
for i=2:length(path)-1
    fprintf(' %5.0f', path(i));
    tempCombine1(1, i-1) = path(i);
end;
fprintf(' %5.0f\n', path(length(path)));
tempCombine1 = sort(tempCombine1);
combineCurves = cell(len, len);
for i=1:length(tempCombine1)
    combineCurves{1,i} = tempCombine1(i);
end;

points_delete = 3:1:len;
for combinations = 1:3
    Comb = nchoosek(points_delete, combinations);
    count_added = 0;
    for c=1:size(Comb, 1)

        if combinations==1 && isempty(find(Comb(c,:)==tempCombine1, 1))
            continue;
        end;
        if combinations>1
            find_comb = 0;
            for j=1:size(combineCurves(combinations-1,:), 2)
                combine=combineCurves{combinations-1,j};
                if isempty(combine)
                    break;
                end;
                comb = 0;
                for k=1:length(combine)
                    if isempty(find(Comb(c,:)==combine(k), 1))
                        break;
                    end;
                    comb = comb+1;
                end;
                if comb==combinations-1
                    find_comb = 1;
                    break;
                end;
            end;
            if find_comb==0
                continue;
            end;
        end;
        
        cost = zeros(len);
        for i=1:length(edges)
            %parcurg fiecare edge
            index_1 = returnIndex(edges{i}(:,1), VfGraphs);
            index_2 = returnIndex(edges{i}(:,2), VfGraphs);
            if(index_1~=0 && index_2~=0 && isempty(find(Comb(c,:)==index_1, 1)) && isempty(find(Comb(c,:)==index_2, 1)))
                cost(index_1, index_2) = norm(edges{i}(:,1)-edges{i}(:,2));
                cost(index_2, index_1) = cost(index_1, index_2); 
            end
        end

        %clc
        [path, cost_path] = find_path(cost,indexStart,indexStop);

        findpath = 0;
        for i=1:length(CostWithoutPoint)
            if cost_path==CostWithoutPoint(i) || isinf(cost_path);
                findpath = 1;
                break;
            end
        end
        if (findpath==0)
            PathWithoutPoint{number,1} = path;
            CostWithoutPoint = [CostWithoutPoint, cost_path];
            number = number + 1;
            for i=1:combinations
                fprintf(' %5.0f', Comb(c,i));
            end;
            fprintf(' Added with cost %3.5f\n', cost_path);
            count_added = count_added + 1;
            if combinations>1
                combineCurves{combinations,count_added} = Comb(c,:);
            end;
        end;
    end;
end;

for i=number:len-2
    PathWithoutPoint(number) = [];
end;

[Dummy MinCost]=min(CostWithoutPoint);
path = PathWithoutPoint{MinCost,1};

disp('VfGraphs:');
VfGraphs{path};
%figure(1)
%figure;hold on;
%hold on
% for i=1:length(path)-1
%     p1 = [VfGraphs{path(i)}(1,:) VfGraphs{path(i)}(2,:)];
%     p2 = [VfGraphs{path(i+1)}(1,:) VfGraphs{path(i+1)}(2,:)];
%     drawEdge(p1,p2,'g','LineWidth', 2);    
% %plot(VfGraphs{i}(1,:),VfGraphs{i}(2,:), '*y')
% end


lon = length(path);
trajectory = zeros(lon,2);
trajectory(1,1) = VfGraphs{path(1)}(1,:);
trajectory(1,2) = VfGraphs{path(1)}(2,:);
for(i=1:lon-1)
    p1 = [VfGraphs{path(i)}(1,:) VfGraphs{path(i)}(2,:)];
    p2 = [VfGraphs{path(i+1)}(1,:) VfGraphs{path(i+1)}(2,:)];    
    if (draw_path==1)
        drawEdge(p1,p2,'r','LineWidth', 2);       
    end
    trajectory(i,1) = VfGraphs{path(i)}(1,:);
    trajectory(i,2) = VfGraphs{path(i)}(2,:);
end
trajectory(lon,1) = VfGraphs{path(lon)}(1,:);
trajectory(lon,2) = VfGraphs{path(lon)}(2,:);

disp('V-graph ends.');
