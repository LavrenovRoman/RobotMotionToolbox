%    This is part of RMTool - Robot Motion Toolbox, for Matlab 2010b or newer.
%
%    Copyright (C) 2016 RMTool developing team. For people, details and citing 
%    information, please see: http://webdiis.unizar.es/RMTool/index.html.
%
%    This program is free software: you can redistribute it and/or modify
%    it under the terms of the GNU General Public License as published by
%    the Free Software Foundation, either version 3 of the License, or
%    (at your option) any later version.
%
%    This program is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%    GNU General Public License for more details.
%
%    You should have received a copy of the GNU General Public License
%    along with this program.  If not, see <http://www.gnu.org/licenses/>.

%% ============================================================================
%   MOBILE ROBOT TOOLBOX
%   Graphical User Interface
%   First version released on September, 2014. 
%   Last modification December 29, 2015.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function [Start, Goal, X_Total_points,Y_Total_points, ...
    All_cells_Number, Cell_start, X1] = rmt_obstacle_draw(handle_axes,Num_Object,limits,wheelbase,epsilonvoronoi)

%Specify filename here to save
%FILE_NAME = 'Obstacle_config';

%Specify Epsilon which is distance b/w consecutive point obstacles
%generated by dividing the objects into number of points larger the value
%of Epsilon smoother the output will be but more computation time it will
%take. Decent value of Epsilon is 1
Epsilon = epsilonvoronoi;

fprintf('\nBuilding a grid-based map with polytopic obstacles (polygons)\n');

Nxi = limits(1);
Nx = limits(2);
Nyi = limits(3);
Ny = limits(4);
dilation = ceil((wheelbase/2));
env_bounds=[Nxi,Nx,Nyi,Ny];
dilation = 0;

axes(handle_axes);
axis(env_bounds);
hold on
grid on

uiwait(msgbox(sprintf('\nFor defining a region:\n\t - left-click = pick a vertex\n\t - right-click = pick last vertex\n\nRegions should be convex and non-overlapping\n'),...
    'Robot Motion Toolbox','modal'));


%Main code starts here
%handle = figure;
%axis([0 100 0 100]);
%hold on;
%grid;

X_Total_points=0;
Y_Total_points=0;
All_cells_Number=0;
Cell_start=1;

Num_Object = Num_Object+1;

for i=1:Num_Object
    if(i==1)%first we add the limits of the environment
        x = [Nxi+dilation,Nx-dilation,Nxi+dilation,Nx-dilation];
        y = [Nyi+dilation,Nyi+dilation,Ny-dilation,Ny-dilation];            
        for(j = 1:4) %j = no. of vertexes for current object
            objects{i}(:,j)=[x(j), y(j)];
        end
        k=convhull(objects{i}(1,:),objects{i}(2,:));
        objects{i}=objects{i}(:,k(1:length(k)-1));        
    else
        %read obstacle's vertices
        j = 1; %j = no. of vertexes for current object
        but = 1;
        while but==1
            [x,y,but]=ginput(1);
            %x = round(x);
            %y = round(y);
            inside = 0;
            for o=2:length(objects)
                if (inside==1)
                    break;
                end
                if (o==i) 
                    continue;
                end
                T = delaunay(objects{o}(1,:),objects{o}(2,:));
                for t=1:size(T)
                    p1 = [objects{o}(1,T(t,1)) objects{o}(2,T(t,1))];
                    p2 = [objects{o}(1,T(t,2)) objects{o}(2,T(t,2))];
                    p3 = [objects{o}(1,T(t,3)) objects{o}(2,T(t,3))];
                    tri = [p1;p2;p3];
                    if isPointInTriangle([x y], tri)==1
                        inside = 1;
                        break;
                    end
                end
            end
            if (inside==0)
                if ((but==3)&&(j>2))||(but==1)
                    plot(x,y,'.k')
                    objects{i}(:,j)=[x, y];
                    j=j+1;
                else
                    xc = objects{i}(1,1);
                    yc = objects{i}(2,1);
                    r = sqrt((xc-x)^2 + (yc-y)^2);
                    if xc-Nxi<r 
                        r=xc-Nxi;
                    end
                    if Nx-xc<r 
                        r=Nx-xc;
                    end
                    if yc-Nyi<r 
                        r=yc-Nyi;
                    end
                    if Ny-yc<r 
                        r=Ny-yc;
                    end
                    %for o=1:length(objects)-1
                    %    rt = distancePointPolygon(objects{i}(:,1), objects{o});
                    %    if (rt < r)
                    %        r= rt;
                    %    end;
                    %end;
                    j = 1;
                    for a=20:20:360
                        x = xc + r*cos(degtorad(a));
                        y = yc + r*sin(degtorad(a));
                        objects{i}(:,j)=[x, y];
                        j=j+1;
                    end
                end
            end
        end    
        %creating convex obstacles & drawing them
        %k=convhull(objects{i}(1,:),objects{i}(2,:));
        %objects{i}=objects{i}(:,k(1:length(k)-1));
        %pause(0.3)
        fill(objects{i}(1,:),objects{i}(2,:),'k','FaceAlpha',0.5); %or functia patch (similara cu fill)    
    end%if
    
    %check orientation of polygon
    v1 = objects{i}(:,2) - objects{i}(:,1);
    v1(3) = 0;
    v2 = objects{i}(:,2) - objects{i}(:,3);
    v2(3) = 0;
    vc = cross (v1, v2);
    if vc(3)>0
        N = length(objects{i});
        for v=1:N
            objects{i+1}(:,v)=objects{i}(:, N+1-v);
        end
        objects{i} = objects{i+1};
        objects{:,i+1} = [];
    end;

    X1{i} = objects{i}';
    X2{i} = circshift(X1{i},1);    %shifting to calculate rms edge length
    l = sqrt(sum(((X2{i}-X1{i}).*(X2{i}-X1{i}))'));   %rms edge length
    %EpsilonTemp = Epsilon;
    %if EpsilonTemp > min(l)/2
    %    EpsilonTemp = min(l)/2;
    %    if Epsilon > EpsilonTemp
    %        Epsilon = EpsilonTemp
    %    end
    %end
    Num_points = floor(l/Epsilon);

    size_input =  length(X1{i});
    X_points{i}=0;
    Y_points{i}=0;

    for j=1:size_input
        %points on one edge of obstacle 
        if((X2{i}(j,1)-X1{i}(j,1))==0)
            X2{i}(j,1) = X2{i}(j,1)-0.005;
        end
        if((X2{i}(j,2)-X1{i}(j,2))==0)
            X2{i}(j,2) = X2{i}(j,2)-0.005;
        end
        X_edge{i}{j}=X1{i}(j,1):(X2{i}(j,1)-X1{i}(j,1))/Num_points(j):X2{i}(j,1);        
        Y_edge{i}{j}=X1{i}(j,2):(X2{i}(j,2)-X1{i}(j,2))/Num_points(j):X2{i}(j,2);
        plot(X_edge{i}{j},Y_edge{i}{j});
        X_edge{i}{j}(1)=[]; %removing first element to avoid repetetion
        Y_edge{i}{j}(1)=[]; %removing first element to avoid repetetion
        
        %Total points on an obstacle
        X_points{i}=cat(2,X_points{i},X_edge{i}{j});
        Y_points{i}=cat(2,Y_points{i},Y_edge{i}{j});     
    end
   %for r=1:size_input
   %    a=r;
   %    if(r==size_input)
   %        b=1;
   %    else
   %        b=r+1;
   %    end
           
       %plot(X1{i}(a:b,1),X1{i}(a:b,2));
       %hold on;
       %drawnow;
   %end

    X_points{i}(1)=[];
    Y_points{i}(1)=[];
    Cell_Number{i} = i*ones(size(X_points{i}));

    %Total points 
    X_Total_points=cat(2,X_Total_points,X_points{i});
    Y_Total_points=cat(2,Y_Total_points,Y_points{i});
    All_cells_Number = cat(2,All_cells_Number,Cell_Number{i});
    Cell_start(i+1) = length(All_cells_Number)+1;
    %drawnow;
end

X_Total_points(1)=[];
Y_Total_points(1)=[];
All_cells_Number(1)=[];
%saves the figure 
%saveas(handle,strcat(FILE_NAME,'.jpg'));
%Stores the drawn obstacle in file with the same name as figure
%save(strcat(FILE_NAME));

uiwait(msgbox(sprintf('\nChoose the initial and goal points with right click.\n'),'Robot Motion Toolbox','modal'));

for i=1:2   
    j=1; %j = no. of vertexes for current object
    but=1;
    %while but==1
    %    [x,y,but]=ginput(1);
    %    x = round(x);       
    %    y = round(y);       
    %end       
    point_ok = 0;
    while(point_ok == 0)
        but=1;
        while but==1
            [x,y,but]=ginput(1);
            x = round(x);
            y = round(y);
        end
        in = 0;
        for(ii=2:Num_Object)
            in = in + inpolygon(x,y,objects{ii}(1,:),objects{ii}(2,:));
        end 
        if(in>0)
            uiwait(msgbox(sprintf('\nInvalid point!\n'),'Robot Motion Toolbox','modal'));
        else
            point_ok = 1;
        end
    end
    
    if i == 1
        %plot(x,y,'or','LineWidth',3);
        plot(x,y,'pw','Markersize',13, 'Color', 'k');
        Start = [x,y];
    else
        %plot(x,y,'xk','LineWidth',3);
        plot(x,y,'pw','Markersize',13, 'Color', 'b');
        Goal = [x,y];
    end    
end

disp('The environment has been built (Voronoi).');

end %function
