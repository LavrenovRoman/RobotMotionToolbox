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

function [X_Total_points,Y_Total_points, ...
    All_cells_Number, Cell_start, X1] = rmt_obstacle_draw(handle_axes,Num_Object,...
    limits,wheelbase,epsilonvoronoi,obstacles)

%Specify filename here to save
%FILE_NAME = 'Obstacle_config';

%Specify Epsilon which is distance b/w consecutive point obstacles
%generated by dividing the objects into number of points larger the value
%of Epsilon smoother the output will be but more computation time it will
%take. Decent value of Epsilon is 1
Epsilon = epsilonvoronoi;

Nxi = limits(1);
Nx = limits(2);
Nyi = limits(3);
Ny = limits(4);
dilation = ceil((wheelbase/2));
env_bounds=[Nxi,Nx,Nyi,Ny];
dilation = 0;

X_Total_points=0;
Y_Total_points=0;
All_cells_Number=0;
Cell_start=1;

%objects = obstacles;
Num_Object = Num_Object+1;

%eps = 0.0001

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
        objects{i}=obstacles{i-1}';       
    end%if
end%for

for i=1:Num_Object
   
    X1{i} = objects{i}';
    X2{i} = circshift(X1{i},1);    %shifting to calculate rms edge length

    %new november 2015
    [ax1 bx1] = size(X1{i})
    if(ax1 < bx1)
        X1{i} = X1{i}';
        X2{i} = X2{i}';
    end


    l = sqrt(sum(((X2{i}-X1{i}).*(X2{i}-X1{i}))'));   %rms edge length
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


%disp('The environment has been re-built (Voronoi).');

end %function
