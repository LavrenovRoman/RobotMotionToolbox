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


function env_h = rmt_plot_environment(objects,env_bounds,varargin)

%env_h=figure(); %figure handle
%axis(env_bounds); %axis(world_dim);
hold on
set(gca,'Box','on');

if(nargin>2)
for i=1:length(objects)
    fill(objects{i}(1,:),objects{i}(2,:),'g-','FaceAlpha',0.5);
    centr=mean(objects{i},2)';
%     if nargin<3
        %comment by ramon text(centr(1),centr(2),sprintf('O_{%d}',i),'HorizontalAlignment','center','Color','w','FontSize',12,'FontWeight','bold','FontAngle','italic','FontName','TimesNewRoman');
%     end
end
end

switch nargin
    case 2
        %do nothing (obstacles already plotted)
        [a1 a2] = size(objects{1});
        for(i=1:length(objects))
            if(a1>a2)
                fill(objects{i}(:,1),objects{i}(:,2),'g-','FaceAlpha',0.5); 
            else
                fill(objects{i}(1,:),objects{i}(2,:),'g-','FaceAlpha',0.5); 
            end
        end        
    case 3  %argument C (cells) - plot cell decomposition
        C=varargin{1};
        %represent cells:
        for i=1:length(C)
            fill(C{i}(1,:),C{i}(2,:),'w','EdgeColor',[.8 .8 .8]);%,'FaceAlpha',0.5);
        end
        
        %write cell number
        for i=1:length(C)
            centr=mean(C{i},2)';
            %comment by ramon text(centr(1),centr(2),sprintf('c_{%d}',i),'HorizontalAlignment','center','Color','k','FontSize',10,'FontAngle','italic','FontName','TimesNewRoman');
        end
        set(gca,'Box','on');%,'XTick',[],'YTick',[]);
        
    case 4 %arguments C (cells) and adjacency - plot adjacency graph and cell decomposition
        C=varargin{1};
        adj=varargin{2};
        %represent cells:
        for i=1:length(C)
            fill(C{i}(1,:),C{i}(2,:),'w','EdgeColor',[.8 .8 .8]);%,'FaceAlpha',0.5);
        end

        centr=zeros(length(C),2);   %store centroids
        for i=1:length(C)
            centr(i,:)=mean(C{i},2)';
        end
        gplot(adj,centr,':b');  %represent adjacency graph

        %write cell number
        for i=1:length(C)
            %comment by ramon text(centr(i,1),centr(i,2),sprintf('c_{%d}',i),'HorizontalAlignment','center','Color','k','FontSize',10,'FontAngle','italic','FontName','TimesNewRoman');
        end
        set(gca,'Box','on');%,'XTick',[],'YTick',[]);
        
    case 5  %arguments C,adj,middle_X,middle_Y - do not represent adjacency graph
        C=varargin{1};
        adj=varargin{2};
        middle_X=varargin{3};
        middle_Y=varargin{4};
        %represent cells:
        for i=1:length(C)
            fill(C{i}(1,:),C{i}(2,:),'w','EdgeColor',[.8 .8 .8]);%,'FaceAlpha',0.5);
        end

        centr=zeros(length(C),2);   %store centroids
        for i=1:length(C)
            centr(i,:)=mean(C{i},2)';
        end
%         gplot(adj,centr,':b');  %represent adjacency graph

        for i=1:length(C)
            for j=setdiff(find(adj(i,:)),1:i)
                plot(middle_X(i,j),middle_Y(i,j),'*r')
            end
        end
        %write cell number
        for i=1:length(C)
            %comment by ramon text(centr(i,1),centr(i,2),sprintf('c_{%d}',i),'HorizontalAlignment','center','Color','k','FontSize',10,'FontAngle','italic','FontName','TimesNewRoman');
        end
        set(gca,'Box','on');%,'XTick',[],'YTick',[]);
        
    otherwise
        fprintf('\nCheck # of arguments for the plot\_environment function\n')
end
