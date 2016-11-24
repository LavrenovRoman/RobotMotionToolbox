function [ paths ] = rmt_iterations_with_creterias (data, Nobstacles, X1, criterii_length, criterii_curve )

    for h=1:length(data.Homotopies)
        
        Draw(data);
        [cLen, cCurv] = rmt_calcCriterias(data.Vertex_Cord_DV, data.DVPaths{h,1}, Nobstacles, X1);
        
        changePoints = [];
        pb=1;
        for i=1:length(data.DVVerts{h,1})
            for j=pb:length(data.DVPaths{h,1})
                if data.DVPaths{h,1}(1,j) == data.DVVerts{h,1}(1,i)
                    changePoints = [changePoints j];
                    break;
                end;
                pb = pb+1;
            end;
        end;
        if 2 ~= changePoints(1,1)
            changePoints = [2 changePoints];
        end;
        if length(data.DVPaths{h,1})-1 ~= changePoints(1,length(changePoints))
            changePoints = [changePoints length(data.DVPaths{h,1})-1];
        end;
        
        stopIteration = 0;
        while stopIteration == 0
            
            
            stopIteration = 1;
        end;
    end;

end

function [cLen, cCurv] = rmt_calcCriterias(Vertex_Cord_DV, path, Nobstacles, X1)
    % maxLength
    cLen = 0;
    for i=1:length(path)-1
        x=[Vertex_Cord_DV(path(i),1) Vertex_Cord_DV(path(i+1),1)];
        y=[Vertex_Cord_DV(path(i),2) Vertex_Cord_DV(path(i+1),2)];
        cLen = cLen + sqrt((x(1,2)-x(1,1))^2 + (y(1,2)-y(1,1))^2);
    end;
    
    % maxCurve
    cCurv = 180;
    for i=1:length(path)-2
        x=[Vertex_Cord_DV(path(i),1) Vertex_Cord_DV(path(i+1),1) Vertex_Cord_DV(path(i+2),1)];
        y=[Vertex_Cord_DV(path(i),2) Vertex_Cord_DV(path(i+1),2) Vertex_Cord_DV(path(i+2),2)];
        x1 = x(1,1)-x(1,2);
        x2 = x(1,3)-x(1,2);
        y1 = y(1,1)-y(1,2);
        y2 = y(1,3)-y(1,2);
        cosF = (x1*x2 + y1*y2)/(sqrt(x1^2 + y1^2)*sqrt(x2^2 + y2^2));
        angle = acosd(cosF);
        if abs(angle) < cCurv
            cCurv = abs(angle);
        end;
    end;
end

function [] = Draw(data)
    cla(data.handle_env);%new
    axes(data.handle_env);
    rmt_plot_environment(data.obstacles,data.frame_limits);                
    plot(data.initial(1),data.initial(2),'pw','Markersize',13, 'Color', 'k');
    plot(data.final(1),data.final(2),'pw','Markersize',13, 'Color', 'b');
    for i=1:size(data.danger_positions,1)
        plot(data.danger_positions(i, 1),data.danger_positions(i, 2),'xw','Markersize',13, 'LineWidth', 5, 'Color', 'r');
    end;
    for i=1:length(data.Edges(1,:))
        plot([data.Edges(1,i) data.Edges(2,i)],[data.Edges(3,i) data.Edges(4,i)],'color',[.8 .8 .8]);
    end
    for i=1:length(data.Verts(:,1))
        if i<5
            plot(data.Verts(i,1),data.Verts(i,2),'*','color','Green','LineWidth',2);
        else
            plot(data.Verts(i,1),data.Verts(i,2),'*','color','Red','LineWidth',2);
        end;
    end;
end

  