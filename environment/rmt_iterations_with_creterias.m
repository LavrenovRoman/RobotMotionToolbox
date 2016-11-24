function [ paths ] = rmt_iterations_with_creterias (data, Nobstacles, X1, criterias)

    cRand = 10;
    dlt = 0.1;

    for h=1:length(data.Homotopies)
        
        nP = data.Homotopies(h, 1);
        path = zeros(length(data.DVPaths{nP,1}), 2);
        for i=1:length(data.DVPaths{nP,1})
            path(i,1) = data.Vertex_Cord_DV(data.DVPaths{nP,1}(1,i),1);
            path(i,2) = data.Vertex_Cord_DV(data.DVPaths{nP,1}(1,i),2);
        end;
        Draw(data, path);        
        changePoints = [];
        changePointsDlt = [];
        pb=1;
        for i=1:length(data.DVVerts{nP,1})
            for j=pb:length(data.DVPaths{nP,1})
                if data.DVPaths{nP,1}(1,j) == data.DVVerts{nP,1}(1,i)
                    changePoints = [changePoints j];
                    break;
                end;
                pb = pb+1;
            end;
        end;
        if 2 ~= changePoints(1,1)
            changePoints = [2 changePoints];
        end;
        if length(data.DVPaths{nP,1})-1 ~= changePoints(1,length(changePoints))
            changePoints = [changePoints length(data.DVPaths{nP,1})-1];
        end;
        
        [cLen, cCurv] = rmt_calcCriterias(path, Nobstacles, X1);
        tempPath = path;
        for i=1:length(changePoints)
            results = zeros(cRand, 2);
            deltas  = zeros(cRand, 2);
            resultsC = zeros(cRand, 2);
            for j=1:cRand
                xr = rand(1) - 0.5;
                yr = rand(1) - 0.5;
                x = dlt*xr/sqrt(xr^2 + yr^2);
                y = dlt*yr/sqrt(xr^2 + yr^2);
                deltas(j, 1) = x;
                deltas(j, 2) = y;
                tempPath(changePoints(i), 1) = path(changePoints(i), 1) + x;
                tempPath(changePoints(i), 2) = path(changePoints(i), 2) + y;
                [cLenN, cCurvN] = rmt_calcCriterias(tempPath, Nobstacles, X1);
                [bLenN, bCurvN] = checkCriterias(cLen, cCurv, cLenN, cCurvN);
                results(j, 1) = bLenN;
                results(j, 2) = bCurvN;
                resultsC(j, 1) = cLenN;
                resultsC(j, 2) = cCurvN;
                Draw(data, tempPath);
            end;
            CurBestResult = setModifyPath(path, tempPath, ...
                deltas, results, resultsC, criterias); 
        end;
        
        stopIteration = 0;
        while stopIteration == 0
            
            stopIteration = 1;
        end;
    end;

end

function [CurBestResult] = setModifyPath(path, tempPath, deltas, results, resultsC, criterias)
    for i=1:length(results(:, 1))
        results(i,1) = results(i,1)*criterias(1,1);
        results(i,2) = results(i,2)*criterias(1,2);
        
    end
    
end

function [bLen, bCurv] = checkCriterias(cLen, cCurv, cLenN, cCurvN)
    bLen = 0;
    bCurv = 0;
    if cLen>cLenN
        bLen = 1;
    end;
    if cCurv>=cCurvN
        bCurv = 1;
    end;
end

function [cLen, cCurv] = rmt_calcCriterias(path, Nobstacles, X1)
    % maxLength
    cLen = 0;
    for i=1:length(path)-1
        x=[path(i,1) path(i+1,1)];
        y=[path(i,2) path(i+1,2)];
        cLen = cLen + sqrt((x(1,2)-x(1,1))^2 + (y(1,2)-y(1,1))^2);
    end;
    
    % maxCurve
    cCurv = 180;
    for i=1:length(path)-2
        x=[path(i,1) path(i+1,1) path(i+2,1)];
        y=[path(i,2) path(i+1,2) path(i+2,2)];
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

function [] = Draw(data, path)
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
    end;
    for i=1:length(data.Verts(:,1))
        if i<5
            plot(data.Verts(i,1),data.Verts(i,2),'*','color','Green','LineWidth',2);
        else
            plot(data.Verts(i,1),data.Verts(i,2),'*','color','Red','LineWidth',2);
        end;
    end;
    for i=1:length(path)-1
        x=[path(i,1) path(i+1,1)];
        y=[path(i,2) path(i+1,2)];
        plot(x,y,'-','color','r','LineWidth',2);
    end;
    drawnow;
    hold on;
end

  