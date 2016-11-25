function [ paths ] = rmt_iterations_with_creterias (data, Nobstacles, X1, criterias)

    cTry = 10;
    cRand = 10;
    dlt = 0.1;
    count_crits = length(criterias);
    
    for h=1:length(data.Homotopies)
        nP = data.Homotopies(h, 1);
        dlt = data.DVCosts(nP)/(2*length(data.DVPaths{nP,1}));
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
        
        [cCrits] = rmt_calcCriterias(path, data);
        tempPath = path;
        for i=1:length(changePoints)
            deltas  = zeros(cRand, 2);
            results = zeros(cRand, count_crits);
            resultsC = zeros(cRand, count_crits);
            value = 0;
            t = 0;
            while value <= 0 && t<cTry
                for j=1:cRand
                    xr = rand(1) - 0.5;
                    yr = rand(1) - 0.5;
                    x = dlt*xr/sqrt(xr^2 + yr^2);
                    y = dlt*yr/sqrt(xr^2 + yr^2);
                    deltas(j, 1) = x;
                    deltas(j, 2) = y;
                    tempPath(changePoints(i), 1) = path(changePoints(i), 1) + x;
                    tempPath(changePoints(i), 2) = path(changePoints(i), 2) + y;
                    [cCritsN] = rmt_calcCriterias(tempPath, data);
                    [bCritsN] = checkCriterias(cCrits, cCritsN);
                    results(j,:) = bCritsN;
                    resultsC(j, :) = cCritsN;
                    %Draw(data, tempPath);
                end;
                [CurBestResult, value] = setModifyPath(path, tempPath, ...
                    deltas, results, resultsC, criterias); 
                t = t + 1;
            end;
            if (value > 0)
                changePointsDlt(i, :) = [deltas(CurBestResult, 1) deltas(CurBestResult, 2)];
                path(changePoints(i), 1) = path(changePoints(i), 1) + deltas(CurBestResult, 1);
                path(changePoints(i), 2) = path(changePoints(i), 2) + deltas(CurBestResult, 2);
                Draw(data, path);
            end;
        end;
        
        stopIteration = 0;
        while stopIteration == 0
            
            stopIteration = 1;
        end;
    end;

end

function [CurBestResult, Value] = setModifyPath(path, tempPath, deltas, results, resultsC, criterias)
    Value = -10;
    sumRes = zeros(length(results(:, 1)), 1);
    for i=1:length(results(:, 1))
        for j=1:length(criterias(1, :))
            results(i,j) = results(i,j)*criterias(1,j);
            sumRes(i,1) = sumRes(i,1) + results(i,j);
        end;
    end;
    for i=1:length(sumRes)
        if Value<sumRes(i,1)
            Value=sumRes(i,1);
            CurBestResult = i;
        end;
    end;
end

function [bCritsN] = checkCriterias(cCrits, cCritsN)
    bCritsN = zeros(1, length(cCrits));
    
    if cCrits(1,1)>cCritsN(1,1)
        bCritsN(1,1) = 1;
    end;
    if cCrits(1,1)<cCritsN(1,1)
        bCritsN(1,1) = -1;
    end;
    
    if cCrits(1,2)<cCritsN(1,2)
        bCritsN(1,2) = 1;
    end;
    if cCrits(1,2)>cCritsN(1,2)
        bCritsN(1,2) = -1;
    end;
    
    if cCrits(1,3)<cCritsN(1,3)
        bCritsN(1,3) = 1;
    end;
    if cCrits(1,3)<cCritsN(1,3)
        bCritsN(1,3) = -1;
    end;
    
    if cCrits(1,4)<cCritsN(1,4)
        bCritsN(1,4) = 1;
    end;
    if cCrits(1,4)>cCritsN(1,4)
        bCritsN(1,4) = -1;
    end;
    
    if cCrits(1,5)<cCritsN(1,5)
        bCritsN(1,5) = 1;
    end;
    if cCrits(1,5)>cCritsN(1,5)
        bCritsN(1,5) = -1;
    end;
end

function [cCrits] = rmt_calcCriterias(path, data)

    cCrits = zeros(1,5);
    
    % maxLength
    cLen = 0;
    for i=1:length(path)-1
        x=[path(i,1) path(i+1,1)];
        y=[path(i,2) path(i+1,2)];
        cLen = cLen + sqrt((x(1,2)-x(1,1))^2 + (y(1,2)-y(1,1))^2);
    end;
    cCrits(1,1) = cLen;
    
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
    cCrits(1,2) = cCurv;
    
    %max_see_begin
    cSBegin = 0;
    for i=2:length(path)
        line = [path(1,1) path(1,2) path(i,1) path(i,2)];
        [findIntersect, pIntersect] = find_point_intersect(data, line);
        if findIntersect==0
            lineSqrt = sqrt((path(i,1)-path(1,1))^2 + (path(i,2)-path(1,2))^2);
            if lineSqrt>cSBegin
                cSBegin = lineSqrt;
            end;
        end;
        if findIntersect==1
            break;
        end;
    end;
    cCrits(1,3) = cSBegin;
    
    %max_see_end
    cSEnd = 0;
    lp = length(path);
    for i=length(path)-1:-1:1
        line = [path(lp,1) path(lp,2) path(i,1) path(i,2)];
        [findIntersect, pIntersect] = find_point_intersect(data, line);
        if findIntersect==0
            lineSqrt = sqrt((path(i,1)-path(lp,1))^2 + (path(i,2)-path(lp,2))^2);
            if lineSqrt>cSEnd
                cSEnd = lineSqrt;
            end;
        end;
        if findIntersect==1
            break;
        end;
    end;
    cCrits(1,4) = cSEnd;
    
    %min_dist_from_obstacles
    minDist = 1000000;
    for i=2:length(path)-1
        [dist] = dist_from_obstacles(data, path(i,:));
        if dist<minDist
            minDist = dist;
        end;
    end;
    cCrits(1,5) = minDist;
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

function [dist] = dist_from_obstacles(data, point)
    dist = 1000000;
    Num_Object = data.Nobstacles+1;
    for i=2:Num_Object
        for r=1:length(data.X1{i})
           d = sqrt((point(1,1)-data.X1{i}(r,1))^2 + (point(1,2)-data.X1{i}(r,2))^2);
           if (d<dist)
               dist = d;
           end;
        end
    end;
end

function [findIntersect, pIntersect] = find_point_intersect(data, line)
    findIntersect = 0;
    Num_Object = data.Nobstacles+1;
    for i=2:Num_Object
        for r=1:length(data.X1{i})
           a=r;
           if(r==length(data.X1{i}))
               b=1;
           else
               b=r+1;
           end
           line_edge = [data.X1{i}(a,1) data.X1{i}(a,2) data.X1{i}(b,1) data.X1{i}(b,2)];
           pIntersect = intersectEdges(line, line_edge);
           if ~isnan(pIntersect(1,1)) || ~isnan(pIntersect(1,2))
               findIntersect = 1;
               break;
           end
        end
        if findIntersect==1
            break;
        end;
    end;
end

  