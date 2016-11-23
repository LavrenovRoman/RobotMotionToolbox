function [ homotopies ] = rmt_create_dv_homotopies(Vertex_Cord_DV, PathWithoutCurve, CostWithoutCurve, VertWithoutCurve, Nobstacles, X1)
%RMT_CREATE_HOMOTOPIES Summary of this function goes here
%   Detailed explanation goes here

    homotopies = zeros(length(PathWithoutCurve), 1);
    AllVertexesInVDs = cell(length(Vertex_Cord_DV), 1);
    for i=1:length(PathWithoutCurve)
        for j=1:length(PathWithoutCurve{i,1})
            AllVertexesInVDs{PathWithoutCurve{i,1}(1,j), 1} = [AllVertexesInVDs{PathWithoutCurve{i,1}(1,j), 1}, i];
        end;
        homotopies(i,1) = i;
    end;
    
    for i=1:length(PathWithoutCurve)-1
        if homotopies(i,1) ~= i
            continue;
        end;
        for j=i+1:length(PathWithoutCurve)
            if homotopies(j,1) ~= j
                continue;
            end;
            
            %if i==1 && j==8
            %    pointJ_max = [0 0];
            %end;
            
            findIntersect = 0;
            findh2 = 0;
            for pi=1:length(PathWithoutCurve{i,1})
                h2 = 0;
                for have2=1:length(AllVertexesInVDs{PathWithoutCurve{i,1}(1,pi), 1})
                    if (AllVertexesInVDs{PathWithoutCurve{i,1}(1,pi), 1}(1,have2) == j)
                        h2 = 1;
                        break;
                    end;
                    if h2==1
                        break;
                    end;
                end;
                if (h2 == 0)     
                    findh2 = 1;
                    pointI = [Vertex_Cord_DV(PathWithoutCurve{i,1}(1,pi), 1) Vertex_Cord_DV(PathWithoutCurve{i,1}(1,pi), 2)];
                    max_size = 0;
                    dist = max_size;
                    pointJ_max = [0 0];                    
                    for pj=1:length(PathWithoutCurve{j,1})
                        h1 = 0;
                        for have1=1:length(AllVertexesInVDs{PathWithoutCurve{j,1}(1,pj), 1})
                            if (AllVertexesInVDs{PathWithoutCurve{j,1}(1,pj), 1}(1,have1) == i)
                                h1 = 1;
                                break;
                            end;
                            if h1==1
                                break;
                            end;
                        end;
                        if h1 == 1
                            continue;
                        end;
                        pointJ = [Vertex_Cord_DV(PathWithoutCurve{j,1}(1,pj), 1) Vertex_Cord_DV(PathWithoutCurve{j,1}(1,pj), 2)];
                        dist = sqrt((pointI(1,1)-pointJ(1,1))^2 + (pointI(1,2)-pointJ(1,2))^2);
                        if max_size < dist
                            max_size = dist;
                            pointJ_max = pointJ;
                        end;
                    end;
                    
                    if max_size == 0
                        findIntersect = 0;
                        break;
                    end;
                    
                    %if dist == min_size 
                    %    findIntersect = 0;
                    %    break;
                    %end;
                    
                    line = [pointI(1,1) pointI(1,2) pointJ_max(1,1) pointJ_max(1,2)];
                    
                    if i==8 && j==13
                        x=[line(1,1) line(1,1)];
                        y=[line(1,2) line(1,2)];
                        plot(x,y,'-','color','g','LineWidth',3);
                    end;
                    
                    for l=2:Nobstacles
                        for r=1:length(X1{l})
                           a=r;
                           if(r==length(X1{l}))
                               b=1;
                           else
                               b=r+1;
                           end
                           line_edge = [X1{l}(a,1) X1{l}(a,2) X1{l}(b,1) X1{l}(b,2)];
                           intersection_point = intersectEdges(line, line_edge);
                           if ~isnan(intersection_point(1,1)) || ~isnan(intersection_point(1,2))
                               findIntersect = 1;
                               break;
                           end;
                        end;
                        if findIntersect==1
                            break;
                        end
                    end
                    if findIntersect==1
                        break;
                    end;
                end;
            end;
            if (findh2 == 0 && (length(VertWithoutCurve{i,1}) ~= length(VertWithoutCurve{j,1}))) %length(PathWithoutCurve{i,1}) < length(PathWithoutCurve{j,1}))
                findIntersect = 1;
            end;
            if findIntersect == 0
                homotopies(j,1) = i;
                fprintf('%5.0f Path is equal %5.0f homotopy\n' , j, i);
            end;
        end;
    end;
    
    %p = 3;
    %for i=1:length(PathWithoutCurve{p,1})-1
    % x=[Vertex_Cord_DV(PathWithoutCurve{p,1}(1,i), 1) Vertex_Cord_DV(PathWithoutCurve{p,1}(1,i+1), 1)];
    % y=[Vertex_Cord_DV(PathWithoutCurve{p,1}(1,i), 2) Vertex_Cord_DV(PathWithoutCurve{p,1}(1,i+1), 2)];
    % plot(x,y,'-','color','r','LineWidth',2);
    % drawnow;
    % hold on;
    %end
    
    for i=1:length(homotopies)
        for j=1:length(homotopies)
            if homotopies(j,1) == i
                if CostWithoutCurve(j) > CostWithoutCurve(i)
                    for k=1:length(homotopies)
                        if homotopies(k,1) == i
                            homotopies(k,1) = j;
                        end;
                    end;
                end;
            end;
        end;
    end;
end

