function [ homotopies ] = rmt_create_dv_homotopies(Vertex_Cord_DV, PathWithoutCurve, CostWithoutCurve, Nobstacles, X1)
%RMT_CREATE_HOMOTOPIES Summary of this function goes here
%   Detailed explanation goes here

    homotopies = zeros(length(PathWithoutCurve), 1);
    AllVertexesInVDs = cell(length(Vertex_Cord_DV), 1);
    for i=1:length(PathWithoutCurve)
        for j=1:length(PathWithoutCurve{i,1})
            AllVertexesInVDs{PathWithoutCurve{i,1}(1,j), 1} = [AllVertexesInVDs{PathWithoutCurve{i,1}(1,j), 1}, i];
        end;
        homotopies(i,1) = 1;
    end;
    
    for i=1:length(PathWithoutCurve)-1
        if homotopies(i,1) == 0
            continue;
        end;
        for j=i+1:length(PathWithoutCurve)
            if homotopies(j,1) == 0
                continue;
            end;
            findIntersect = 0;
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
                    pointI = [Vertex_Cord_DV(PathWithoutCurve{i,1}(1,pi), 1) Vertex_Cord_DV(PathWithoutCurve{i,1}(1,pi), 2)];
                    min_size = 1000000;
                    dist = min_size;
                    pointJ_min = [0 0];
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
                        if min_size > dist
                            min_size = dist;
                            pointJ_min = pointJ;
                        end;
                    end;
                    
                    if dist == min_size 
                        findIntersect = 0;
                        break;
                    end;
                    
                    line = [pointI(1,1) pointI(1,2) pointJ_min(1,1) pointJ_min(1,2)];
                    
                    if (i==2 && j==7)
                        x=[pointI(1,1) pointJ_min(1,1)];
                        y=[pointI(1,2) pointJ_min(1,2)];
                        plot(x,y,'-','color','r','LineWidth',2);
                        drawnow;
                        hold on;
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
                               %if ~(abs(intersection_point(1,1)-line(1,3))<eps && abs(intersection_point(1,2)-line(1,4))<eps)
                                   %if ~isequal(min_edge,line_edge)
                               findIntersect = 1;
                               break;
                                   %end;
                               %end;
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
            if findIntersect == 0
                homotopies(j,1) = 0;
                fprintf('%5.0f Path is equal %5.0f homotopy\n' , j, i);
            end;
        end;
    end;

    %find voronoi diagram and 
    %{
    eps = 0.000001;
    homotopies = zeros(length(PathWithoutCurve), length(PathWithoutPoint));
    for i=1:length(PathWithoutCurve)
        for j=1:length(PathWithoutPoint)
            findIntersect = 0;
            for k=1:length(PathWithoutCurve{i, 1})
                min_size = 1000000;
                for l=1:length(PathWithoutPoint{j,1})-1
                    point = [Vertex_Cord_DV(PathWithoutCurve{i, 1}(1, k), 1) Vertex_Cord_DV(PathWithoutCurve{i, 1}(1, k), 2)];
                    edge = [Vertex_Cord_VG{1, PathWithoutPoint{j, 1}(1, l)}(1, 1) Vertex_Cord_VG{1, PathWithoutPoint{j, 1}(1, l)}(2, 1) Vertex_Cord_VG{1, PathWithoutPoint{j, 1}(1, l+1)}(1, 1) Vertex_Cord_VG{1, PathWithoutPoint{j, 1}(1, l+1)}(2, 1)];
                    [dist pos] = distancePointEdge(point, edge);
                    if min_size > dist
                        min_size = dist;
                        min_p = point;
                        min_pos = pos;
                        min_edge = edge;
                    end;
                end;
                min_point(1,1) = min_edge(1,1) + min_pos*(min_edge(1,3) - min_edge(1,1));
                min_point(1,2) = min_edge(1,2) + min_pos*(min_edge(1,4) - min_edge(1,2));
                line = [min_p(1,1) min_p(1,2) min_point(1,1) min_point(1,2)];
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
                           if ~(abs(intersection_point(1,1)-line(1,3))<eps && abs(intersection_point(1,2)-line(1,4))<eps)
                               if ~isequal(min_edge,line_edge)
                                   findIntersect = 1;
                                   break;
                               end;
                           end
                       end
                    end
                    if findIntersect==1
                        break;
                    end
                end
                if findIntersect==1
                    break;
                end
            end;
            if findIntersect == 0
                homotopies(i,j) = 1;
                break;
            end;
        end;
    end;
    %}
end

