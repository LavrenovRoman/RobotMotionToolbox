function [ homotopies ] = rmt_create_homotopies(Vertex_Cord_DV, PathWithoutCurve, Vertex_Cord_VG, PathWithoutPoint, Nobstacles, X1)
%RMT_CREATE_HOMOTOPIES Summary of this function goes here
%   Detailed explanation goes here

    %find voronoi diagram and 
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
                if (min_p(1,1)==min_point(1,1)&&(min_p(1,2)==min_point(1,2))) 
                    continue;
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
end

