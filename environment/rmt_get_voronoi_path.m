
function [coord_pathDV] = rmt_get_voronoi_path(Vertex_Cord_DV, CostDV, pathDV, initial, final, eps)

    path_size = 2*CostDV/eps;

    coord_pathDV = zeros(ceil(path_size), 2);
    coord_pathDV(1,:) = [initial(1) initial(2)];
    number = 1;
    for i=1:length(pathDV)
        dist = norm([Vertex_Cord_DV(pathDV(i),1)-coord_pathDV(number,1) Vertex_Cord_DV(pathDV(i),2)-coord_pathDV(number,2)]);
        dict(1,:) = [(Vertex_Cord_DV(pathDV(i),1)-coord_pathDV(number,1))/dist (Vertex_Cord_DV(pathDV(i),2)-coord_pathDV(number,2))/dist];
        while (dist>eps)
            number = number + 1;
            coord_pathDV(number, :) = [eps*dict(1,1)+coord_pathDV(number-1,1) eps*dict(1,2)+coord_pathDV(number-1,2)];
            dist = norm([Vertex_Cord_DV(pathDV(i),1)-coord_pathDV(number,1) Vertex_Cord_DV(pathDV(i),2)-coord_pathDV(number,2)]);
        end
        number = number + 1;
        coord_pathDV(number, :) = [Vertex_Cord_DV(pathDV(i),1) Vertex_Cord_DV(pathDV(i),2)];
    end
    dist = norm([final(1)-coord_pathDV(number,1) final(2)-coord_pathDV(number,2)]);
    dict(1,:) = [(final(1)-coord_pathDV(number,1))/dist (final(2)-coord_pathDV(number,2))/dist];
    while (dist>eps)
        number = number + 1;
        coord_pathDV(number, :) = [eps*dict(1,1)+coord_pathDV(number-1,1) eps*dict(1,2)+coord_pathDV(number-1,2)];
        dist = norm([final(1)-coord_pathDV(number,1) final(2)-coord_pathDV(number,2)]);
    end
    number = number + 1;
    coord_pathDV(number,:) = [final(1) final(2)];
    number = number + 1;
    for i=number:size(coord_pathDV,1)
        coord_pathDV(number,:) = [];
    end;

end%function


