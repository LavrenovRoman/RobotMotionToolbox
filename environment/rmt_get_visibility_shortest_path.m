function [coord_pathVG] = rmt_get_visibility_shortest_path(Vertex_Cord_VG, pathVG, costVG, point_counts)
%RMT_GET_VISIBILITY_SHORTEST_PATH Summary of this function goes here
%   Detailed explanation goes here
    step_size_VG = costVG/(point_counts-1);
    coord_pathVG = zeros(ceil(point_counts), 2);
    coord_pathVG(1,:) = [Vertex_Cord_VG{1,pathVG(1)}(1,1) Vertex_Cord_VG{1,pathVG(1)}(2,1)];
    number = 1;
    for i=2:length(pathVG)
        dist = norm([Vertex_Cord_VG{1,pathVG(i)}(1,1)-coord_pathVG(number,1) Vertex_Cord_VG{1,pathVG(i)}(2,1)-coord_pathVG(number,2)]);
        dict(1,:) = [(Vertex_Cord_VG{1,pathVG(i)}(1,1)-coord_pathVG(number,1))/dist (Vertex_Cord_VG{1,pathVG(i)}(2,1)-coord_pathVG(number,2))/dist];
        while (dist>2*step_size_VG)
            number = number + 1;
            coord_pathVG(number, :) = [step_size_VG*dict(1,1)+coord_pathVG(number-1,1) step_size_VG*dict(1,2)+coord_pathVG(number-1,2)];
            dist = norm([Vertex_Cord_VG{1,pathVG(i)}(1,1)-coord_pathVG(number,1) Vertex_Cord_VG{1,pathVG(i)}(2,1)-coord_pathVG(number,2)]);
        end
        number = number + 1;
        coord_pathVG(number, :) = [Vertex_Cord_VG{1,pathVG(i)}(1,1) Vertex_Cord_VG{1,pathVG(i)}(2,1)];
        if (i~=length(pathVG))
            number = number + 1;
            dist = 2*step_size_VG - dist;
            distNext = norm([Vertex_Cord_VG{1,pathVG(i+1)}(1,1)-coord_pathVG(number,1) Vertex_Cord_VG{1,pathVG(i+1)}(2,1)-coord_pathVG(number,2)]);
            dict(1,:) = [(Vertex_Cord_VG{1,pathVG(i+1)}(1,1)-coord_pathVG(number,1))/distNext (Vertex_Cord_VG{1,pathVG(i+1)}(2,1)-coord_pathVG(number,2))/distNext];
            coord_pathVG(number, :) = [dist*dict(1,1)+coord_pathVG(number-1,1) dist*dict(1,2)+coord_pathVG(number-1,2)];
        end;
    end;
    number = number + 1;
    for i=number:size(coord_pathVG,1)
        coord_pathVG(number,:) = [];
    end;
end

