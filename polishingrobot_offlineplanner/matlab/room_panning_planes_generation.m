function [renovation_plane_edge_cell,renovation_plane_norm_vector,renovation_plane_triangle_edge_cell]=room_panning_planes_generation(room_plane_norm_vector,room_plane_edge_cell,room_plane_triangle_edge_cell,panning_distance)

for i=1:1:size(room_plane_norm_vector,2)
    a=room_plane_norm_vector{i}(1,1);
    b=room_plane_norm_vector{i}(1,2);
    c=room_plane_norm_vector{i}(1,3);
        
    for j=1:1:size(room_plane_edge_cell{i},1)
        for k=1:1:2
            renovation_plane_edge_cell{i}(j,3*k-2)=room_plane_edge_cell{i}(j,3*k-2)-a/sqrt(a^2+b^2+c^2)*panning_distance;
            renovation_plane_edge_cell{i}(j,3*k-1)=room_plane_edge_cell{i}(j,3*k-1)-b/sqrt(a^2+b^2+c^2)*panning_distance;
            renovation_plane_edge_cell{i}(j,3*k)=room_plane_edge_cell{i}(j,3*k)-c/sqrt(a^2+b^2+c^2)*panning_distance;
        end
    end
    renovation_plane_norm_vector{i}(1,1)=a;
    renovation_plane_norm_vector{i}(1,2)=b;
    renovation_plane_norm_vector{i}(1,3)=c;
    
    for j=1:1:size(room_plane_triangle_edge_cell{i},1)
        for k=1:1:2
            renovation_plane_triangle_edge_cell{i}(j,3*k-2)=room_plane_triangle_edge_cell{i}(j,3*k-2)-a/sqrt(a^2+b^2+c^2)*panning_distance;
            renovation_plane_triangle_edge_cell{i}(j,3*k-1)=room_plane_triangle_edge_cell{i}(j,3*k-1)-b/sqrt(a^2+b^2+c^2)*panning_distance;
            renovation_plane_triangle_edge_cell{i}(j,3*k)=room_plane_triangle_edge_cell{i}(j,3*k)-c/sqrt(a^2+b^2+c^2)*panning_distance;
        end
    end
end







end


