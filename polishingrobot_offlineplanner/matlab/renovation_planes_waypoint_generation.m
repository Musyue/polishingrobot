function [renovation_effective_waypoints,renovation_effective_waypaths,room_plane_boundary,distance_waypoints2wallboundary_direction1,distance_waypoints2wallboundary_direction2]=renovation_planes_waypoint_generation(room_plane_edge_cell,room_plane_norm_vector,room_vertices,room_plane_triangle_cell,interval1,interval2,panning_distance1)

% interval1=waypoints_interval;
% interval2=path_interval;
room_triangle_cell=room_plane_triangle_cell;

% the first step: determine max x y and z
for i=1:1:size(room_vertices,2)
    room_vertices_mat(:,:)=room_vertices{1}(:,:);
    xmax=max(room_vertices_mat(:,1));
    ymax=max(room_vertices_mat(:,2));
    zmax=max(room_vertices_mat(:,3));
end

% the second step: add the function of boundary_cell_generation
for i=1:1:size(room_plane_edge_cell,2)
    plane_boundaries=room_plane_edge_cell{i};
    [boundary_cell]=boundary_cell_generation(plane_boundaries);
    room_plane_boundary_cell{i}=boundary_cell;
end
room_plane_boundary=room_plane_boundary_cell;

% the third step: add the function of boundary_cell_sort
for i=1:1:size(room_plane_edge_cell,2)
    boundary_cell=room_plane_boundary_cell{i};
    [boundary_cell_sorted]=boundary_cell_sort(boundary_cell);
    room_plane_boundary_cell_sorted{i}=boundary_cell_sorted;
end

% the fourth step: obtain the outer boundary of each plane
room_plane_boundary={};
room_plane_boundary=room_plane_boundary_cell_sorted;
for i=1:1:size(room_plane_boundary,2)
    for j=1:1:size(room_plane_boundary{i},2)
        if room_plane_boundary{i}{j}(1,7)==1
            room_plane_outer_boundary{i}(:,1:6)=room_plane_boundary{i}{j}(:,1:6);
        end
    end
end
for i=1:1:size(room_plane_outer_boundary,2)
    m=1;
    for j=1:1:size(room_plane_outer_boundary{i},1)
        room_plane_outer_boundary_point_rep{i}(m,1:3)=room_plane_outer_boundary{i}(j,1:3);
        m=m+1;
        room_plane_outer_boundary_point_rep{i}(m,1:3)=room_plane_outer_boundary{i}(j,4:6);
        m=m+1;
    end
end
for i=1:1:size(room_plane_outer_boundary,2)
    point_rep_mat=room_plane_outer_boundary_point_rep{i};
    room_plane_outer_boundary_point{i}=unique(point_rep_mat(:,1:3),'rows');
end

% the fifth step: obtain intersection points inside one plane 
for i=1:1:size(room_plane_norm_vector,2)
    room_plane_point(i,1:3)=room_plane_boundary{i}{1}(1,1:3);
    room_plane_parameters(i,1:3)=room_plane_norm_vector{i}(1,1:3);
    room_plane_parameters(i,4)=-room_plane_norm_vector{i}(1,1:3)*room_plane_point(i,1:3)';
end
intersect_plane_norm_vector1=zeros(size(room_plane_norm_vector,2),3);
intersect_plane_norm_vector2=zeros(size(room_plane_norm_vector,2),3);
for i=1:1:size(room_plane_norm_vector,2)
    sin_theta=-room_plane_parameters(i,1);
    cos_theta=sqrt(1-sin_theta^2);
    if cos_theta~=0     
        sin_beta=room_plane_parameters(i,2)/cos_theta;
        cos_beta=room_plane_parameters(i,3)/cos_theta;
    else
        sin_beta=0;
        cos_beta=1;
    end
    intersect_plane_norm_vector1(i,1:3)=[cos_theta, sin_theta*sin_beta, sin_theta*cos_beta];
    intersect_plane_norm_vector2(i,1:3)=[0, cos_beta, -sin_beta];
%     if abs(abs(room_plane_parameters(i,1))-1)<=0.01
%         intersect_plane_norm_vector1(i,1:3)=[0, cos_beta, -sin_beta];
%         intersect_plane_norm_vector2(i,1:3)=[cos_theta, sin_theta*sin_beta, sin_theta*cos_beta];
%     end
%     if abs(abs(room_plane_parameters(i,2))-1)<=0.01
%         intersect_plane_norm_vector1(i,1:3)=[cos_theta, sin_theta*sin_beta, sin_theta*cos_beta];
%         intersect_plane_norm_vector2(i,1:3)=[0, cos_beta, -sin_beta];
%     end
end
for i=1:1:size(room_plane_outer_boundary_point,2) 
    for j=1:1:size(room_plane_outer_boundary_point{i},1)
        intersect_plane1_d_candidate{i}(1,j)=-intersect_plane_norm_vector1(i,1:3)*room_plane_outer_boundary_point{i}(j,1:3)';
        intersect_plane2_d_candidate{i}(1,j)=-intersect_plane_norm_vector2(i,1:3)*room_plane_outer_boundary_point{i}(j,1:3)';
    end
end
for i=1:1:size(room_plane_outer_boundary_point,2) 
    for j=1:1:size(room_plane_outer_boundary_point{i},1)
       intersect_plane1_dmin_max{i}(1,1)=min(intersect_plane1_d_candidate{i});
       intersect_plane1_dmin_max{i}(1,2)=max(intersect_plane1_d_candidate{i});
       
       intersect_plane2_dmin_max{i}(1,1)=min(intersect_plane2_d_candidate{i});
       intersect_plane2_dmin_max{i}(1,2)=max(intersect_plane2_d_candidate{i});
    end
end
for i=1:1:size(room_plane_outer_boundary_point,2)
    l1=abs(intersect_plane1_dmin_max{i}(1,2)-intersect_plane1_dmin_max{i}(1,1));
    l2=abs(intersect_plane2_dmin_max{i}(1,2)-intersect_plane2_dmin_max{i}(1,1));
    if l1>=l2
        room_plane_interval{i}(1)=interval1;
        room_plane_interval{i}(2)=interval2;
    else
        room_plane_interval{i}(1)=interval1;
        room_plane_interval{i}(2)=interval2;
    end
end
for i=1:1:size(room_plane_outer_boundary_point,2) 
    dmin=intersect_plane1_dmin_max{i}(1,1);
    dmax=intersect_plane1_dmin_max{i}(1,2);
    interval=room_plane_interval{i}(1);
    plane1_num=floor(abs(dmax-dmin)/interval);
    for j=1:1:plane1_num
        intersect_plane1_d{i}(1,j)=dmin+(j-0.5)*interval;
    end
end

for i=1:1:size(room_plane_outer_boundary_point,2) 
    dmin=intersect_plane2_dmin_max{i}(1,1);
    dmax=intersect_plane2_dmin_max{i}(1,2);
    interval=room_plane_interval{i}(2);
    plane2_num=floor(abs(dmax-dmin)/interval);
    for j=1:1:plane2_num
        intersect_plane2_d{i}(1,j)=dmin+(j-0.5)*interval;
    end
end

for i=1:1:size(room_plane_outer_boundary_point,2)
    distance_waypoints2wallboundary_direction1(i,1)=intersect_plane1_d{i}(1,1)-intersect_plane1_dmin_max{i}(1,1);
    distance_waypoints2wallboundary_direction1(i,2)=intersect_plane1_dmin_max{i}(1,2)-intersect_plane1_d{i}(1,end);
    distance_waypoints2wallboundary_direction1(i,3)=intersect_plane1_dmin_max{i}(1,2)-intersect_plane1_dmin_max{i}(1,1);
    distance_waypoints2wallboundary_direction1(i,4)=room_plane_interval{i}(1);
    distance_waypoints2wallboundary_direction1(i,5)=floor(abs(intersect_plane1_dmin_max{i}(1,2)-intersect_plane1_dmin_max{i}(1,1))/room_plane_interval{i}(1));
    
    distance_waypoints2wallboundary_direction2(i,1)=intersect_plane2_d{i}(1,1)-intersect_plane2_dmin_max{i}(1,1);
    distance_waypoints2wallboundary_direction2(i,2)=intersect_plane2_dmin_max{i}(1,2)-intersect_plane2_d{i}(1,end);
    distance_waypoints2wallboundary_direction2(i,3)=intersect_plane2_dmin_max{i}(1,2)-intersect_plane2_dmin_max{i}(1,1);
    distance_waypoints2wallboundary_direction2(i,4)=room_plane_interval{i}(2);
    distance_waypoints2wallboundary_direction2(i,5)=floor(abs(intersect_plane2_dmin_max{i}(1,2)-intersect_plane2_dmin_max{i}(1,1))/room_plane_interval{i}(2));    
end

for i=1:1:size(room_plane_outer_boundary_point,2)
    for j=1:1:size(intersect_plane1_d{i},2)
        mat1=zeros(3,3);
        mat2=zeros(3,1);
        for k=1:1:size(intersect_plane2_d{i},2)
            mat1(1,1:3)=room_plane_parameters(i,1:3);
            mat1(2,1:3)=intersect_plane_norm_vector1(i,1:3);
            mat1(3,1:3)=intersect_plane_norm_vector2(i,1:3);
            mat2(1,1)=-room_plane_parameters(i,4);
            mat2(2,1)=-intersect_plane1_d{i}(1,j);
            mat2(3,1)=-intersect_plane2_d{i}(1,k);
            room_plane_intersect_point{i}(j,k,1:3)=inv(mat1)*mat2;
            room_plane_intersect_point{i}(j,k,4:6)=[0 0 0];
        end
    end
end

% the fifth step: select the effective points and show them off
for i=1:1:size(room_triangle_cell,2)
    for j=1:1:size(room_triangle_cell{i},1)
        room_triangle_edge_cell{i}(3*j-2,1:6)=room_triangle_cell{i}(j,1:6);
        room_triangle_edge_cell{i}(3*j-1,1:6)=room_triangle_cell{i}(j,7:12);
        room_triangle_edge_cell{i}(3*j,1:6)=room_triangle_cell{i}(j,13:18);
        
        room_triangle_plane_cell{i}(j,1:3)=room_plane_norm_vector{i}(1,1:3);
        room_triangle_plane_cell{i}(j,4)=-room_plane_norm_vector{i}(1,1:3)*room_triangle_edge_cell{i}(3*j,1:3)';
    end
end

for i=1:1:size(room_triangle_cell,2)
    for j=1:1:size(room_triangle_cell{i},1)
        for k=1:1:3
            delta1=room_triangle_edge_cell{i}(3*(j-1)+k,4:6)-room_triangle_edge_cell{i}(3*(j-1)+k,1:3);
            delta2=room_plane_norm_vector{i}(1,1:3);
            room_triangle_boundary_plane{i}(3*(j-1)+k,1)=delta1(2)*delta2(3)-delta2(2)*delta1(3);
            room_triangle_boundary_plane{i}(3*(j-1)+k,2)=delta1(3)*delta2(1)-delta2(3)*delta1(1);
            room_triangle_boundary_plane{i}(3*(j-1)+k,3)=delta1(1)*delta2(2)-delta2(1)*delta1(2);
            room_triangle_boundary_plane{i}(3*(j-1)+k,4)=-room_triangle_boundary_plane{i}(3*(j-1)+k,1:3)*room_triangle_edge_cell{i}(3*(j-1)+k,1:3)';
        end
    end
end

for i=1:1:size(room_triangle_cell,2)
    for j=1:1:size(room_triangle_cell{i},1)
        room_triangle_centroid_point{i}(j,1)=sum(room_triangle_edge_cell{i}(3*j-2:3*j,1)/6+room_triangle_edge_cell{i}(3*j-2:3*j,4)/6);
        room_triangle_centroid_point{i}(j,2)=sum(room_triangle_edge_cell{i}(3*j-2:3*j,2)/6+room_triangle_edge_cell{i}(3*j-2:3*j,5)/6);
        room_triangle_centroid_point{i}(j,3)=sum(room_triangle_edge_cell{i}(3*j-2:3*j,3)/6+room_triangle_edge_cell{i}(3*j-2:3*j,6)/6);        
        for k=1:1:3
            if room_triangle_centroid_point{i}(j,1:3)*room_triangle_boundary_plane{i}(3*(j-1)+k,1:3)'+room_triangle_boundary_plane{i}(3*(j-1)+k,4)<=0
                room_triangle_centriod_point_flag{i}(j,k)=0;
            else
                room_triangle_centriod_point_flag{i}(j,k)=1;
            end 
        end
    end
end

for i=1:1:size(room_plane_intersect_point,2)
    for j=1:1:size(room_plane_intersect_point{i},1)
        for k=1:1:size(room_plane_intersect_point{i},2)
            for m=1:1:size(room_triangle_cell{i},1)
                for n=1:1:3
                    sumt=0;
                    for s=1:1:3
                        sumt=sumt+room_plane_intersect_point{i}(j,k,s)*room_triangle_boundary_plane{i}(3*(m-1)+n,s);
                    end
                    if sumt+room_triangle_boundary_plane{i}(3*(m-1)+n,4)<=0
                        room_triangle_intersect_point_flag{i}(j,k,m,n)=0;
                    else
                        room_triangle_intersect_point_flag{i}(j,k,m,n)=1;
                    end
                end
            end
        end
    end
end


effective_intersect_point_num=0;
for i=1:1:size(room_plane_intersect_point,2)
    for j=1:1:size(room_plane_intersect_point{i},1)
        for k=1:1:size(room_plane_intersect_point{i},2)
            count=0;
            for m=1:1:size(room_triangle_cell{i},1)
                if room_triangle_intersect_point_flag{i}(j,k,m,1)==room_triangle_centriod_point_flag{i}(m,1) && room_triangle_intersect_point_flag{i}(j,k,m,2)==room_triangle_centriod_point_flag{i}(m,2) && room_triangle_intersect_point_flag{i}(j,k,m,3)==room_triangle_centriod_point_flag{i}(m,3)
                    count=1;
                end
            end
            room_plane_effective_intersect_point{i}(j,k,1:6)=room_plane_intersect_point{i}(j,k,1:6);            
            if count==1
                room_plane_effective_intersect_point{i}(j,k,7)=1;
                effective_intersect_point_num=effective_intersect_point_num+1;
            else
                room_plane_effective_intersect_point{i}(j,k,7)=0;
            end
        end
    end
end

room_effective_waypoints={};
room_effective_waypaths={};
for i=1:1:size(room_plane_effective_intersect_point,2)
    effective_room_waypoints_num=1;
    effective_room_waypaths_num=1;
    for j=1:1:size(room_plane_effective_intersect_point{i},1)
        for k=1:1:size(room_plane_effective_intersect_point{i},2)
            if room_plane_effective_intersect_point{i}(j,k,7)==1
                room_effective_waypoints{i}(effective_room_waypoints_num,1:3)=room_plane_effective_intersect_point{i}(j,k,1:3);
                effective_room_waypoints_num=effective_room_waypoints_num+1;
            end
        end
    end
    for j=1:1:size(room_plane_effective_intersect_point{i},1)-1
        for k=1:1:size(room_plane_effective_intersect_point{i},2)
            if room_plane_effective_intersect_point{i}(j,k,7)==1 && room_plane_effective_intersect_point{i}(j+1,k,7)==1
                room_effective_waypaths{i}(effective_room_waypaths_num,1:3)=room_plane_effective_intersect_point{i}(j,k,1:3);
                room_effective_waypaths{i}(effective_room_waypaths_num,4:6)=room_plane_effective_intersect_point{i}(j+1,k,1:3);
                effective_room_waypaths_num=effective_room_waypaths_num+1;
            end
        end
    end
end

for i=1:1:size(room_effective_waypaths,2)
    a=room_plane_norm_vector{i}(1,1);
    b=room_plane_norm_vector{i}(1,2);
    c=room_plane_norm_vector{i}(1,3);
    for j=1:1:size(room_effective_waypaths{i},1)
        for k=1:1:2
            renovation_effective_waypaths{i}(j,3*k-2)=room_effective_waypaths{i}(j,3*k-2)-a/sqrt(a^2+b^2+c^2)*panning_distance1;
            renovation_effective_waypaths{i}(j,3*k-1)=room_effective_waypaths{i}(j,3*k-1)-b/sqrt(a^2+b^2+c^2)*panning_distance1;
            renovation_effective_waypaths{i}(j,3*k)=room_effective_waypaths{i}(j,3*k)-c/sqrt(a^2+b^2+c^2)*panning_distance1;
        end
    end
    for j=1:1:size(room_effective_waypoints{i},1)
        renovation_effective_waypoints{i}(j,1)=room_effective_waypoints{i}(j,1)-a/sqrt(a^2+b^2+c^2)*panning_distance1;
        renovation_effective_waypoints{i}(j,2)=room_effective_waypoints{i}(j,2)-b/sqrt(a^2+b^2+c^2)*panning_distance1;
        renovation_effective_waypoints{i}(j,3)=room_effective_waypoints{i}(j,3)-c/sqrt(a^2+b^2+c^2)*panning_distance1;
    end
end
% renovation_effective_waypoints=room_effective_waypoints;
% renovation_effective_waypaths=room_effective_waypaths;



end