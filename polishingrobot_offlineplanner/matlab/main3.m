
%function [renovation_cells_clustering_waypaths,renovation_cells_waypioints_onwaypath,renovation_cells_manipulatorbase_positions, manipulator_endeffector_positions_onpath]=renovation_cells_generation(room_plane_edge_cell,renovation_plane_edge_cell,renovation_effective_waypaths,renovation_plane_norm_vector,manipulatorbase_plane_edge_cell,length_interval,width_interval,path_distance)

path_interval=painting_path_interval;
length_interval=cell_length;
width_interval=cell_width;
path_distance=path_interval;

manipulatorbase_plane_edge_cell;
renovation_plane_norm_vector;
for i=1:1:size(renovation_plane_norm_vector,2)
    a=renovation_plane_norm_vector{i}(1,1);
    b=renovation_plane_norm_vector{i}(1,2);
    c=renovation_plane_norm_vector{i}(1,3);
    x0=manipulatorbase_plane_edge_cell{i}(1,1);
    y0=manipulatorbase_plane_edge_cell{i}(1,2);
    z0=manipulatorbase_plane_edge_cell{i}(1,3);
    d=-a*x0-b*y0-c*z0;
    renovation_manipulatorbase_planes{i}(1,1)=a;
    renovation_manipulatorbase_planes{i}(1,2)=b;
    renovation_manipulatorbase_planes{i}(1,3)=d;
end

%% obtain two types of norm vector of intersection plane norm vector for the renovation planes edges 
for i=1:1:size(renovation_plane_norm_vector,2)
    renovation_planes_point(i,1:3)=renovation_plane_edge_cell{i}(1,1:3);
    renovation_planes_parameters(i,1:3)=renovation_plane_norm_vector{i}(1,1:3);
    renovation_planes_parameters(i,4)=-renovation_plane_norm_vector{i}(1,1:3)*renovation_planes_point(i,1:3)';
end
intersect_plane_norm_vector=zeros(size(renovation_plane_norm_vector,2),3);
intersect_plane_norm_vector1=zeros(size(renovation_plane_norm_vector,2),3);
intersect_plane_norm_vector2=zeros(size(renovation_plane_norm_vector,2),3);
for i=1:1:size(intersect_plane_norm_vector,1)
    sin_theta=-renovation_planes_parameters(i,1);
    cos_theta=sqrt(1-sin_theta^2);
    if cos_theta~=0
        sin_beta=renovation_planes_parameters(i,2)/cos_theta;
        cos_beta=renovation_planes_parameters(i,3)/cos_theta;
    else
        sin_beta=0;
        cos_beta=1;
    end
    intersect_plane_norm_vector1(i,1:3)=[cos_theta, sin_theta*sin_beta, sin_theta*cos_beta];
    intersect_plane_norm_vector2(i,1:3)=[0, cos_beta, -sin_beta];
end


%% obtain dmin and dmax of intersection plane for the renovation planes edges.
for i=1:1:size(renovation_plane_edge_cell,2)
    for j=1:1:size(renovation_plane_edge_cell{i},1)
        renovation_planes_edge_points{i}(2*j-1,1:3)=renovation_plane_edge_cell{i}(j,1:3);
        renovation_planes_edge_points{i}(2*j,1:3)=renovation_plane_edge_cell{i}(j,4:6);
    end
    renovation_planes_edge_points{i}=unique(renovation_planes_edge_points{i},'rows');
end

for i=1:1:size(renovation_planes_edge_points,2)
    a=renovation_plane_norm_vector{i}(1,1);
    b=renovation_plane_norm_vector{i}(1,2);
    a1=intersect_plane_norm_vector1(i,1);
    b1=intersect_plane_norm_vector1(i,2);
    a2=intersect_plane_norm_vector2(i,1);
    b2=intersect_plane_norm_vector2(i,2);
    intersect_plane_norm_vector(i,1:3)=intersect_plane_norm_vector1(i,1:3);
    critical_plane_norm_vector(i,1:3)=intersect_plane_norm_vector2(i,1:3);

    for j=1:1:size(renovation_planes_edge_points{i},1)  
        intersect_plane_d_candidate{i}(1,j)=-intersect_plane_norm_vector(i,1:3)*renovation_planes_edge_points{i}(j,1:3)';
    end
    intersect_plane_dmin_max{i}(1,1)=min(intersect_plane_d_candidate{i});
    intersect_plane_dmin_max{i}(1,2)=max(intersect_plane_d_candidate{i});
end

for i=1:1:size(intersect_plane_dmin_max,2)
    dmin=intersect_plane_dmin_max{i}(1,1);
    dmax=intersect_plane_dmin_max{i}(1,2);
    plane_num=floor(abs(dmax-dmin)/length_interval);
    for j=1:1:plane_num 
        intersect_plane_d{i}(1,j)=dmin+j*length_interval;
    end
end

for i=1:1:size(renovation_planes_edge_points,2)
    for j=1:1:size(renovation_planes_edge_points{i},1)  
        intersect_plane2_d_candidate{i}(1,j)=-critical_plane_norm_vector(i,1:3)*renovation_planes_edge_points{i}(j,1:3)';
    end
    intersect_plane2_dmin_max{i}(1,1)=min(intersect_plane2_d_candidate{i});
    intersect_plane2_dmin_max{i}(1,2)=max(intersect_plane2_d_candidate{i});
end
for i=1:1:size(intersect_plane2_dmin_max,2)
    dmin_intersect_plane2=intersect_plane2_dmin_max{i}(1,1);
    dmax_intersect_plane2=intersect_plane2_dmin_max{i}(1,2);
    plane_num=floor(abs(dmax_intersect_plane2-dmin_intersect_plane2)/width_interval);
    for j=1:1:plane_num 
        intersect_plane2_d{i}(1,j)=dmin_intersect_plane2+j*width_interval;
    end
end


%% gather renovation cells edges into renovation cells, the output is:renovation_cells_waypaths
for i=1:1:size(intersect_plane_d,2)
    cell_num=1;
    h_num=size(intersect_plane_d{i},2)+1;
    for j=1:1:h_num
        for k=1:1:size(intersect_plane2_d{i},2)+1
            renovation_cells_edges_num=1;
            for n=1:1:size(renovation_effective_waypaths{i},1)
                p1=renovation_effective_waypaths{i}(n,1:3);
                p1_d1=-intersect_plane_norm_vector(i,1:3)*p1';
                p1_d2=-critical_plane_norm_vector(i,1:3)*p1';       
                p2=renovation_effective_waypaths{i}(n,4:6);
                p2_d1=-intersect_plane_norm_vector(i,1:3)*p2';
                p2_d2=-critical_plane_norm_vector(i,1:3)*p2';
                
                flag1=0; flag2=0;
                if j==1
                    if p1_d1<=intersect_plane_d{i}(1,j) && p2_d1<=intersect_plane_d{i}(1,j)
                        flag1=1;
                    end
                end
                if j~=size(intersect_plane_d{i},2)+1
                    if j~=1
                        if p1_d1>=intersect_plane_d{i}(1,j-1)-0.01 && p1_d1<=intersect_plane_d{i}(1,j)+0.01 && p2_d1>=intersect_plane_d{i}(1,j-1)-0.01 && p2_d1<=intersect_plane_d{i}(1,j)+0.01
                            flag1=1;
                        end
                    end
                end
                if j==size(intersect_plane_d{i},2)+1
                    if p1_d1>=intersect_plane_d{i}(1,j-1) && p2_d1>=intersect_plane_d{i}(1,j-1)
                        flag1=1;
                    end
                end
                
                if k==1
                    if p1_d2<=intersect_plane2_d{i}(1,k) && p2_d2<=intersect_plane2_d{i}(1,k)
                        flag2=1;
                    end
                end
                if  k~=size(intersect_plane2_d{i},2)+1
                    if k~=1
                        if p1_d2>=intersect_plane2_d{i}(1,k-1)-0.01 && p1_d2<=intersect_plane2_d{i}(1,k)+0.01 && p2_d2>=intersect_plane2_d{i}(1,k-1)-0.01 && p2_d2<=intersect_plane2_d{i}(1,k)+0.01
                            flag2=1;
                        end
                    end
                end
                if k==size(intersect_plane2_d{i},2)+1
                    if p1_d2>=intersect_plane2_d{i}(1,k-1) && p2_d2>=intersect_plane2_d{i}(1,k-1)
                        flag2=1;
                    end
                end
                % renovation_cells_waypaths
                if flag1==1 && flag2==1
                    renovation_cells_waypaths{i}{j}{k}(renovation_cells_edges_num,1:6)=renovation_effective_waypaths{i}(n,1:6);
                    renovation_cells_edges_num=renovation_cells_edges_num+1;
                end
                
            end
        end
    end
end

%% renovation_cells_clustering_waypaths should be generated from renovation_cells_waypaths 
%% input:renovation_cells_waypaths
%% output:renovation_cells_clustering_waypaths
for i=1:1:size(renovation_cells_waypaths,2)
    for j=1:1:size(renovation_cells_waypaths{i},2)
        for k=1:1:size(renovation_cells_waypaths{i}{j},2)
            a=renovation_cells_waypaths{i}{j}{k};
            b=renovation_path_clustering(a);
            renovation_cells_clustering_waypaths{i}{j}{k}=b;
            if i==2 && j==1 && k==1
                renovation_cells_clustering_waypaths{i}{j}{k};
            end
        end
    end
end



%% add orientations into renovation path waypoints 
theta_x=-pi/2;
theta_z=pi/2;
for i=1:1:size(renovation_plane_norm_vector,2)
    nx=renovation_plane_norm_vector{i}(1,1);
    ny=renovation_plane_norm_vector{i}(1,2);
    nz=renovation_plane_norm_vector{i}(1,3);
    sin_theta_y=nx;
    cos_theta_y=ny;
    if sin_theta_y>=0
        if cos_theta_y>=0
            theta_y(i)=asin(sin_theta_y);
        else
            theta_y(i)=pi-asin(sin_theta_y);
        end
    else
        if cos_theta_y>=0
            theta_y(i)=2*pi-asin(abs(sin_theta_y));
        else
            theta_y(i)=pi+asin(abs(sin_theta_y));
        end
    end
end 


for i=1:1:size(renovation_cells_clustering_waypaths,2)
    for j=1:1:size(renovation_cells_clustering_waypaths{i},2)
        for k=1:1:size(renovation_cells_clustering_waypaths{i}{j},2)
            for m=1:1:size(renovation_cells_clustering_waypaths{i}{j}{k},1)+1
                if m<=size(renovation_cells_clustering_waypaths{i}{j}{k},1)
                    renovation_cells_waypioints_onwaypath{i}{j}{k}(m,1:3)=renovation_cells_clustering_waypaths{i}{j}{k}(m,1:3);
                    renovation_cells_waypioints_onwaypath{i}{j}{k}(m,4:6)=[theta_x,theta_y(i),theta_z];
                else
                    renovation_cells_waypioints_onwaypath{i}{j}{k}(m,1:3)=renovation_cells_clustering_waypaths{i}{j}{k}(m-1,4:6);
                    renovation_cells_waypioints_onwaypath{i}{j}{k}(m,4:6)=[theta_x,theta_y(i),theta_z];
                end
            end
            if i==2 && j==1 && k==1
                renovation_cells_waypioints_onwaypath{i}{j}{k};
            end
        end
    end
end


%% compute manipulator base positions with orientations for each renovation cell:renovation_cells_manipulatorbase_positions
for i=1:1:size(renovation_cells_clustering_waypaths,2)
    for j=1:1:size(renovation_cells_clustering_waypaths{i},2)
        n=1; xy_vector=[];
        for k=1:1:size(renovation_cells_clustering_waypaths{i}{j},2)
            for m=1:1:size(renovation_cells_clustering_waypaths{i}{j}{k},1)
                xy_vector(n,1)=renovation_cells_clustering_waypaths{i}{j}{k}(m,1);
                xy_vector(n,2)=renovation_cells_clustering_waypaths{i}{j}{k}(m,2);
                n=n+1;
            end
        end
        xy_vector=unique(xy_vector,'rows');
        xy_vector=sortrows(xy_vector);
        renovationcells_horizontalposition{i}(2*j-1,1:2)=xy_vector(1,1:2);
        renovationcells_horizontalposition{i}(2*j,1:2)=xy_vector(end,1:2);
    end
end
for i=1:1:size(renovationcells_horizontalposition,2)
    for j=1:1:size(renovationcells_horizontalposition{i},1)/2
        x0=renovationcells_horizontalposition{i}(2*j-1,1);
        y0=renovationcells_horizontalposition{i}(2*j-1,2);
        x1=renovationcells_horizontalposition{i}(2*j,1);
        y1=renovationcells_horizontalposition{i}(2*j,2);
        distance=sqrt((x0-x1)^2+(y0-y1)^2);
        renovationcells_horizontal_midposition{i}(j,1)=(renovationcells_horizontalposition{i}(2*j-1,1)+renovationcells_horizontalposition{i}(2*j,1))/2;
        renovationcells_horizontal_midposition{i}(j,2)=(renovationcells_horizontalposition{i}(2*j-1,2)+renovationcells_horizontalposition{i}(2*j,2))/2;
    end
end
for i=1:1:size(renovationcells_horizontal_midposition,2)
    for j=1:1:size(renovationcells_horizontal_midposition{i},1)
        x0=renovationcells_horizontalposition{i}(2*j-1,1);
        y0=renovationcells_horizontalposition{i}(2*j-1,2);
        x1=renovationcells_horizontalposition{i}(2*j,1);
        y1=renovationcells_horizontalposition{i}(2*j,2);
        
        a1=y1-y0;
        b1=x0-x1;
        a2=b1;
        b2=-a1;
        c2=-(a2*renovationcells_horizontal_midposition{i}(j,1)+b2*renovationcells_horizontal_midposition{i}(j,2));
        
        intersection_line{i}(j,1)=a2;
        intersection_line{i}(j,2)=b2;
        intersection_line{i}(j,3)=c2;
        
    end
end
for i=1:1:size(intersection_line,2)
    for j=1:1:size(intersection_line{i},1)
        a1=intersection_line{i}(j,1);
        b1=intersection_line{i}(j,2);
        c1=intersection_line{i}(j,3);
        a2=renovation_manipulatorbase_planes{i}(1,1);
        b2=renovation_manipulatorbase_planes{i}(1,2);
        c2=renovation_manipulatorbase_planes{i}(1,3);
        
        renovation_horizontalcells_manipulatorbase_points{i}(j,1)=(c2*b1-c1*b2)/(a1*b2-a2*b1);
        renovation_horizontalcells_manipulatorbase_points{i}(j,2)=(c1*a2-c2*a1)/(a1*b2-a2*b1);
        renovation_horizontalcells_manipulatorbase_points{i}(j,3)=0;
    end
end

for i=1:1:size(renovation_plane_norm_vector,2)
    nx=renovation_plane_norm_vector{i}(1,1);
    ny=renovation_plane_norm_vector{i}(1,2);
    nz=renovation_plane_norm_vector{i}(1,3);
    base_theta_z(i)=atan2(ny,nx);
end 
for i=1:1:size(renovation_cells_clustering_waypaths,2)
    for j=1:1:size(renovation_cells_clustering_waypaths{i},2)
        for k=1:1:size(renovation_cells_clustering_waypaths{i}{j},2)
            num=size(renovation_cells_clustering_waypaths{i}{j}{k},1);
            if mod(num,2)==1
                z1=renovation_cells_clustering_waypaths{i}{j}{k}(1,3);
                z2=renovation_cells_clustering_waypaths{i}{j}{k}(end,3);
                z_mean=(z1+z2)/2;
            else
                z1=renovation_cells_clustering_waypaths{i}{j}{k}(1,3);
                z2=renovation_cells_clustering_waypaths{i}{j}{k}(end,3);
                z_mean=(z1+z2)/2+path_distance/2;
            end
            renovation_cells_manipulatorbase_positions{i}{j}{k}(1,1)=renovation_horizontalcells_manipulatorbase_points{i}(j,1);
            renovation_cells_manipulatorbase_positions{i}{j}{k}(1,2)=renovation_horizontalcells_manipulatorbase_points{i}(j,2);
            renovation_cells_manipulatorbase_positions{i}{j}{k}(1,3)=z_mean;
            renovation_cells_manipulatorbase_positions{i}{j}{k}(1,4)=0;
            renovation_cells_manipulatorbase_positions{i}{j}{k}(1,5)=0;
            renovation_cells_manipulatorbase_positions{i}{j}{k}(1,6)=base_theta_z(i);
        end
    end
end


%% compute manipulator end effector positions:manipulator_endeffector_positions_onpath
for i=1:1:size(renovation_cells_manipulatorbase_positions,2)
    for j=1:1:size(renovation_cells_manipulatorbase_positions{i},2)
        for k=1:1:size(renovation_cells_manipulatorbase_positions{i}{j},2)
% for i=1:1:1
%     for j=1:1:1
%         for k=1:1:1
            manipulatorbase_xyz=renovation_cells_manipulatorbase_positions{i}{j}{k}(1,1:3);
            manipulatorbase_rpy=renovation_cells_manipulatorbase_positions{i}{j}{k}(1,4:6);

            for m=1:1:size(renovation_cells_waypioints_onwaypath{i}{j}{k},1)
            % for m=1:1:1
                renovationpathwaypoints_xyz=renovation_cells_waypioints_onwaypath{i}{j}{k}(m,1:3);
                renovationpathwaypoints_rpy=renovation_cells_waypioints_onwaypath{i}{j}{k}(m,4:6);
                rot_mat=rotz(manipulatorbase_rpy(3))';
                tran=rot_mat*(renovationpathwaypoints_xyz-manipulatorbase_xyz)';
                
                manipulatorpathwaypoints_xyzrpy=[];
                manipulatorpathwaypoints_xyzrpy(1,1)=tran(1);
                manipulatorpathwaypoints_xyzrpy(1,2)=tran(2);
                manipulatorpathwaypoints_xyzrpy(1,3)=tran(3);
                
                if k==1
                    manipulatorpathwaypoints_xyzrpy(1,4)=0;
                    manipulatorpathwaypoints_xyzrpy(1,5)=pi/2;
                    manipulatorpathwaypoints_xyzrpy(1,6)=0;
                else
                    manipulatorpathwaypoints_xyzrpy(1,4)=0;
                    manipulatorpathwaypoints_xyzrpy(1,5)=pi/2;
                    manipulatorpathwaypoints_xyzrpy(1,6)=pi;
                end
                
                manipulator_endeffector_positions_onpath{i}{j}{k}(m,1:6)=manipulatorpathwaypoints_xyzrpy(1,1:6);
            end
            
            if i==2 && j==1 && k==1
                manipulator_endeffector_positions_onpath{i}{j}{k};
            end
            
        end
    end
end

save('scan_data2.mat','manipulator_endeffector_positions_onpath','renovation_cells_manipulatorbase_positions','renovation_cells_waypioints_onwaypath');
renovation_cells_waypath_visualization(renovation_cells_waypioints_onwaypath,renovation_cells_manipulatorbase_positions,renovation_plane_edge_cell,room_plane_edge_cell,manipulatorbase_plane_edge_cell);

positions_num=0;
for i=1:1:size(renovation_cells_manipulatorbase_positions,2)
    for j=1:1:size(renovation_cells_manipulatorbase_positions{i},2)
        for k=1:1:size(renovation_cells_manipulatorbase_positions{i}{j},2)
            position=renovation_cells_manipulatorbase_positions{i}{j}{k}(1,1:3);
            fprintf('the position is: %6.4f, %6.4f, %6.4f\n',position(1),position(2),position(3));
            positions_num=positions_num+1;
        end
    end
end
fprintf('the position number is: %6.4f\n',positions_num);


%  end



