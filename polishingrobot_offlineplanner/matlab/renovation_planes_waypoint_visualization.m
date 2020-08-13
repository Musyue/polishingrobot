function renovation_planes_waypoint_visualization(renovation_effective_waypoints,room_plane_edge_cell,renovation_plane_edge_cell,renovation_effective_waypaths)
figure;
for i=1:1:size(renovation_plane_edge_cell,2)
    for j=1:1:size(renovation_plane_edge_cell{i},1)
        xlabel("x axis");
        ylabel("y axis");
        zlabel("z axis");
        title('3D model of interior surfaces framework','FontSize',24);
        x1=[renovation_plane_edge_cell{i}(j,1),renovation_plane_edge_cell{i}(j,4)];
        y1=[renovation_plane_edge_cell{i}(j,2),renovation_plane_edge_cell{i}(j,5)];
        z1=[renovation_plane_edge_cell{i}(j,3),renovation_plane_edge_cell{i}(j,6)];
        plot3(x1 ,y1,z1,'r','LineWidth',1);
        axis equal;
        view(-114,24);
        hold on;
    end
    hold on;
end
hold on;
for i=1:1:size(room_plane_edge_cell,2)
    for j=1:1:size(room_plane_edge_cell{i},1)
        xlabel("x axis");
        ylabel("y axis");
        zlabel("z axis");
        title('3D model of interior surfaces framework','FontSize',24);
        x1=[room_plane_edge_cell{i}(j,1),room_plane_edge_cell{i}(j,4)];
        y1=[room_plane_edge_cell{i}(j,2),room_plane_edge_cell{i}(j,5)];
        z1=[room_plane_edge_cell{i}(j,3),room_plane_edge_cell{i}(j,6)];
        plot3(x1 ,y1,z1,'b','LineWidth',1);
        axis equal;
        view(-114,24);
        hold on;
    end
end
hold on;
for i=1:1:size(renovation_effective_waypoints,2)
    x=renovation_effective_waypoints{i}(:,1);
    y=renovation_effective_waypoints{i}(:,2);
    z=renovation_effective_waypoints{i}(:,3);
    scatter3(x,y,z);
    hold on;
end
hold on;
for i=1:1:size(renovation_effective_waypaths,2)
    for j=1:1:size(renovation_effective_waypaths{i},1)
        x1=[renovation_effective_waypaths{i}(j,1),renovation_effective_waypaths{i}(j,4)];
        y1=[renovation_effective_waypaths{i}(j,2),renovation_effective_waypaths{i}(j,5)];
        z1=[renovation_effective_waypaths{i}(j,3),renovation_effective_waypaths{i}(j,6)];
        plot3(x1,y1,z1,'b','LineWidth',1);
        hold on;
    end
end
axis equal;
end


