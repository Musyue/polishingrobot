function renovation_cells_waypath_visualization(renovation_cells_waypioints_onwaypath,renovation_cells_manipulatorbase_positions,renovation_plane_edge_cell,room_plane_edge_cell,mobilebase_plane_edge_cell)

%% drawing all data
figure;

for i=1:1:size(renovation_cells_waypioints_onwaypath,2)
    for j=1:1:size(renovation_cells_waypioints_onwaypath{i},2)
        for k=1:1:size(renovation_cells_waypioints_onwaypath{i}{j},2)
            for m=1:1:size(renovation_cells_waypioints_onwaypath{i}{j}{k},1)-1
                x1=[renovation_cells_waypioints_onwaypath{i}{j}{k}(m,1),renovation_cells_waypioints_onwaypath{i}{j}{k}(m+1,1)];
                y1=[renovation_cells_waypioints_onwaypath{i}{j}{k}(m,2),renovation_cells_waypioints_onwaypath{i}{j}{k}(m+1,2)];
                z1=[renovation_cells_waypioints_onwaypath{i}{j}{k}(m,3),renovation_cells_waypioints_onwaypath{i}{j}{k}(m+1,3)];
                plot3(x1,y1,z1,'b','LineWidth',1);
                hold on;
            end
            
            x0=renovation_cells_manipulatorbase_positions{i}{j}{k}(:,1);
            y0=renovation_cells_manipulatorbase_positions{i}{j}{k}(:,2);
            z0=renovation_cells_manipulatorbase_positions{i}{j}{k}(:,3);
            scatter3(x0,y0,z0,'*');
            hold on;
            axis equal;
        end
    end
end
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
for i=1:1:size(renovation_plane_edge_cell,2)
    for j=1:1:size(renovation_plane_edge_cell{i},1)
        xlabel("x axis");
        ylabel("y axis");
        zlabel("z axis");
        title('3D model of interior surfaces framework','FontSize',24);
        x1=[renovation_plane_edge_cell{i}(j,1),renovation_plane_edge_cell{i}(j,4)];
        y1=[renovation_plane_edge_cell{i}(j,2),renovation_plane_edge_cell{i}(j,5)];
        z1=[renovation_plane_edge_cell{i}(j,3),renovation_plane_edge_cell{i}(j,6)];
        plot3(x1,y1,z1,'r','LineWidth',1);
        axis equal;
        view(-114,24);
        hold on;
    end
    hold on;
end
hold on;

for i=1:1:size(mobilebase_plane_edge_cell,2)
    for j=1:1:size(mobilebase_plane_edge_cell{i},1)
        xlabel("x axis");
        ylabel("y axis");
        zlabel("z axis");
        title('3D model of interior surfaces framework','FontSize',24);
        x1=[mobilebase_plane_edge_cell{i}(j,1),mobilebase_plane_edge_cell{i}(j,4)];
        y1=[mobilebase_plane_edge_cell{i}(j,2),mobilebase_plane_edge_cell{i}(j,5)];
        z1=[mobilebase_plane_edge_cell{i}(j,3),mobilebase_plane_edge_cell{i}(j,6)];
        plot3(x1 ,y1,z1,'k','LineWidth',1);
        axis equal;
        view(-114,24);
        hold on;
    end
end

end