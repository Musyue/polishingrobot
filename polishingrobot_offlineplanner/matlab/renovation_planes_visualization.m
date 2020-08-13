function renovation_planes_visualization(room_plane_edge_cell,renovation_plane_edge_cell,mobilebase_plane_edge_cell)
figure;
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
axis equal;
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
end
axis equal;
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