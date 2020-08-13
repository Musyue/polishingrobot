function room_stl_matplot(facet, vertices, norm_vector)

figure;
for i=1:1:size(facet,2)
    xlabel("x axis");
    ylabel("y axis");
    zlabel("z axis");
    title('3D model of one house','FontSize',24);
    % patch('Faces',facet,'Vertices',vertices);
    patch('Faces',facet{i},'Vertices',vertices{i},'FaceColor',[0.8 0.8 1.0], ...
        'EdgeColor','none',...
        'FaceLighting','gouraud',...
        'AmbientStrength', 0.15);
    camlight('headlight');
    material('dull');
    view(-150,20);
    axis equal;
    hold on;
end


end
