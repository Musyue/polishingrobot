function [room_facet,room_vertices,room_norm_vector]=vertices_processing(house_facet,house_vertices,house_norm_vector)

house_vertices(:,1)=house_vertices(:,1);
house_vertices(:,2)=house_vertices(:,2);
house_vertices(:,3)=house_vertices(:,3)-min(house_vertices(:,3));

m=1;
for i=1:1:size(house_norm_vector,1)
    % if abs(abs(house_norm_vector(i,3))-1)>=0.3
        house_norm_vector1(m,:)=house_norm_vector(i,:);
        house_vertices1(3*m-2:3*m,:)=house_vertices(3*i-2:3*i,:);
        m=m+1;
    % end
end

for i=1:1:1
    for j=1:1:size(house_norm_vector1,1)
        room_facet{i}(j,1:3)=[3*j-2, 3*j-1, 3*j];
        room_vertices{i}(3*j-2:3*j,1:3)=house_vertices1(3*j-2:3*j,1:3);
        room_norm_vector{i}(j,1:3)=house_norm_vector1(j,1:3);
    end
end

end