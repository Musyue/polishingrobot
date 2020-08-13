function [room_plane_norm_vector,room_plane_edge_cell,room_plane_edge_centroid,room_triangle_cell, room_plane_triangle_edge_cell]=room_planes_generation(room_facet,room_vertices,room_norm_vector)

% computation of room_planes{i}{j}
% computation of room_plane_boundary{i}{j}
% computation of room_plane_points{i}{j}

%% first step: build up triangles 
for i=1:1:size(room_vertices,2)
    for j=1:1:size(room_vertices{i},1)/3
        p1=room_vertices{i}(3*j-2,1:3);
        p2=room_vertices{i}(3*j-1,1:3);
        p3=room_vertices{i}(3*j,1:3);
        triangle{i}(j,1:6)=pointsort([p1,p2]);
        triangle{i}(j,7:12)=pointsort([p2,p3]);
        triangle{i}(j,13:18)=pointsort([p1,p3]);
    end
end
%% second step: greedy strategies to build up planes 
room_triangle=triangle{1};
triangle_num=size(room_triangle,1);
flag1=ones(1,triangle_num);
room_triangle_cell{1}(1,1:18)=room_triangle(1,1:18);
triangle_norm_vector=room_norm_vector{1};
triangle_norm_vector_cell{1}(1,1:3)=triangle_norm_vector(1,1:3);
flag1(1,1)=0;

while(1)
    triangle_cell_num=size(room_triangle_cell,2);
    flag2=ones(1,triangle_cell_num);
    for j=1:1:triangle_cell_num
        triangle_cell_triangle_num=size(room_triangle_cell{j},1);
        triangle_cell_triangle_num_before=triangle_cell_triangle_num;
        for k=1:1:triangle_cell_triangle_num
            for i=1:1:triangle_num
                flag3=0;
                
                
                if abs(triangle_norm_vector(i,1)-triangle_norm_vector_cell{j}(k,1))<0.1 || abs(triangle_norm_vector(i,2)-triangle_norm_vector_cell{j}(k,2))<0.1
                    flag3=1;
                end
                for n=1:1:size(room_triangle_cell{j},1)
                     if all(room_triangle_cell{j}(n,1:18)==room_triangle(i,1:18))
                         flag3=0;
                     end
                end
                if flag3==1
                    triangle_cell_triangle_num=triangle_cell_triangle_num+1;
                    room_triangle_cell{j}(triangle_cell_triangle_num,1:18)=room_triangle(i,1:18);
                    triangle_norm_vector_cell{j}(triangle_cell_triangle_num,1:3)=triangle_norm_vector(i,1:3);
                    flag1(1,i)=0;
                end
            end
        end
        triangle_cell_triangle_num_after=size(room_triangle_cell{j},1);
        if triangle_cell_triangle_num_before~=triangle_cell_triangle_num_after
            flag2(1,j)=0;
        end
    end
    if all(flag1==zeros(1,triangle_num))
        break;
    end
    if all(flag2==ones(1,triangle_cell_num))
        triangle_cell_num=triangle_cell_num+1;
        index=min(find(flag1==1));
        room_triangle_cell{triangle_cell_num}(1,1:18)=room_triangle(index,1:18);
        triangle_norm_vector_cell{triangle_cell_num}(1,1:3)=triangle_norm_vector(index,1:3);
        flag1(1,index)=0;
    end
end


%% third step:šgenerate room_plane_triangle_edge_cell from room_triangle_cell
for i=1:1:size(room_triangle_cell,2)
    for j=1:1:size(room_triangle_cell{i},1)
        room_plane_triangle_edge_cell{i}(3*j-2,1:6)=room_triangle_cell{i}(j,1:6);
        room_plane_triangle_edge_cell{i}(3*j-1,1:6)=room_triangle_cell{i}(j,7:12);
        room_plane_triangle_edge_cell{i}(3*j,1:6)=room_triangle_cell{i}(j,13:18);
    end
end

%% fourth step: generate room triangle centroids
for i=1:1:size(room_triangle_cell,2)
    for j=1:1:size(room_triangle_cell{i},1)
        room_triangle_centriod{i}(j,1)=sum(room_plane_triangle_edge_cell{i}(3*j-2:3*j,1)+room_plane_triangle_edge_cell{i}(3*j-2:3*j,4))/6;
        room_triangle_centriod{i}(j,2)=sum(room_plane_triangle_edge_cell{i}(3*j-2:3*j,2)+room_plane_triangle_edge_cell{i}(3*j-2:3*j,5))/6;
        room_triangle_centriod{i}(j,3)=sum(room_plane_triangle_edge_cell{i}(3*j-2:3*j,3)+room_plane_triangle_edge_cell{i}(3*j-2:3*j,6))/6;
    end
end

%% fifth step: generate room_plane_edge_cell, room_plane_edge_centroid
for i=1:1:size(room_triangle_cell,2)
    m=1;
    for j=1:1:size(room_plane_triangle_edge_cell{i},1)
        adjacent_num=0;
        for k=1:1:size(room_plane_triangle_edge_cell{i},1)
            if j~=k
                if room_plane_triangle_edge_cell{i}(j,1:6)==room_plane_triangle_edge_cell{i}(k,1:6)
                    adjacent_num=adjacent_num+1;
                end
            end
        end
        if adjacent_num==0
            room_plane_edge_cell{i}(m,1:6)=room_plane_triangle_edge_cell{i}(j,1:6);
            triangle_num=ceil(j/3);
            room_plane_edge_centroid{i}(m,1:3)=room_triangle_centriod{i}(triangle_num,1:3);
            m=m+1;
        end
    end
    room_plane_norm_vector{i}(1,1:3)=triangle_norm_vector_cell{i}(1,1:3);
end 



end





function [points]=pointsort(points)
% sorting points based on their x position
points_num=size(points,2)/3;
for i=1:1:points_num
    single_points(i,1:3)=points(1,3*i-2:3*i);
end
for i=1:1:points_num-1
    for j=1:1:points_num-i
        if single_points(j,1)>single_points(j+1,1)
            [single_points(j,1:3),single_points(j+1,1:3)]=swap(single_points(j,1:3),single_points(j+1,1:3));
%             x=['1'];
%             disp(x);
        else if single_points(j,1)==single_points(j+1,1)
                if   single_points(j,2)>single_points(j+1,2)
                    [single_points(j,1:3),single_points(j+1,1:3)]=swap(single_points(j,1:3),single_points(j+1,1:3));
%                     x=['2'];
%                     disp(x);
                else if single_points(j,2)==single_points(j+1,2)
                        if  single_points(j,3)>single_points(j+1,3)
                            [single_points(j,1:3),single_points(j+1,1:3)]=swap(single_points(j,1:3),single_points(j+1,1:3));
%                             x=['3'];
%                             disp(x);
                        end
                    end
                end
            end
        end
    end
end

for i=1:1:points_num
    points(1,3*i-2:3*i)=single_points(i,1:3);
end
end

function [a,b]=swap(x,y)
a=y;
b=x;
end

