function [renovation_cells_paths]=renovation_path_clustering(renovation_cells_waypaths1)

% clc,clear all, close all
% data=load('data4.mat','renovation_cells_waypaths');
% ground_boundaries=data.renovation_cells_waypaths{1}{1}{1};

ground_boundaries=renovation_cells_waypaths1;
boundary_num=size(ground_boundaries,1);
flag1=ones(1,boundary_num);
boundary_cell{1}(1,1:6)=ground_boundaries(1,1:6);
flag1(1,1)=0;
while(1)
    boundary_cell_num=size(boundary_cell,2);
    flag2=ones(1,boundary_cell_num);
    for j=1:1:boundary_cell_num
        boundary_cell_boundary_num=size(boundary_cell{j},1);
        boundary_cell_boundary_num_before=boundary_cell_boundary_num;
        for k=1:1:boundary_cell_boundary_num
            for i=1:1:boundary_num
                p1=ground_boundaries(i,1:6);
                p2=boundary_cell{j}(k,1:3);
                p3=boundary_cell{j}(k,4:6);
                flag3=0;
                flag4=0; flag5=0; flag6=0; flag7=0;
                if p1(1)==p2(1) && p1(2)==p2(2) && p1(3)==p2(3)
                    flag4=1;
                end
                if p1(4)==p3(1) && p1(5)==p3(2) && p1(6)==p3(3)
                    flag5=1;
                end
                if p1(1)==p3(1) && p1(2)==p3(2) && p1(3)==p3(3)
                    flag6=1;
                end
                if p1(4)==p2(1) && p1(5)==p2(2) && p1(6)==p2(3)
                    flag7=1;
                end
                if flag4==1 || flag5==1 || flag6==1 || flag7==1
                    flag3=1;
                end
                for n=1:1:size(boundary_cell{j},1)
                    p4=boundary_cell{j}(n,1:6);
                    if p1(1)==p4(1) && p1(2)==p4(2) && p1(3)==p4(3) && p1(4)==p4(4) && p1(5)==p4(5) && p1(6)==p4(6)
                        flag3=0;
                    end
                end
                if flag3==1
                    boundary_cell_boundary_num=boundary_cell_boundary_num+1;
                    boundary_cell{j}(boundary_cell_boundary_num,1:6)=ground_boundaries(i,1:6);
                    flag1(1,i)=0;
                end
            end
        end
        boundary_cell_boundary_num_after=size(boundary_cell{j},1);
        if boundary_cell_boundary_num_before~=boundary_cell_boundary_num_after
            flag2(1,j)=0;
        end
    end
    if all(flag1==zeros(1,boundary_num))
        break;
    end
    if all(flag2==ones(1,boundary_cell_num))
        boundary_cell_num=boundary_cell_num+1;
        index=min(find(flag1==1));
        boundary_cell{boundary_cell_num}(1,1:6)=ground_boundaries(index,1:6);
        flag1(1,index)=0;
    end
end

% renovation_cells_paths=boundary_cell;

for i=1:1:size(boundary_cell,2)
    onepathpoints=[];
    for j=1:1:size(boundary_cell{i},1)
        onepathpoints(2*j-1,1:3)=boundary_cell{i}(j,1:3);
        onepathpoints(2*j,1:3)=boundary_cell{i}(j,4:6);
    end
    onepathpoints=unique(onepathpoints,'rows');
    onepathpoints=sortrows(onepathpoints);
    mat1(1,1:3)=onepathpoints(1,1:3);
    mat1(2,1:3)=onepathpoints(end,1:3);
    mat1=sortrows(mat1);
    if mod(i,2)==1
        pathspoints(2*i-1,1:3)=mat1(1,1:3);
        pathspoints(2*i,1:3)=mat1(2,1:3);
    else
        pathspoints(2*i-1,1:3)=mat1(2,1:3);
        pathspoints(2*i,1:3)=mat1(1,1:3);
    end
end
% pathspoints=sortrows(pathspoints);
for i=1:1:size(pathspoints,1)/2
   horizontal_pathspoints(i,1:3)=pathspoints(2*i-1,1:3);
   horizontal_pathspoints(i,4:6)=pathspoints(2*i,1:3);
end
horizontal_pathspoints=sortrows(horizontal_pathspoints,6,'descend');
for i=1:1:size(horizontal_pathspoints,1)
    mat1(1,1:3)=horizontal_pathspoints(i,1:3);
    mat1(2,1:3)=horizontal_pathspoints(i,4:6);
    if mod(i,2)==1
        mat1=sortrows(mat1,'ascend');
    else
        mat1=sortrows(mat1,'descend');
    end
    pathspoints1(2*i-1,1:3)=mat1(1,1:3);
    pathspoints1(2*i,1:3)=mat1(2,1:3);
end

for i=1:1:size(pathspoints1,1)-1
    renovation_cells_paths(i,1:3)=pathspoints1(i,1:3);
    renovation_cells_paths(i,4:6)=pathspoints1(i+1,1:3);
end

% figure;
% for i=1:1:size(renovation_cells_paths,1)
%     x1=[renovation_cells_paths(i,1),renovation_cells_paths(i,4)];
%     y1=[renovation_cells_paths(i,2),renovation_cells_paths(i,5)];
%     z1=[renovation_cells_paths(i,3),renovation_cells_paths(i,6)];
%     plot3(x1,y1,z1,'k','LineWidth',1)
%     hold on;
%     axis equal;
%     view(-114,24);
% end

end



