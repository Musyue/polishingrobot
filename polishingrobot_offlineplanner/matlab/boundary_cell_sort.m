function [boundary_cell_sort]=boundary_cell_sort(boundary_cell)
cell_num=size(boundary_cell,2);
% boundary_cell_mat=load('file3.mat');
% boundary_cell=boundary_cell_mat.boundary_cell;

for i=1:1:cell_num
    boundary_cell_sort{i}(:,1:6)=boundary_cell{i}(:,1:6);
    boundaries_num=size(boundary_cell{i},1);
    for j=1:1:boundaries_num
        boundary_cell_sort{i}(j,7)=0;
    end
end

boundary_cell_range=zeros(cell_num,6);
for i=1:1:cell_num
    xmin=min(min(boundary_cell{i}(:,1)),min(boundary_cell{i}(:,4)));
    xmax=max(max(boundary_cell{i}(:,1)),max(boundary_cell{i}(:,4)));
    ymin=min(min(boundary_cell{i}(:,2)),min(boundary_cell{i}(:,5)));
    ymax=max(max(boundary_cell{i}(:,2)),max(boundary_cell{i}(:,5)));
    zmin=min(min(boundary_cell{i}(:,3)),min(boundary_cell{i}(:,6)));
    zmax=max(max(boundary_cell{i}(:,3)),max(boundary_cell{i}(:,6)));
    boundary_cell_range(i,1)=xmin;
    boundary_cell_range(i,2)=xmax;
    boundary_cell_range(i,3)=ymin;
    boundary_cell_range(i,4)=ymax;
    boundary_cell_range(i,5)=zmin;
    boundary_cell_range(i,6)=zmax;
end
xxmin=min(boundary_cell_range(:,1));
xxmax=max(boundary_cell_range(:,2));
yymin=min(boundary_cell_range(:,3));
yymax=max(boundary_cell_range(:,4));
zzmin=min(boundary_cell_range(:,5));
zzmax=max(boundary_cell_range(:,6));
xyz_range=[xxmin,xxmax,yymin,yymax,zzmin,zzmax];
for i=1:1:cell_num
    if all(boundary_cell_range(i,1:6)==xyz_range)
        boundary_cell_sort{i}(:,7)=1;
    end
end
end