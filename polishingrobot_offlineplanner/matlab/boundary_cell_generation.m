function [boundary_cell]=boundary_cell_generation(ground_boundaries)
% 在这里，输入的格式为ground_boundaries(i,6),i:第i条线段
%输出的格式为：boundary_cell{j}(k,6),第j个cell的第k条线段
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
end
% three important flags: flag



