clc,clear all,close all;
tic;

%% generating room planes of interior environment
[house_facet,house_vertices,house_norm_vector]=house_stl_reading("bim_document/20200716_scan_planning_1.stl");
% house_stl_matplot(house_facet,house_vertices,house_norm_vector);

[room_facet,room_vertices,room_norm_vector]=vertices_processing(house_facet,house_vertices,house_norm_vector);
room_stl_matplot(room_facet,room_vertices,room_norm_vector)

%% generating room planes 
[room_plane_norm_vector,room_plane_edge_cell,room_plane_edge_centroid,room_plane_triangle_cell,room_plane_triangle_edge_cell]=room_planes_generation(room_facet,room_vertices,room_norm_vector);
save('scan_data1.mat','room_vertices','room_plane_norm_vector','room_plane_edge_cell','room_plane_edge_centroid','room_plane_triangle_edge_cell','room_plane_triangle_cell');


toc;


