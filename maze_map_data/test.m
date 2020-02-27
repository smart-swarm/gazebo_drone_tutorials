load maze_map.mat
imshow(maze_map)
%test
% P(1, 1) <==> (20, 20)
% P(400, 1)<==> (-20, 20)
% P(400, 400)<==> (-20, -20)
% P(1, 400)<==> (20, -20)

get_pos_from_map_index(1, 1)
get_pos_from_map_index(400, 1)

get_index_from_pos(15.8, -12.0)
