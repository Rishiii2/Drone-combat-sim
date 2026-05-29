% test_rrt.m
MAP = [0, 200, 0, 200];
OBS_POS = [70 70; 130 130; 70 130; 130 70];
OBS_R = 10.0;

start_pos = [55, 65];
goal_pos = [190, 40];

path = rrt_raw(start_pos, goal_pos, OBS_POS);

disp('Path size:')
disp(size(path, 1))

if size(path, 1) == 2
    disp('RRT FAILED to find path, returned straight line.')
else
    disp('RRT SUCCEEDED!')
end
