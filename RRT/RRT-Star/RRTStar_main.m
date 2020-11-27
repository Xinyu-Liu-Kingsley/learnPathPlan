%  /*RRT-Connect algoritm in 2D with collision avoidance
% *Version: MATLAB 2017b
% *Time: 2020.07.07
% *Author: Joey Zhou */

clc;
close all;
clear;

num_of_runs = 1;
dim = 2;
% stepsize = [5, 7, 9]; %调试不同的生长步长
stepsize = 7;
radius = 10;
samples = 2000;

random_world = 0;
show_output = 1;

t_star = [];
l_star = [];
p_star = [];

for sits = 1:size(stepsize,2)
    segmentLength = stepsize(1,sits);
    str0 = ['Segment length is ', num2str(segmentLength)];
    
    time = 0;
    avg_its = 0;
    avg_path = 0;
    for i = 1:num_of_runs
        [n_its, path_n, run_time] =  RRTstar3D(dim,segmentLength,radius,random_world,show_output,samples);
        time = time + run_time;
        avg_its = avg_its + n_its;
        avg_path = avg_path + path_n;
    end

    str1 = ['The time taken by RRT-Star for ', num2str(num_of_runs), ' runs is ', num2str(time)];
    str2 = ['The averagae time taken by RRT-Star for each run is ', num2str(time/num_of_runs)];
    str3 = ['The averagae number of states explored by RRT-Star for each run is ', num2str(avg_its/num_of_runs)];
    str4 = ['The averagae number of state in Path by RRT-Star for each run is ', num2str(avg_path/num_of_runs)];

    disp('%%%%%%%%%%%%%%%%%%% RRT-Star %%%%%%%%%%%%%%%%%%%%%%%%%%');
    disp(str0);
    disp(str1);
    disp(str2);
    disp(str3);
    disp(str4);
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');

end


