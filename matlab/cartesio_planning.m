clear
clc
close all

% Load the latest .mat file available in /home/luca/my_log
dir_content = dir('/home/luca/my_log');

for ii =3:size(dir_content,1)
    if dir_content(ii).isdir == true
        date(ii) = 0;
    else
        date(ii) = dir_content(ii).datenum;
    end
end

i = find(date == max(date));
load([dir_content(i).folder '/' dir_content(i).name])
disp(['Loaded: ' dir_content(i).folder '/' dir_content(i).name])

figure(1)
title("Tree Vertices")
for i = 1:size(state,2)
    x(i) = sum(state(1:2:end, i));
    y(i) = sum(state(2:2:end, i));  
end
plot(x/4,y/4,'bo')
hold on
plot(0,0,'ro','LineWidth',2)
plot(3.0,0,'go','LineWidth',2)
xlim([-1.0 6.0])
ylim([-2.0 2.0])
rectangle('Position', [1, 0.35, 1, 1])
rectangle('Position', [1, -1.35, 1, 1])
grid on



    
