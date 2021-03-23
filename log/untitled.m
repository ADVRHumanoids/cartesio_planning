close all

good = sum(success == 1);
bad = sum(success == 0);

average = sum(time) / length(time);

figure
plot([1:length(time)], time)

figure
plot([1:length(collisions)], collisions, 'b')
hold on
plot([1:length(stability)], stability, 'r')
plot([1:length(time_ik)], time_ik, 'g')
legend('collision check', 'centroidal statics check', 'IK solve')

disp(["success rate is: ", good/length(success)*100])
disp(["average: ", average])