close all

good = sum(success == 1);
bad = sum(success == 0);

average = sum(time) / length(time);
average_ik = sum(time_ik)/length(time_ik);
average_stability = sum(stability)/length(stability);
average_collisions = sum(collisions)/length(collisions);

% fails_coll = sum(coll_fails)/sum(tot_calls);
% fails_stab = sum(stab_fails)/sum(tot_calls);

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
disp(["average IK: ", sum(time_ik)/length(time_ik)])
disp(["average stability: ", sum(stability)/length(stability)])
disp(["average collisions: ", sum(collisions)/length(collisions)])
disp(["average #calls: ", average/(average_ik + average_stability + average_collisions)])
% disp(["% collision fails: ", fails_coll])
% disp(["% stability fails: ", fails_stab])
% disp(["total time: ", sum(time)])


