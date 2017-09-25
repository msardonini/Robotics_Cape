clear all
data = csvread('/home/msardonini/git/Robotics_Cape/src/rc_test_fusion/logger.csv',1,0);
time = data(:,1);
figure
plot(time,data(:,2))
hold on
plot(time,data(:,3))
plot(time,data(:,4))


figure
plot(diff(time))
% ylim([0 40000])
