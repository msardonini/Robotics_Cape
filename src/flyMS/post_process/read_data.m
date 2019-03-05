%  close all;
clear all; 
clc;
 
filename = uigetdir('./','pick a run for plotting');

nohup=csvread([filename '/logger.txt'],1,0);

% nohup(1,:) = []; % discard first log

time=nohup(:,1);

roll=nohup(:,2);
pitch=nohup(:,3);
yaw=nohup(:,4);

d_roll=nohup(:,5); 
d_pitch=nohup(:,6);
d_yaw=nohup(:,7);

wing1=nohup(:,8);
wing2=nohup(:,9);
wing3=nohup(:,10);
wing4=nohup(:,11);

throttle=nohup(:,12);
upitch=nohup(:,13);
uroll=nohup(:,14);
uyaw=nohup(:,15);

pitch_ref=nohup(:,16);
roll_ref=nohup(:,17);
yaw_ref=nohup(:,18);
yaw_rate_ref=nohup(:,19);

Aux=nohup(:,20);
lat_err=nohup(:,21);
lon_err=nohup(:,22);
kalman_lat = nohup(:,23);
kalman_lon = -nohup(:,24);

accel(:,1)=nohup(:,25);
accel(:,2)=nohup(:,26);
accel(:,3)=nohup(:,27);

v_batt=nohup(:,28);
baro_alt = nohup(:,29);
compass_heading=nohup(:,30);

ned_pos(:,1) = nohup(:,31);
ned_pos(:,2) = nohup(:,32);
ned_pos(:,3) = nohup(:,33);

ned_vel(:,1) = nohup(:,34);
ned_vel(:,2) = nohup(:,35);
ned_vel(:,3) = nohup(:,36);

mag(:,1) = nohup(:,37);
mag(:,2) = nohup(:,38);
mag(:,3) = nohup(:,39);


droll_setpoint = nohup(:,40);
dpitch_setpoint = nohup(:,41);

try
    nohup2=dlmread([filename '/GPS_logger.csv'],',',1,0);

    nohup2(1,:) = [];
    GPS_time = nohup2(:,1);
    deg_lon = nohup2(:,2);
    min_lon = nohup2(:,3);
    deg_lat = nohup2(:,4);
    min_lat = nohup2(:,5);
    speed = nohup2(2:end,6);
    direction = nohup2(:,7);
    gps_alt = nohup2(:,8);
    hdop = nohup2(:,9);
    fix = nohup2(:,10);

    gps_lat = deg_lat + min_lat/60;
    gps_lon = -deg_lon - min_lon/60;

    gps_meters_lat = (gps_lat)*111000;
    gps_meters_lon = (gps_lon)*111000 .* cosd(gps_lat);

    gps_meters_lat = gps_meters_lat - gps_meters_lat(1);
    gps_meters_lon = gps_meters_lon - gps_meters_lon(1);
    plot_gps=1;
catch
    plot_gps=0;
end

% throttle1=(wing1+wing2+wing3+wing4)/4;
% uroll=(wing1-wing2+wing3-wing4)/4;
% upitch=(wing3-wing2-wing1+wing4)/4;
% uyaw=(wing1-wing2-wing3+wing4)/4;




figure
hold on
plot(time,pitch)
plot(time,roll,'c')
plot(time,yaw,'g')
plot(time,compass_heading,'k')
legend('Pitch','Roll','Yaw','Compass Heading')
% ylim([-pi pi])


figure
hold on
plot(time,wing1,'y')
plot(time,wing2,'k')
plot(time,wing3,'r')
plot(time,wing4,'m')
legend('wing1','wing2','wing3','wing4')



figure
hold on
plot(time,d_pitch)
plot(time,d_roll,'c')
plot(time,d_yaw,'g')
legend('Pitch Vel','Roll Vel','Yaw Vel')
% ylim([-pi pi])



figure
hold on
plot(time,pitch_ref)
plot(time,roll_ref,'c')
plot(time,yaw_ref,'g')
legend('Pitch Ref','Roll Ref','Yaw Ref')
% ylim([-pi pi])


figure
hold on
% plot(time,throttle1)
plot(time,uyaw,'r','Linewidth',2)
plot(time,upitch,'k')
plot(time,uroll,'c')
legend('uYaw','uPitch','uRoll')


figure
plot(time,throttle)
title('throttle')


figure
hold on
plot(time,pitch)
plot(time,pitch_ref)
plot(time,upitch)
plot(time,d_pitch)
legend('pitch','ref','upitch', 'pitch vel')


figure
hold on
plot(pitch)
plot(pitch_ref)
plot(upitch)
plot(d_pitch/15)
plot(dpitch_setpoint)
legend('pitch','ref','upitch', 'pitch vel','dpitch_setpoint')

% figure
% plot(time,baro_alt)
% title('Barometer Altitude')
% 
% 
% figure
% hold on
% plot(time,accel(:,1))
% plot(time,accel(:,2))
% plot(time,accel(:,3))
% title('Acceleration')
% legend('x','y','z')
% 
% figure
% hold on
% plot(time,mag(:,1))
% plot(time,mag(:,2))
% plot(time,mag(:,3))
% legend('Mag1','Mag2','Mag3')
% title('Raw Magnetometer Data')

if plot_gps


    figure
    plot(ned_pos(:,2),ned_pos(:,1))
    title('NED Position North East')


    dlmwrite('gps_plot_data.csv',[gps_lat, gps_lon],'precision',14)
end
