 close all; clear all; clc;
 
 filename = uigetdir;
 cd(filename)
% load('flight_with_teather');e 
nohup=dlmread('logger.csv',',',1,0);

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
accel_lat=nohup(:,25);
accel_lon=nohup(:,26);
baro_alt =nohup(:,27);
v_batt =nohup(:,28);

try
    nohup2=dlmread('GPS_logger.csv',',',1,0);
deg_lon = nohup2(2:end,1);
min_lon = nohup2(2:end,2);
deg_lat = nohup2(2:end,3);
min_lat = nohup2(2:end,4);
speed = nohup2(2:end,5);
direction = nohup2(2:end,6);
gps_alt = nohup2(2:end,7);
hdop = nohup2(2:end,8);
fix = nohup2(2:end,9);

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
plot(time,yaw_rate_ref,'k')
legend('Pitch','Roll','Yaw','Yaw rate ref')
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
plot(time,upitch,'k')
plot(time,uroll,'c')
plot(time,uyaw,'r','Linewidth',2)
legend('uPitch','uRoll','uYaw')


figure
plot(time,throttle)
title('throttle')


figure
hold on
plot(time,pitch)
plot(time,pitch_ref)
plot(time,upitch)
legend('pitch','ref','upitch')

figure
plot(time,baro_alt)
title('Barometer Altitude')


figure
plot(time,v_batt)
title('Battery Voltage')

if plot_gps
    figure
    hold on
    plot(kalman_lon,kalman_lat)
    scatter(gps_meters_lon,gps_meters_lat)
    legend('Kalman Navigation')
    % 
    initial_lon = 0;
    initial_lat = 0;
    % 
    kalman_deg_lon = initial_lon + kalman_lon/(111000*cosd(initial_lat));
    kalman_deg_lat = initial_lat + kalman_lat/111000;


    figure
    hold on
    plot(kalman_deg_lon,kalman_deg_lat)
    legend('Kalman Navigation')


    cd ..


    dlmwrite('gps_plot_data.csv',[gps_lat, gps_lon],'precision',14)
end
