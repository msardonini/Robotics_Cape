 close all; clc;
% load('flight_with_teather');
nohup=dlmread('logger.txt');
wing1=nohup(:,1);
wing2=nohup(:,2);
wing3=nohup(:,3);
wing4=nohup(:,4);
Pitch=nohup(:,5);
Roll=nohup(:,6);
Yaw=nohup(:,7);
d_pitch=nohup(:,8);
d_roll=nohup(:,9);
d_yaw=nohup(:,10);
throttle=nohup(:,11);
pitch_ref=nohup(:,12);
roll_ref=nohup(:,13);
yaw_ref=nohup(:,14);
unwrapped_yaw=nohup(:,15);
filtered_pitch=nohup(:,16);
ePitch=nohup(:,17);
eRoll=nohup(:,18);
eYaw=nohup(:,19);
upitch=nohup(:,20);
uroll=nohup(:,21);
uyaw=nohup(:,22);
Altitude=nohup(:,23);
d_Altitude=nohup(:,24);
time=linspace(0,length(wing1)/200,length(wing1));



% throttle1=(wing1+wing2+wing3+wing4)/4;
% uroll=(wing1-wing2+wing3-wing4)/4;
% upitch=(wing3-wing2-wing1+wing4)/4;
% uyaw=(wing1-wing2-wing3+wing4)/4;




figure
hold on
plot(time,Pitch)
plot(time,Roll,'c')
plot(time,Yaw,'g')
plot(time,unwrapped_yaw,'k')
legend('Pitch','Roll','Yaw','Unwrapped Yaw')
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
plot(time,filtered_pitch)
plot(time,roll_ref,'c')
plot(time,yaw_ref,'g')
legend('Pitch Ref','Filtered Pitch','Roll Ref','Yaw Ref')
% ylim([-pi pi])



figure
hold on
plot(time,ePitch)
plot(time,eRoll,'c')
plot(time,eYaw,'r','Linewidth',2)
legend('ePitch','eRoll','eYaw')



figure
hold on
% plot(time,throttle1)
plot(time,upitch,'k')
plot(time,uroll,'c')
plot(time,uyaw,'r','Linewidth',2)
legend('uPitch','uRoll','uYaw')


% Y1=fft(pitch_ref-mean(pitch_ref));
% Y2=2*abs(Y1(1:end/2))/length(Y1);
% 
% figure
% plot(linspace(0,100,length(Y2)),Y2)
% xlim([0 1.5])
% 
% 
figure
hold on
plotyy(time,throttle,time,Altitude)
% plot(Altitude,'r')
legend('throttle','Altitude')

figure
plot(time,d_Altitude)
title('Vertical Velocity')
