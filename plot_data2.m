close all
clear all
fname = 'single_bot_data.json'; 
fid = fopen(fname); 
raw = fread(fid,inf); 
str = char(raw'); 
fclose(fid); 
val = jsondecode(str);



times = (1:1400)/100;
body_pos = val.body_pos(1:1400,:);



figure
subplot(2,2,1);
hold on
plot(times,body_pos(:,2))
plot(times,body_pos(:,3))
xlabel('time (s)')
ylabel('position (m)')
legend('y','z')
title('Position of the base link')
grid on
hold off 


body_vel = val.body_vel(1:1400,:);

subplot(2,2,2);
hold on
plot(times,body_vel(:,2))
plot([0,14],[0.5,0.5],'-')
plot(times,body_vel(:,3))
xlabel('time (s)')
ylabel('velocity (m/s)')
legend('y','reference','z')
title('Velocity of the base link')
grid on
hold off 


body_ori = val.body_ori(1:1400,:);
body_ori = unwrap(quat2eul(body_ori,'XYZ')) ;


subplot(2,2,3);
hold on
plot(times,body_ori(:,3))
xlabel('time (s)')
ylabel('Pitch angle (rad)')
title('Orientation of the base link')
grid on
hold off 



body_ori_vel = val.body_ori_vel(1:1400,:);

subplot(2,2,4);
hold on
plot(times,body_ori_vel(:,1))
plot([0,14],[0,0],'-')
xlabel('time (s)')
ylabel('Angular velocity (rad/s)')
legend('pitch','reference')
title('Angular velocity of the base link')
grid on
hold off 


joint_pos = val.joint_pos(1:1400,:);

join_vel = val.joint_vel(1:1400,:);

join_eff = val.joint_eff(1:1400,:);

figure
subplot(2,2,1);
hold on
plot(times,joint_pos(:,2))
plot(times,join_vel(:,2))
xlabel('time (s)')
legend('position (rad)' ,  'velocity (rad/s)')
title('Hip actuator joint states')
grid on
hold off 


subplot(2,2,2);
hold on
plot(times,joint_pos(:,3))
plot(times,join_vel(:,3))
xlabel('time (s)')
legend('position (m)' ,  'velocity (m/s)')
title('Leg actuator joint states')
grid on
hold off 



subplot(2,2,3);
hold on
plot(times,join_eff(:,2))
xlabel('time (s)')
legend( 'effort (Nm)')
title('Hip actuator joint effort')
grid on
hold off 


subplot(2,2,4);
hold on
plot(times,join_eff(:,3))
xlabel('time (s)')
ylabel('effort (N)')
title('Leg actuator joint effort')
grid on
hold off 