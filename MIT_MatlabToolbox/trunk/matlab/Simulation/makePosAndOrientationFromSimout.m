t = simout.Time;

data = simout.Data;
x = data(:,1);
y = data(:,2);
z = data(:,3);
yaw = data(:,4);
pitch = data(:,5);
roll = data(:,6);

close all;

figure;
plot(t,x, 'b-*');
hold on
plot(t,y,'r');
plot(t,z,'g');
legend('x','y','z')
title('Positions')

figure;
plot(t,yaw, 'b-*');
hold on
plot(t,pitch,'r');
plot(t,roll,'g');
legend('yaw','pitch','roll')
title('Orientations')