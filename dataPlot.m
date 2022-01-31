%T=importdata('file.csv');
%T.data
F=csvread('Torque.csv');
P=csvread('Position.csv');
V=csvread('Velocity.csv');
sT=0.01;
t=(0:0.01:sT*length(F)-sT)';
Power=-1*F(:,1).*V(:,1);
E=cumtrapz(Power);

figure(1)
%figure(2)
subplot(4,1,1)
plot(t,P(:,1))
grid on
%xlabel('Time(sec)')
ylabel('Position(mm)')
title('X axis')
%figure()
subplot(4,1,2)
plot(t,V(:,1))
grid on
%xlabel('Time(sec)')
ylabel('Velocity(mm/s)')
%title('X axis')

subplot(4,1,3)
plot(t,F(:,1))
grid on
%xlabel('Time(sec)')
ylabel('Force(N)')
%title('X')



%figure(4)
subplot(4,1,4)
plot(t,E(:,1))
grid on
xlabel('Time(sec)')
ylabel('Energy(N.mm)')
%title('X')



