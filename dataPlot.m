%T=importdata('file.csv');
%T.data
clc

close all
clear all
F=csvread('Torque.csv');
P=csvread('Position.csv');
V=csvread('Velocity.csv');
sT=csvread('sampling.csv');
al=csvread('Alpha.csv');
Eob=csvread('Energy.csv');
khat2=csvread('khat.csv');
Fe=csvread('ControlForce.csv');


%sT=0.001;
%t=(0:sT:sT*length(F)-sT)';
%Power=-1*F(:,1).*V(:,1)*sT;
%E=cumtrapz(Power);

for i=1:length(V)
    if i==1
        E(i)=-1*F(i,1)*V(i,1)*sT(i);
        t(i)=sT(i);
    else
       E(i)=-1*F(i,1)*V(i,1)*sT(i)+E(i-1); 
       t(i)=t(i-1)+sT(i);
    end
end

av=0;
for i=1:length(F)
    av=av+F(i);
end
for i=1:length(F)-1
    derf(i)=(F(i+1)-F(i))/sT(i+1);
end
max(derf)/max(abs(F))
ra=find(Fe);%1050:1700;
av=av/length(Fe);
RMSE=sqrt(sum((F(ra)-Fe(ra)).^2)/length(ra));
NRMSE=RMSE/max(abs(F(ra)))
%Q=sqrt(sum(N.^2));
% for i=1:length(t)/2
%     if i==1
%         E2(i)=-1*F(i,1)*V(i,1)*sT*2;
%     else
%        E2(i)=-1*F(i*2,1)*V(i*2,1)*sT*2+E2(i-1); 
%     end
% end        
figure_handle = figure;
figure(1)
%figure(2)
subplot(5,1,1)
plot(t,P(:,1))
grid on
%xlabel('Time(sec)')
ylabel('Position(mm)')
title('X axis')
%figure()
subplot(5,1,2)
plot(t,V(:,1))
grid on
%xlabel('Time(sec)')
ylabel('Velocity(mm/s)')
%title('X axis')
subplot(5,1,3)
plot(t,al(:,1))
grid on
%xlabel('Time(sec)')
ylabel('Alpha)')

subplot(5,1,4)
plot(t,F(:,1))
grid on
%xlabel('Time(sec)')
ylabel('Force(N)')
%title('X')


EE=E';
%figure(4)
subplot(5,1,5)
plot(t,Eob)
grid on
hold on
plot(t,0.5*P.^2.*khat2)
xlabel('Time(sec)')
ylabel('Energy(N.mm)')
legend('actual','spring')
%title('X')

% figure()
% plot(t,Eob)
% 
% hold on
% plot(t,EE)

% figure()
% %figure(2)
% subplot(3,1,1)
% plot(t,P(:,1))
% grid on
% %xlabel('Time(sec)')
% ylabel('Position(mm)')
% title('X axis')
% %figure()
% subplot(3,1,2)
% plot(t,V(:,1))
% grid on
% %xlabel('Time(sec)')
% ylabel('Velocity(mm/s)')
% %title('X axis')

% EE=E';
% %figure(4)
% %subplot(3,1,3)
% plot(t,Eob)
% hold on
% plot(t,0.5*P.^2.*khat2)
% grid on
% xlabel('Time(sec)')
% ylabel('Energy(N.mm)')
%title('X')
all_ha = findobj( figure_handle, 'type', 'axes', 'tag', '' );
linkaxes( all_ha, 'x' );
% figure();
% plot(khat2)
