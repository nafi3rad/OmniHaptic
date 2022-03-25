%estimate by RLS using the saved data
clc
clear all
close all
F=csvread('Torque.csv');
P=csvread('Position.csv');
V=csvread('Velocity.csv');
sT=csvread('sampling.csv');
al=csvread('Alpha.csv');
Eob=csvread('Energy.csv');
khat2=csvread('khat.csv');
z=0.5*P.^2;
y=Eob;
a=1;
b=length(P);
for i=a:b
    if i==a

        t(i)=sT(i);
        khat(a)=0.1;
        p(a)=1;
    else
       [khat(i),p(i)]=myRLS(z(i),y(i),p(i-1),khat(i-1),0.9);
       t(i)=t(i-1)+sT(i);
    end
end

plot(khat)
hold on
plot(khat2)
figure()
plot(Eob)