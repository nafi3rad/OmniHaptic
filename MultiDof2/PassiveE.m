function [c,ceq] = PassiveE(x)
velo=[50;1];
dEenergy=0.1;
B=[velo(1)^2;velo(1)*velo(2);velo(1)*velo(2);velo(2)^2];
c = B'*[x(1); x(2);x(3);x(4)] - dEenergy;
ceq = [];