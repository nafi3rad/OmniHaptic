velo=[50;10];

%A=[velo',0,0;0,0,velo'];
A=-[velo(1)^2,velo(1)*velo(2),velo(1)*velo(2),velo(2)^2];
b=-0.01;
fun=@(x)((x(1)*velo(1)+x(2)*velo(2))^2+(x(3)*velo(1)+x(4)*velo(2))^2);
x0=[1;0;0;0]';
Aeq = [];
beq = [];
 %fmincon(fun,x0,A,b,Aeq,beq)
%cc=[0.5316,0.2265;0.0935,0.1094]*0.0001;
%(cc*velo)'*velo