close all
figure_handle=figure;
%figure_handle.Position = [300 0 400 600];
for i=1:length(Fe)
    t(i)=i*0.001;                                  
end
a=600;
b=1200;

figure(1)
subplot(3,1,1)

plot(t(a:b)-t(a),-Fe(a:b,1),'b','LineWidth',1)
hold on
plot(t(a:b)-t(a),-F(a:b,1),'r','LineWidth',1)
grid on
set(gca,'fontsize',12,'FontName','Times')
xlabel('time (s)', 'Interpreter','latex')
ylabel('$f_{x}$ (N)', 'Interpreter','latex')
legend('Environment','Adaptive', 'Interpreter','latex')
%ylim([-0.1,0.1])
subplot(3,1,2)

plot(t(a:b)-t(a),-Fe(a:b,2),'b','LineWidth',1)
hold on
plot(t(a:b)-t(a),-F(a:b,2),'r','LineWidth',1)
grid on
set(gca,'fontsize',12,'FontName','Times')
xlabel('time (s)', 'Interpreter','latex')
ylabel('$f_{y}$ (N)', 'Interpreter','latex')
%legend('Fc','Fe', 'Interpreter','latex')
%ylim([-2,9])


subplot(3,1,3)

plot(t(a:b)-t(a),magFe(a:b),'b','LineWidth',1)
grid on
hold on
plot(t(a:b)-t(a),magF(a:b),'r','LineWidth',1)
grid on
set(gca,'fontsize',12,'FontName','Times')
xlabel('time (s)', 'Interpreter','latex')
ylabel('$\left \|f \right \|_{2}$ (N)', 'Interpreter','latex')
% ylim([-0.002,0.3])
%legend('TDPA','Adaptive''Without delay')

figure()
subplot(2,1,1)
plot(t(a:b)-t(a),E(a:b)/1000,'b','LineWidth',1)
grid on
set(gca,'fontsize',12,'FontName','Times')
xlabel('time (s)', 'Interpreter','latex')
ylabel('Energy (N.mm)', 'Interpreter','latex')

subplot(2,1,2)
plot(t(a:b)-t(a),Khat(a:b,1)*1000,'r','LineWidth',1)
hold on
plot(t(a:b)-t(a),Khat(a:b,2)*1000,'r:','LineWidth',1)
hold on
plot(t(a:b)-t(a),Khat(a:b,3)*1000,'r--','LineWidth',1)
grid on
set(gca,'fontsize',12,'FontName','Times')
xlabel('time (s)', 'Interpreter','latex')
ylabel('$k$ (N/mm)', 'Interpreter','latex')
legend('$k_{xx}$ (N/mm)','$k_{yy}$ (N/mm)','$k_{xy}$ (N/mm)' ,'Interpreter','latex')