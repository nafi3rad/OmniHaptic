close all
clear all
V=readmatrix('Velocity.csv');
P=readmatrix('position.csv');
F=readmatrix('torque.csv');
al=readmatrix('Alpha.csv');
E=readmatrix('energy.csv');
Fe=readmatrix('ControlForce.csv');
Khat=readmatrix('khat.csv');
Eref=readmatrix('sampling.csv');
plot(E);hold on;plot(Eref)
legend('E','Eref')
figure()
plot(Khat)
figure()
plot(P(:,1),P(:,2))
figure();plot(F);hold on;plot(Fe)
%myVideo = VideoWriter('myVideoFile'); %open video file
%myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
%open(myVideo)
g=1;
% for i=1:length(Fe)-1
%     deltae(i)=F(i,1)*V(i+1,1)+F(i,2)*V(i+1,2)+F(i,3)*V(i+1,3);
% end
magFe=sqrt(Fe(:,1).^2+Fe(:,2).^2);
magF=sqrt(F(:,1).^2+F(:,2).^2);
magV=sqrt(V(:,1).^2+V(:,2).^2);
Emag=mean((magFe-magF)/magFe,'All')*100
%aFe=Fe(:,1)./Fe(:,2);
noZero=find(magF);

for i=1:length(noZero)
    if magV(i)==0
        magV(i)=0.000001;
    end
        if magFe(i)==0
        magFe(i)=0.000001;
    end
    aFe(i)=(Fe(noZero(i),1)*V(noZero(i),1)+Fe(noZero(i),2)*V(noZero(i),2)+Fe(noZero(i),3)*V(noZero(i),3))/(magFe(noZero(i))*magV(noZero(i)));
    aF(i)=(F(noZero(i),1)*V(noZero(i),1)+F(noZero(i),2)*V(noZero(i),2)+F(noZero(i),3)*V(noZero(i),3))/(magF(noZero(i))*magV(noZero(i)));
end
gg=10;
Edir=mean((-acos(aFe)+acos(aF))/(acos(aFe)),'all')*100
figure()
for k=950:15:1200
    
%     quiver(P(k,1)/gg,P(k,2)/gg,V(k,1)/100,V(k,2)/100,'b');
%     hold on
    quiver(P(k,1)/gg,P(k,2)/gg,-F(k,1),-F(k,2),'r-','linewidth',2.5);
    hold on
    quiver(P(k,1)/gg,P(k,2)/gg,-Fe(k,1),-Fe(k,2),'k','linewidth',1.5);
    %hold on
    %quiver(0,0,P(k,1)/50,P(k,2)/50,'--');
    hold on
    plot(P(k,1)/gg,P(k,2)/gg,'o','MarkerEdgeColor','blue','MarkerFaceColor',[1 .6 .6])%,'-s','MarkerSize',10,'MarkerEdgeColor','blue','MarkerFaceColor',[1 .6 .6])
    set(gca,'fontsize',12,'FontName','Times')
    xlabel('$p_x$ (cm)', 'Interpreter','latex')
    ylabel('$p_y$ (cm)', 'Interpreter','latex')
    legend('-$F_c$ (N)','-$F_e$ (N)','p (cm)','Interpreter','latex')%,'X')

    grid on
    
    %set(gca,'XLim',[-10/g,10/g],'YLim',[-10/g,10/g])
    %drawnow
    %pause(0.01) %Pause and grab frame
    %frame = getframe(gcf); %get frame
    %writeVideo(myVideo, frame);
end
%close(myVideo)

