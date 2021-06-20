function animation()
global theta1 theta2 psi1 psi2 psi1dot psi2dot yr xw yw zw xb yb zb R r time T pitchset rollset theta1_bias theta2_bias
clc;
% Create Movie
% v = VideoWriter('x.avi');
% open(v);

s = linspace(0,2*pi,100);
sins = sin(s);
coss = cos(s);
n = 10;

for i = 1:n:length(theta1)-n
    subplot(221)
    % Ground
    X = [1 -1;1 -1];
    Y = [1 1;-1 -1];
    Z = [0 0;0 0];
    surf(X,Y,Z,'FaceColor',[0.9 0.9 0.9],'edgecolor','none'); hold on;
    % Body
    plot3([xw(i) xb(i)],[yw(i) yb(i)],[zw(i) zb(i)],'b','linewidth',2);%hold on;
    %plot3(xb(i),yb(i),zb(i),'r','markersize',10);
    % Wheel
    plot3(xw(i)+(R-r)*coss,yw(i)+(R-r)*sins*sin(theta2(i)),zw(i)+(R-r)*sins*cos(theta2(i)),'k','linewidth',2);
    plot3([xw(i) xw(i)+(R-r)*sin(psi1(i))],[yw(i) yw(i)+(R-r)*cos(psi1(i))*sin(theta2(i))],[zw(i) zw(i)+(R-r)*cos(psi1(i))*cos(theta2(i))],'r','linewidth',2);%hold on;
    % Label
    xlabel('$x$ [m]','interpreter','latex');
    ylabel('$y$ [m]','interpreter','latex');
    zlabel('$z$ [m]','interpreter','latex');
    set(gca,'Fontsize',15);
    axis([-1 1 -1 1 0 1]*0.5); % [-1 1 -0.5 1.5]*0.5
%     axis([-0.3 0.3 -0.3 0.3 0 0.4]);
    pbaspect([2 2 1])
    %axis equal;
    %axis square;
    grid on;
    %view(0,90);
    %view(40,30);
    %fmesh(z(4));
    hold off;
    
    subplot(222)
    plot([psi1(i) psi1(i+n)]*R*100,[psi2(i) psi2(i+n)]*r*100,'k','linewidth',1); hold on;
    xlabel('$x$ [cm]','interpreter','latex');
    ylabel('$y$ [cm]','interpreter','latex');
    pbaspect([1 1 1]);
    axis([-1 1 -1 1]*7);
    set(gca,'Fontsize',15);
    grid on;
    
    subplot(223)
    plot([time(i) time(i+n)],[theta1(i) theta1(i+n)]/pi*180,'b',[time(i) time(i+n)],[theta2(i) theta2(i+n)]/pi*180,'r','linewidth',1); hold on;
    xlabel('Time $t$ [s]','interpreter','latex');
    ylabel('Body Orientation [deg]','interpreter','latex');
    axis([0 T -3 6]);
    set(gca,'Fontsize',15);
    grid on;
    h1 = legend('$\theta_1$','$\theta_2$','Location','northeast');
    set(h1,'interpreter','latex');
    
    subplot(224)
%     plot([time(i) time(i+n)],[psi1(i) psi1(i+n)]*R*100,'b',[time(i) time(i+n)],[psi2(i) psi2(i+n)]*r*100,'r','linewidth',1); hold on;
%     xlabel('Time $t$ [s]','interpreter','latex');
%     ylabel('Wheel Position [cm]','interpreter','latex');
%     axis([0 T -2 2]);
%     set(gca,'Fontsize',15);
%     grid on;
%     h2 = legend('$p_x$','$p_y$','Location','southeast');
%     set(h2,'interpreter','latex');

%     plot([time(i) time(i+n)],[pitchset(i) pitchset(i+n)]/pi*180,'b',[time(i) time(i+n)],[rollset(i) rollset(i+n)]/pi*180,'r','linewidth',1); hold on;
%     xlabel('Time $t$ [s]','interpreter','latex');
%     ylabel('Reference [deg]','interpreter','latex');
%     axis([0 T -10 5]);
%     set(gca,'Fontsize',15);
%     grid on;
%     h2 = legend('$\theta_{1o}$','$\theta_{2o}$','Location','southeast');
%     set(h2,'interpreter','latex');
    
    plot([time(i) time(i+n)],[theta1_bias(i) theta1_bias(i+n)]/pi*180,'b',[time(i) time(i+n)],[theta2_bias(i) theta2_bias(i+n)]/pi*180,'r','linewidth',1); hold on;
    xlabel('Time $t$ [s]','interpreter','latex');
    ylabel('IMU Bias [deg]','interpreter','latex');
    axis([0 T -1 1]);
    set(gca,'Fontsize',15);
    grid on;
    h2 = legend('$\theta_{1o}$','$\theta_{2o}$','Location','southeast');
    set(h2,'interpreter','latex');
    
%     
%     hold off;
%     pause(0.01);
    drawnow;
    
%     frame = getframe(gcf);
%     writeVideo(v,frame);

end
end