%% 
clear all
addpath('/home/hwadong/catkin_ws/src/vsds_orient/data/quat_utils') ;


x=load('x_rob1.txt') ; 
vsds_name = 'JShape' ; 
 vsds_name = 'Trapezoid' ; 
%  vsds_name = 'Worm' ; 
direc=strcat('/home/hwadong/catkin_ws/src/vsds_orient/config/', vsds_name,'/x_rec.txt') ;
x_des=load(direc) ;

for i=1:length(x)
x_ts(:,i)=quat_log(array2quat(x(i,4:end)'),array2quat([1 0 0 0]')) ;
end



dt=1/500 ;

T=linspace(0,length(x)*dt,length(x)) ;
T_des=linspace(0,length(x_des)*dt,length(x_des)) ;

% plot(T,x(:,4:end)) ;
% hold on
% plot(T_des,x_des(:,4:end),'--') ;
% hold on

figure
% xl = xlabel('Time');yl = ylabel('s');
set(gca,'fontsize',18,'LineWidth',1);
% set([xl yl],'interpreter','latex','fontsize',18);
plot3(x_ts(1,:),x_ts(2,:),x_ts(3,:),'-k','LineWidth',2)
hold on 
plot3(x_des(1,:),x_des(2,:),x_des(3,:),'--r','LineWidth',2)
title('Tangent Space','interpreter','latex','fontsize',16,'fontweight','normal') ;
box on;grid on;hold on;hold on
% legend({'Actual','Desired'},'interpreter','latex','fontsize',16,'fontweight','normal') ;



% Q=load('Q_des_test.mat') ;
% figure
% plot(Q.Q_all.T,Q.Q_all.Q_arr) ;