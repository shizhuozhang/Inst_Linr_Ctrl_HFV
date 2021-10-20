clear all;
close all;

My_PI = 3.14159;
Rad2Deg = 180 / My_PI; 

 %%校正网络1级控制器
load CtrlData_1_0_1.mat;
Dlt_e0 = CtrlData(1:10000,11); e_alp0 = CtrlData(1:10000,14); % Ero_qw1 = CtrlData(:,17);   

%%普通PID控制
load NNadpData_b0_off_pid.mat;  
T = NNadpData(:,1);  
e_alp1 = NNadpData(:,14);  eC_SldSum1 = NNadpData(:,19);  

%%自适应pid
load NNadpData_b0_on_adp1.mat;   
e_idtf2 = NNadpData(:,2); eI_SldSum2 = NNadpData(:,12); yk_now  = NNadpData(:,11); 
Dlt_a2 = NNadpData(:,13);  
e_alp2 = NNadpData(:,14);  eC_SldSum2 = NNadpData(:,19); 
a1 = NNadpData(:,3);  a2 = NNadpData(:,4); b1 = NNadpData(:,5); b2 = NNadpData(:,6); bias = NNadpData(:,7);
k1 = NNadpData(:,15); k2 = NNadpData(:,16); k3 = NNadpData(:,17); 

load FltData_b0_on_adp1.mat
Cmd_alp = FltData2(:,18);  Dlt_e2 = FltData2(:,21); 

% 不同方法的俯仰通道误差曲线对比
figure(1);
plot(T,-e_alp2*4,'-r','LineWidth',2); 
hold on
grid on
plot(T,-e_alp1*4,':b','LineWidth',2); 
plot(T,e_alp0*Rad2Deg,'--k','LineWidth',2); 
xlabel('Time (s)','FontSize',13);
ylabel('Error_{\alpha} (deg)','FontSize',13);
axis([0 50 -0.5 0.8])
h1=legend('GMVAC','PID','CNC','Location','NorthEast');
set(h1,'box','off');
set(gcf,'windowstyle','normal');
set(gcf,'position',[550,100,600,400]);
set(gca,'FontSize',13);
% magnify   %%局部放大镜

figure(2);
set(gca,'FontSize',13);
plot(T,eC_SldSum2*4/10,'-r','LineWidth',2); 
hold on 
plot(T,eC_SldSum1*4/10,'--b','LineWidth',2); 
xlabel('Time (s)','FontSize',13);
ylabel('C_e (deg)','FontSize',13);
grid on;
axis([0 50 -0.01 0.5])
h1=legend('GMVAC','PID','Location','NorthEast');
set(h1,'box','off');
set(gcf,'windowstyle','normal');
set(gcf,'position',[550,100,600,400]);

% figure(3);
% set(gcf,'windowstyle','normal');
% set(gcf,'position',[550,100,1100,400]);
% 
% subplot(1,3,1)
% plot(T,k1,'-r','LineWidth',2); 
%  axis([0 50 -60.01 -59.8]);
%  grid on;
% xlabel('Time (s)','FontSize',13);
% ylabel('kp','FontSize',13);
% set(gca,'FontSize',13);
% 
% subplot(1,3,2)
% plot(T,k2,'--r','LineWidth',2); 
%  axis([0 50 0.5 2.6]);
%  grid on;
% xlabel('Time (s)','FontSize',13);
% ylabel('Ti','FontSize',13);
% set(gca,'FontSize',13);
% 
% subplot(1,3,3)
% plot(T,0.2+(k3-0.2)*3,'-.r','LineWidth',2); 
%  axis([0 50 0.1998 0.202]);
%  grid on;
% xlabel('Time (s)','FontSize',13);
% ylabel('Td','FontSize',13);
% set(gca,'FontSize',13);
% 
% figure(4);
% set(gcf,'windowstyle','normal');
% set(gcf,'position',[550,100,600,400]);
% set(gca,'FontSize',13);
% plot(T,Dlt_e2,'--r','LineWidth',2);
% grid on
% xlabel('Time (s)','FontSize',13);
% ylabel('Delta-e (deg)','FontSize',13);
% axis([0 50 -20 20])
% 
% figure(5);
% set(gcf,'windowstyle','normal');
% set(gcf,'position',[550,100,600,450]);
% set(gca,'FontSize',13);
% plot(T,e_idtf2,'-r','LineWidth',2);
% grid on
% xlabel('Time (s)','FontSize',13);
% ylabel('e_p (deg)','FontSize',13);
% axis([0 50 -6 6])
% 
% figure(6);
% set(gcf,'windowstyle','normal');
% set(gcf,'position',[550,100,600,450]);
% set(gca,'FontSize',13);
% plot(T,eI_SldSum2/10,'-r','LineWidth',2);
% grid on
% xlabel('Time (s)','FontSize',13);
% ylabel('C_{ep} (deg)','FontSize',13);
% axis([0 50 0 0.1])
% 
% figure(7);
% set(gca,'FontSize',13);
% plot(T,a1,'-r','LineWidth',2); 
% hold on 
% grid on
% plot(T,a2,'-.b','LineWidth',2); 
% plot(T,b2,'--g','LineWidth',2); 
% plot(T,b1,':m','LineWidth',2); 
% xlabel('Time (s)','FontSize',13);
% ylabel('AB ','FontSize',13);
% h1=legend('a1','a2','b1','b2','Location','NorthEast');
% set(h1,'box','off');
% 
% figure(8);
% set(gcf,'windowstyle','normal');
% set(gcf,'position',[550,100,600,400]);
% set(gca,'FontSize',13);
% plot(T,bias,'--r','LineWidth',2);
% grid on
% xlabel('Time (s)','FontSize',13);
% ylabel('bias (deg)','FontSize',13);
% axis([0 50 -5 25])
