clear all;
close all;

% load CtrlData_0_0_0.mat;
   load CtrlData_0_1_0.mat;
% load CtrlData_1_1_1.mat;
%load CtrlData_2_1_1.mat;

My_PI = 3.14159;
Rad2Deg = 180 / My_PI; 

% CtrlBuf = [(Time + h_step), ctrler_level, Bias_amp, Bias_case, Omega_1, CtrlSdt_B, CtrlSdt_P];%%全部的状态量1+1+1+1+3+6+12=25

T = CtrlData(:,1);  
ctrler_level= CtrlData(:,2);  Bias_amp = CtrlData(:,3);   Bias_case = CtrlData(:,4) ;   
q = CtrlData(:,5); r = CtrlData(:,6);   p = CtrlData(:,7);  
Cmd_alp = CtrlData(:,8);  Cmd_bet = CtrlData(:,9);    Cmd_mu = CtrlData(:,10); %构成指令矢量 CmdAng = [Cmd_alp ,Cmd_bet, Cmd_mu];
Dlt_e = CtrlData(:,11);  Dlt_a = CtrlData(:,12);    Dlt_r = CtrlData(:,13); %构成实际舵偏矢量 DeltaA = [Dlt_a ,Dlt_e, Dlt_r];

Ero_alp = CtrlData(:,14);   Ero_bta = CtrlData(:,15);  Ero_mu = CtrlData(:,16);   %构成控制误差矢量 Eror_ang = [Ero_pit ,Ero_yaw, Ero_rol];
Ero_qw = CtrlData(:,17);   Ero_rw = CtrlData(:,18);  Ero_pw = CtrlData(:,19);  %构成角速度控制误差矢量 Eror_omg = [Ero_qw ,Ero_rw, Ero_pw];
Acc_pit = CtrlData(:,20);  Acc_yaw = CtrlData(:,21);  Acc_rol = CtrlData(:,22);  %构成角加速度矢量 Accelrt = [Acc_pit ,Acc_yaw, Acc_rol];
Dlt_alp = CtrlData(:,23);  Dlt_bet = CtrlData(:,24);  Dlt_mu = CtrlData(:,25);  %构成等效舵偏矢量 DeltaE = [Dlt_alp ,Dlt_bet, Dlt_mu];
rankS = CtrlData(:,26); rankD = CtrlData(:,27);Koutp = CtrlData(:,28);
%% 绘图
figure(20);
plot(T,rankS,'*r','LineWidth',2);    
figure(21)
plot(T,rankD,'*r','LineWidth',2);   
figure(22);
plot(T,Koutp,'-r','LineWidth',2);    
% figure(16);
% plot(T,Cmd_alp*Rad2Deg,'-r','LineWidth',2);    
% 
% grid on
% xlabel('Time (s)','FontSize',12);
% ylabel('alp (deg)','FontSize',12);
% 
% figure(17);
% plot(T,Cmd_bet*Rad2Deg,'-r','LineWidth',2);   
% grid on
% xlabel('Time (s)','FontSize',12);
% ylabel('beta (deg)','FontSize',12);
% 
% figure(18);
% plot(T,Cmd_mu*Rad2Deg,'-r','LineWidth',2);   
% grid on
%  grid on
% xlabel('Time (s)','FontSize',12);
% ylabel('mu (deg)','FontSize',12);
% % 
figure(19);
plot(T,Dlt_a,'-.r','LineWidth',2); 
hold on;
% plot(T,Dlt_e,'-.b','LineWidth',2); 
% plot(T,Dlt_r,'-.g','LineWidth',2); 
% grid on
% xlabel('Time (s)','FontSize',12);
% ylabel('Delta (deg)','FontSize',12);

%% 俯仰通道
figure(1);
plot(T,Ero_alp*Rad2Deg,'-b','LineWidth',2); 
grid on
xlabel('Time (s)','FontSize',12);
ylabel('Ero_{alp} (deg)','FontSize',12);
% 
% figure(4);
% plot(T,q*Rad2Deg,'-r','LineWidth',2); 
% grid on
% xlabel('Time (s)','FontSize',12);
% ylabel('q (deg)','FontSize',12);
% 
% figure(7);
% plot(T,Ero_qw*Rad2Deg,'-r','LineWidth',2); 
% grid on
% xlabel('Time (s)','FontSize',12);
% ylabel('Ero_{qw} (deg)','FontSize',12);
% 
% figure(10);
% plot(T,Acc_pit*Rad2Deg,'-r','LineWidth',2); 
% grid on
% xlabel('Time (s)','FontSize',12);
% ylabel('Acc_{pit} (deg)','FontSize',12);
% 
% figure(13);
% plot(T,Dlt_alp,'-.r','LineWidth',2); 
%  grid on
% xlabel('Time (s)','FontSize',12);
% ylabel('Dlt_{alp} (deg)','FontSize',12);
 
% %% 偏航通道
% figure(2);
% plot(T,Ero_bta*Rad2Deg,'-r','LineWidth',2);   
% grid on
% xlabel('Time (s)','FontSize',12);
% ylabel('Ero_bta (deg)','FontSize',12);
% 
% figure(5);
% plot(T,r*Rad2Deg,'-r','LineWidth',2);  
% grid on
% xlabel('Time (s)','FontSize',12);
% ylabel('r (deg)','FontSize',12);
% 
% figure(8);
% plot(T,Ero_rw*Rad2Deg,'-r','LineWidth',2);  
% grid on
% xlabel('Time (s)','FontSize',12);
% ylabel('Ero_rw (deg)','FontSize',12);
% 
% figure(11);
% plot(T,Acc_yaw*Rad2Deg,'-r','LineWidth',2);  
% grid on
% xlabel('Time (s)','FontSize',12);
% ylabel('Acc_yaw (deg)','FontSize',12);
% 
% figure(14);
% plot(T,Dlt_bet,'-.r','LineWidth',2); 
% grid on
% xlabel('Time (s)','FontSize',12);
% ylabel('Dlt_bet (deg)','FontSize',12);
%  
%% 滚转通道
% figure(3);
% plot(T,Ero_mu*Rad2Deg,'-r','LineWidth',2); 
% grid on
% xlabel('Time (s)','FontSize',12);
% ylabel('Ero_mu (deg)','FontSize',12);
% 
% figure(6);
% plot(T,p*Rad2Deg,'-r','LineWidth',2);
% grid on
% xlabel('Time (s)','FontSize',12);
% ylabel('p (deg)','FontSize',12);
% 
% figure(9);
% plot(T,Ero_pw*Rad2Deg,'-r','LineWidth',2);
% grid on
% xlabel('Time (s)','FontSize',12);
% ylabel('Ero_pw (deg)','FontSize',12);
% 
% figure(12);
% plot(T,Acc_rol*Rad2Deg,'-r','LineWidth',2); 
% grid on
% xlabel('Time (s)','FontSize',12);
% ylabel('Acc_rol (deg)','FontSize',12);
% 
% figure(15);
% plot(T,Dlt_mu,'-.r','LineWidth',2); 
% grid on
% xlabel('Time (s)','FontSize',12);
% ylabel('Dlt_mu (deg)','FontSize',12);

%% 另一种画图
% figure(1);
% plot(T,Cmd_alp*Rad2Deg,'-r','LineWidth',2);        
% hold on
% plot(T,Cmd_bet*Rad2Deg,'--m','LineWidth',2);
% plot(T,Cmd_mu*Rad2Deg,':b','LineWidth',2); 
% grid on
% xlabel('Time (s)','FontSize',14);
% ylabel('Angles (deg)','FontSize',14);
% axis([0 100 -20 20])
% h1=legend('alpha','beta','mu','Location','SouthWest');
% set(h1,'box','off');
% set(gca,'FontSize',14);
% 
% figure(2);
% plot(T,Ero_alp*Rad2Deg,'-r','LineWidth',2);        
% hold on
% plot(T,Ero_bta*Rad2Deg,'--m','LineWidth',2);
% plot(T,Ero_mu*Rad2Deg,':b','LineWidth',2); 
% grid on
% xlabel('Time (s)','FontSize',14);
% ylabel('e-Angle (deg)','FontSize',14);
% axis([0 100 -0.5 1])
% h1=legend('e_{\alpha}','e_{\beta}','e_{\mu}','Location','SouthEast');
% set(h1,'box','off');
% set(gca,'FontSize',14);
% 
% figure(3);
% plot(T,q*Rad2Deg,'-r','LineWidth',2);  
% hold on;
% plot(T,r*Rad2Deg,'-.m','LineWidth',2);  
% plot(T,p*Rad2Deg,':b','LineWidth',2);  
% grid on;
% xlabel('Time (s)','FontSize',14);
% ylabel('Omega (deg/s)','FontSize',14);
% axis([0 100 -2 3])
% h1=legend('q','r','p','Location','SouthEast');
% set(h1,'box','off');
% set(gca,'FontSize',14);
% 
% figure(4);
% plot(T,Ero_qw*Rad2Deg,'-r','LineWidth',2);  
% hold on;
% plot(T,Ero_rw*Rad2Deg,'-.m','LineWidth',2);  
% plot(T,Ero_pw*Rad2Deg,':b','LineWidth',2);  
% grid on;
% xlabel('Time (s)','FontSize',14);
% ylabel('e-Omega (deg/s)','FontSize',14);
% axis([0 100 -0.3 0.2])
% set(gca, 'YTick', -0.3:0.1:0.2) %设置X坐标轴刻度数据点位置
% h1=legend('e_q','e_r','e_p','Location','SouthEast');
% set(h1,'box','off');
% set(gca,'FontSize',14);
% 
% figure(5);
% plot(T,Acc_pit*Rad2Deg,'-r','LineWidth',2);  
% hold on;
% plot(T,Acc_yaw*Rad2Deg,'-.m','LineWidth',2);  
% plot(T,Acc_rol*Rad2Deg,':b','LineWidth',2);  
% grid on;
% xlabel('Time (s)','FontSize',14);
% ylabel('Acc (deg/s/s)','FontSize',14);
% axis([0 100 -10 10])
% h1=legend('Acc_{\alpha}','Acc_{\beta}','Acc_{mu}','Location','NorthEast');
% set(h1,'box','off');
% set(gca,'FontSize',14);
% 
% figure(6);
% plot(T,Dlt_e,'-r','LineWidth',2);  
% hold on;
% plot(T,Dlt_a,'-.b','LineWidth',2);  
% plot(T,Dlt_r,':m','LineWidth',2);  
% grid on;
% xlabel('Time (s)','FontSize',14);
% ylabel('Delta (deg)','FontSize',14);
% axis([0 100 -20 5])
% h1=legend('Dlt-e','Dlt-a','Dlt-r','Location','SouthEast');
% set(h1,'box','off');
% set(gca,'FontSize',14);

% % figure(7);
% % plot(T,Dlt_alp,'-.k','LineWidth',2);  
% % grid on;
% % xlabel('Time (s)','FontSize',13);
% % ylabel('Dt-bta (deg)','FontSize',13);
% % axis([0 100 -0.8 0.8])
% % set(gca,'FontSize',13);

% % figure(8);
% % plot(T,Dlt_bet,'-.k','LineWidth',2);  
% % grid on;
% % xlabel('Time (s)','FontSize',13);
% % ylabel('Dt-bta (deg)','FontSize',13);
% % axis([0 100 -0.8 0.8])
% % set(gca,'FontSize',13);
% % 
% % figure(9);
% % plot(T,Dlt_mu,'--b','LineWidth',2);  
% % grid on;
% % xlabel('Time (s)','FontSize',13);
% % ylabel('Dt-mu (deg)','FontSize',13);
% % axis([0 100 -1 1])
% % set(gca,'FontSize',13);