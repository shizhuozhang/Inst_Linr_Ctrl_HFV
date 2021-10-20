clear all;
close all;

My_PI = 3.14159;
Rad2Deg = 180 / My_PI; 

load NNadpData_b0_off_pid.mat;
T = NNadpData(:,1);
e_idtf1 = NNadpData(:,2); eI_SldSum1 = NNadpData(:,12); 
% Dlt_a1 = NNadpData(:,13);  e_alp1 = NNadpData(:,14);  eC_SldSum1 = NNadpData(:,19);  

load NNadpData_b15_off_pid.mat;
e_idtf2 = NNadpData(:,2); eI_SldSum2 = NNadpData(:,12); 
% Dlt_a2 = NNadpData(:,13);  e_alp2 = NNadpData(:,14);  eC_SldSum2 = NNadpData(:,19); 

load NNadpData_b0_on1_pid.mat;
e_idtf3 = NNadpData(:,2); eI_SldSum3 = NNadpData(:,12); yk_now  = NNadpData(:,11); 
Dlt_a3 = NNadpData(:,13);  e_alp3 = NNadpData(:,14);  eC_SldSum3 = NNadpData(:,19); 

load NNadpData_b10_on_pid.mat;
e_idtf4 = NNadpData(:,2); eI_SldSum4 = NNadpData(:,12); 
Dlt_a4 = NNadpData(:,13);  e_alp4 = NNadpData(:,14);  eC_SldSum4 = NNadpData(:,19); 

load NNadpData_b15_on1_pid.mat;
e_idtf5 = NNadpData(:,2); eI_SldSum5 = NNadpData(:,12); 
Dlt_a5 = NNadpData(:,13);  e_alp5 = NNadpData(:,14);  eC_SldSum5 = NNadpData(:,19); 



figure(1);
set(gca,'FontSize',13);
plot(T,e_idtf3,'-r','LineWidth',2); 
% hold on 
% plot(T,e_idtf3,'--b','LineWidth',2); 
axis([0 50 -15 20]);
grid on;
xlabel('Time /sec','FontSize',13);
ylabel('e_p (deg)','FontSize',13);
% h1=legend('alp_{Exp}','alp_{Pre}','Location','NorthEast');
% set(h1,'box','off');
% title('Dlt_alp','FontSize',13);

figure(2);
set(gca,'FontSize',13);
plot(T,eI_SldSum1,'-r','LineWidth',2); 
hold on 
plot(T,eI_SldSum3,'--b','LineWidth',2); 
xlabel('Time (s)','FontSize',13);
ylabel('eI-SldSum (deg)','FontSize',13);


figure(3);
set(gca,'FontSize',13);
plot(T,yk_now,'-r','LineWidth',2); 
hold on 
plot(T,yk_now -e_idtf3,'-.b','LineWidth',2); 
grid on;
axis([0 50 -10 25]);
xlabel('Time (s)','FontSize',13);
ylabel('Alpha ','FontSize',13);
h1=legend('alp_{act}','alp_{Pre}','Location','NorthEast');
set(h1,'box','off');
% title('Dlt_alp','FontSize',13);
%% 不同方法的俯仰通道误差曲线对比
% figure(1);
% plot(T,Eror_alp_PPF*2*Rad2Deg,'-r','LineWidth',2); 
% hold on
% grid on
% plot(T,Ero_alp_SG*2*Rad2Deg,':m','LineWidth',2); 
% plot(T,Ero_alp_CNC*2*Rad2Deg,'--b','LineWidth',2); 
% plot(T,PPF_L*2*Rad2Deg,'-.k','LineWidth',2);
% plot(T,PPF_U*2*Rad2Deg,'-.k','LineWidth',2);
% 
% xlabel('Time (s)','FontSize',13);
% ylabel('Error_{\alpha} (deg)','FontSize',13);
% axis([0 100 -3.5 2])
% h1=legend('NPPSMC','SSMC','CNC','Location','NorthEast');
% set(h1,'box','off');
% set(gcf,'windowstyle','normal');
% set(gcf,'position',[550,100,800,500]);
% set(gca,'FontSize',13);
% magnify   %%局部放大镜


% figure(2);
% set(gcf,'windowstyle','normal');
% set(gcf,'position',[550,100,1000,400]);
% 
% subplot(1,3,1)
% plot(T,Dlt_e_PPF,'-r','LineWidth',2); 
% axis([0 100 -20 5])
% xlabel('Time (s)','FontSize',13);
% ylabel('Delta-e (deg)','FontSize',13);
% title('Dlt-e-NPPSMC','FontSize',13);
% set(gca,'FontSize',13);
% subplot(1,3,2)
% plot(T,Dlt_e_SG,':m','LineWidth',2); 
% axis([0 100 -20 5])
% xlabel('Time (s)','FontSize',13);
% ylabel('Delta-e (deg)','FontSize',13);
% title('Dlt-e-SSMC','FontSize',13);
% set(gca,'FontSize',13);
% 
% subplot(1,3,3)
% plot(T,Dlt_e_CNC,'--b','LineWidth',2); 
% axis([0 100 -20 5])
% xlabel('Time (s)','FontSize',13);
% ylabel('Delta-e (deg)','FontSize',13);
% title('Dlt-e-CNC','FontSize',13);
% set(gca,'FontSize',13);
% 
% 
% figure(3);
% set(gcf,'windowstyle','normal');
% set(gcf,'position',[550,100,600,400]);
% set(gca,'FontSize',13);
% plot(T,ErorV,'-.r','LineWidth',2);
% grid on
% xlabel('Time (s)','FontSize',13);
% ylabel('ErrorV (deg)','FontSize',13);
% axis([0 100 -1.5 0.5])
% 
% figure(4);
% set(gcf,'windowstyle','normal');
% set(gcf,'position',[550,100,600,400]);
% set(gca,'FontSize',13);
% plot(T,Cmd_alp*Rad2Deg,'-.r','LineWidth',2);
% hold on
% grid on
% plot(T,(Cmd_alp + 2*Eror_alp_PPF)*Rad2Deg,':b','LineWidth',2); 
% h1=legend('\alpha_{cmd}','\alpha_{act}','Location','NorthEast');
% set(h1,'box','off');
% xlabel('Time (s)','FontSize',13);
% ylabel('alpha (deg)','FontSize',13);
% axis([0 100 0 20])
% magnify   %%局部放大镜
