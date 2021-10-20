clear all 
close all 

% load Motv_data0.mat
% load Motv_data11.mat
 %  load Motv_data1.mat
%load Motv_data2.mat
% load Motv_data3.mat
%  load Motv_data4.mat
% load Motv_data5.mat
% load Motv_data6.mat
% load Motv_data7.mat
% load Motv_data8.mat
load Motv_data9.mat

My_PI = 3.14159;
Rad2Deg = 180 / My_PI; 

T = MtvData(:,1);
Dlt_alp = MtvData(:,2);  Dlt_bta = MtvData(:,3);    Dlt_mu = MtvData(:,4); %%µÈÐ§¶æÆ«
alpha = MtvData(:,5);   beta = MtvData(:,6);   mu = MtvData(:,7);   %%×ËÌ¬½Ç
q = MtvData(:,8);   r = MtvData(:,9);   p = MtvData(:,10);   %%×ËÌ¬½ÇËÙ¶È

figure(1)
set(gcf,'windowstyle','normal');
set(gcf,'position',[550,100,600,450]);
plot(T,Dlt_alp,'-k','LineWidth',2); 
hold on 
grid on
plot(T,alpha*Rad2Deg,'--r','LineWidth',2); 
plot(T,q*Rad2Deg,':b','LineWidth',2); 
axis([0 50 -20.5 20.5])
xlabel('Time (s)','FontSize',13);
ylabel('Angle (deg)','FontSize',13);
h1=legend('Delta','alpha','q','Location','North');
set(h1,'Orientation','horizon')
set(h1,'box','off');
% title('Dlt_alp','FontSize',13);
set(gca,'FontSize',13);

% figure(2)
% plot(T,alpha*Rad2Deg,'-r','LineWidth',2); 
% xlabel('Time (s)','FontSize',13);
% ylabel('alpha (deg)','FontSize',13);
% % 
% % title('alpha','FontSize',13);
% set(gca,'FontSize',13);
% 
% figure(3)
% plot(T,q*Rad2Deg*50,'-r','LineWidth',2); 
% xlabel('Time (s)','FontSize',13);
% ylabel('q (deg/s)','FontSize',13);
% % axis([0 100 -20 5])
% % title('q','FontSize',13);
% set(gca,'FontSize',13);
