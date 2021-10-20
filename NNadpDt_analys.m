
%%  NNadpDt_analys
% NNadpBuf =[(Time + h_step), NNpara, CtrlPara,0,0];
% NNpara(1,1) = e_idtf;
% A_B_bias = [a1, a2, b1, b2, bias];
% NNpara(1, 2:6) = A_B_bias;
% CtrlPara = [Dlt_a,e_now, K',omg];

% load NNadpData.mat;

% My_PI = 3.14159;
% Rad2Deg = 180 / My_PI; 

T = NNadpData(:,1);
e_idtf = NNadpData(:,2); a1 = NNadpData(:,3);  a2 = NNadpData(:,4); b1 = NNadpData(:,5); b2 = NNadpData(:,6); bias = NNadpData(:,7);
uk_lst1 = NNadpData(:,8);  uk_now = NNadpData(:,9);   yk_lst1 = NNadpData(:,10);  yk_now  = NNadpData(:,11);  eI_SldSum = NNadpData(:,12); 
Dlt_a = NNadpData(:,13);  e_alp = NNadpData(:,14);  
k1 = NNadpData(:,15); k2 = NNadpData(:,16); k3 = NNadpData(:,17); omg = NNadpData(:,18);eC_SldSum = NNadpData(:,19);  
grd_K1 = NNadpData(:,20); grd_K2 = NNadpData(:,21);   grd_K3 = NNadpData(:,22); 

figure(1);
set(gca,'FontSize',13);
plot(T,e_idtf,'-r','LineWidth',2); 
% hold on 
% plot(Ans1,'--b','LineWidth',2); 
axis([0 50 -1 1])
xlabel('Time /sec','FontSize',13);
ylabel('e_p (deg)','FontSize',13);
% h1=legend('alp_{Exp}','alp_{Pre}','Location','NorthEast');
% set(h1,'box','off');
% title('Dlt_alp','FontSize',13);

figure(2);
set(gca,'FontSize',13);
plot(T,yk_now,'-r','LineWidth',2); 
hold on 
plot(T,yk_now -e_idtf,'-.b','LineWidth',2); 
xlabel('Time /sec','FontSize',13);
ylabel('Alpha ','FontSize',13);
h1=legend('alp_{act}','alp_{Pre}','Location','NorthEast');
set(h1,'box','off');
% title('Dlt_alp','FontSize',13);

figure(20);
set(gca,'FontSize',13);
plot(T,a1,'-r','LineWidth',2); 
hold on 
plot(T,a2,'-.b','LineWidth',2); 
plot(T,b2,'--g','LineWidth',2); 
plot(T,b1,':m','LineWidth',2); 
axis([0 50 -3 4])
xlabel('Time /sec','FontSize',13);
ylabel('AB ','FontSize',13);
h1=legend('a1','a2','b1','b2','Location','SouthWest');
set(h1,'box','off');

figure(3);
set(gca,'FontSize',13);
plot(T,eI_SldSum,'-r','LineWidth',2); 
% hold on 
% plot(Ans1,'--b','LineWidth',2); 
xlabel('Time /sec','FontSize',13);
ylabel('eI-SldSum (deg)','FontSize',13);

figure(30);
set(gca,'FontSize',13);
plot(T,bias,'-r','LineWidth',2); 
axis([0 50 -20 20])
% hold on 
% plot(Ans1,'--b','LineWidth',2); 
xlabel('Time /sec','FontSize',13);
ylabel('bias ','FontSize',13);
% 
figure(4);
set(gca,'FontSize',13);
plot(T,Dlt_a,'-r','LineWidth',2); 
% hold on 
% plot(Ans1,'--b','LineWidth',2); 
xlabel('Time /sec','FontSize',13);
ylabel('Dlt-a ','FontSize',13);

figure(5);
set(gca,'FontSize',13);
plot(T,e_alp,'-r','LineWidth',2); 
% hold on 
% plot(Ans1,'--b','LineWidth',2); 
xlabel('Time /sec','FontSize',13);
ylabel('e-alp ','FontSize',13);

% figure(6);
% set(gca,'FontSize',13);
% plot(T,k1,'-r','LineWidth',2); 
% % hold on 
% % plot(Ans1,'--b','LineWidth',2); 
% xlabel('Time /sec','FontSize',13);
% ylabel('kp ','FontSize',13);
% 
% figure(7);
% set(gca,'FontSize',13);
% plot(T,k2,'-r','LineWidth',2); 
% % hold on 
% % plot(Ans1,'--b','LineWidth',2); 
% xlabel('Time /sec','FontSize',13);
% ylabel('tau-i ','FontSize',13);
% 
% figure(8);
% set(gca,'FontSize',13);
% plot(T,k3,'-r','LineWidth',2); 
% % hold on 
% % plot(Ans1,'--b','LineWidth',2); 
% xlabel('Time /sec','FontSize',13);
% ylabel('tau-d ','FontSize',13);
% 
% % figure(9);
% % set(gca,'FontSize',13);
% % plot(T,omg,'-r','LineWidth',2); 
% % % hold on 
% % % plot(Ans1,'--b','LineWidth',2); 
% % xlabel('Time /sec','FontSize',13);
% % ylabel('omg ','FontSize',13);
% 
figure(10);
set(gca,'FontSize',13);
plot(T,eC_SldSum,'-r','LineWidth',2); 
% hold on 
% plot(Ans1,'--b','LineWidth',2); 
xlabel('Time /sec','FontSize',13);
ylabel('eC_SldSum ','FontSize',13);
% 
% % figure(11);
% % set(gca,'FontSize',13);
% % plot(T,grd_K1,'-r','LineWidth',2); 
% % % hold on 
% % % plot(Ans1,'--b','LineWidth',2); 
% % xlabel('Time /sec','FontSize',13);
% % ylabel('grd-K1 ','FontSize',13);
% % 
% % figure(12);
% % set(gca,'FontSize',13);
% % plot(T,grd_K2,'-r','LineWidth',2); 
% % % hold on 
% % % plot(Ans1,'--b','LineWidth',2); 
% % xlabel('Time /sec','FontSize',13);
% % ylabel('grd-K2 ','FontSize',13);
% % 
% % figure(13);
% % set(gca,'FontSize',13);
% % plot(T,grd_K3,'-r','LineWidth',2); 
% % % hold on 
% % % plot(Ans1,'--b','LineWidth',2); 
% % xlabel('Time /sec','FontSize',13);
% % ylabel('grd-K3 ','FontSize',13);

