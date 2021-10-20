% clear all 
% close all 

t = 0:0.1:50;
N = length(t);
y = zeros(N,1);

for i= 1:N
%     y(i) = (2 +0.3*t(i))* sin(4* pi* t(i)/10 ); %%仿真时长50s
%      y(i) = (2 +0.3*t(i))* sin(3* pi* t(i)/10 ); %%仿真时长60s
%      y(i) = (2 +0.3*t(i))* sin(2* pi* t(i)/10 ); %%仿真时长60s
%      y(i) = (2 +0.3*t(i))* sin(1* pi* t(i)/10 ); %%仿真时长60s
%       y(i) = (2 +0.3*t(i))* sin(0.5* pi* t(i)/10 ); %%仿真时长60s

     y(i) = (2 + 0.15*t(i))* sign(sin(4* pi* t(i)/10 ));   %%仿真时长60s     
%     y(i) = (2 + 0.15*t(i))* sign(sin(3* pi* t(i)/10 ));   %%仿真时长60s
%     y(i) = (2 + 0.15*t(i))* sign(sin(2* pi* t(i)/10 ));   %%仿真时长60s
%     y(i) = (2 + 0.15*t(i))* sign(sin(1* pi* t(i)/10 ));   %%仿真时长60s
%     y(i) = (2 + 0.15*t(i))* sign(sin(0.5* pi* t(i)/10 ));   %%仿真时长60s
    
%     if t(i)<=20
%          y(i) = 10* sin(1.5*pi* t(i)/10); %%仿真时长60s
%     else 
%         y(i) = 5* sign(sin(0.8*pi* t(i)/10));   %%仿真时长60s
%     end

end
figure(1)
plot(t,y,'-r','LineWidth',2); %
hold on
grid on;
xlabel('Time (s)','FontSize',13);
ylabel('Delta (deg)','FontSize',13);
set(gca,'FontSize',13);


% load CtrlData_1_0_1.mat
% T = CtrlData(:,1);  
% Dlt_alp = CtrlData(:,23);  Dlt_bet = CtrlData(:,24);  Dlt_mu = CtrlData(:,25); 
% % DltA = Dlt_alp;
% % save('Dlt_tst.mat','DltAlp');
% figure(1)
% plot(T,Dlt_bet,'-r','LineWidth',2); 
% grid on;
% xlabel('Time (s)','FontSize',13);
% ylabel('Delta (deg)','FontSize',13);
% set(gca,'FontSize',13);