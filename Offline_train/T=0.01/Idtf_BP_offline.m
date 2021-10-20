%生成数据
clear all;
close all;
%% 二阶模型数据加载
%角度作为输出
%load Dt_alp_2O_sin.mat
% load Dt_alp_2O_puls.mat
 load Dt_alp_2O_train.mat

load Dt_alp_2O_tst.mat

% %角速度作为输出
% load Dt_q_2O_sin.mat
% load Dt_q_2O_puls.mat
% load Dt_q_2O_train.mat
% 
% load Dt_q_2O_tst.mat

  %%数据预处理
X = Dt_alp_2O_train(1:4,:);  %%网络输入
T = Dt_alp_2O_train(5,:);  %%网络输出

Xc1 = Dt_alp_2O_tst1(1:4,:);
Tc1 = Dt_alp_2O_tst1(5,:);
Xc2 = Dt_alp_2O_tst2(1:4,:);
Tc2 = Dt_alp_2O_tst2(5,:);
 %% 三阶模型数据加载
% %角度作为输出
% % load Dt_alp_3O_sin.mat
% % load Dt_alp_3O_puls.mat
% load Dt_alp_3O_train.mat
% 
% load Dt_alp_3O_tst.mat
% 
% % %角速度作为输出
% % load Dt_q_3O_sin.mat
% % load Dt_q_3O_puls.mat
% % load Dt_q_3O_train.mat
% % 
% % load Dt_q_3O_tst.mat
% 
%   %%数据预处理
% X = Dt_alp_3O_train(1:6,:);  %%网络输入
% T = Dt_alp_3O_train(7,:);  %%网络输出
% 
% Xc1 = Dt_alp_3O_tst1(1:6,:);
% Tc1 = Dt_alp_3O_tst1(7,:);
% Xc2 = Dt_alp_3O_tst2(1:6,:);
% Tc2 = Dt_alp_3O_tst2(7,:);

%% 网络训练
[R,Q]= size(X); [S,~]= size(T); 
Hid_num = 2*R+1;  %经验公式选隐含层元数

 %%网络搭建与训练
% net = newff(X,T,Hid_num,{'tansig','purelin'});  %建立BP神经网络， {'logsig','purelin'},'trainlm
net = newff(minmax(X), [Hid_num S], {'tansig' 'purelin'},'trainlm' );
net.trainParam.epochs = 1000;  %设置训练次数
net.trainParam.max_fail = 50;
net.trainParam.goal = 0.001;  %设置mean square error， 均方误差,
net.trainParam.lr = 0.15; %设置学习速率
net.iw{1,1} = rands(Hid_num,R);  %隐含层神经元的初始权值
net.lw{2,1} = rands(S,Hid_num);  %输出层神经元的初始权值
net.b{1} = rands(Hid_num,1);  %隐含层神经元的初始偏置
net.b{2} = rands(S,1);  %输出层神经元的初始偏置

[net,tr]=train(net,X,T);  %训练网络

% %% 保存权值
Hw = net.iw{1,1};
Hb = net.b{1};
Ow = net.lw{2,1};
Ob= net.b{2};

save('NetWeight2O_alp.mat','Hw','Hb','Ow','Ob');   %%25个隐含层元比较好，5还不错
save('Net2O_alp.mat','net');
% save('NetWeight3O_alp.mat','Hw','Hb','Ow','Ob');   %%25个隐含层元比较好，5还不错
% save('Net3O_alp.mat','net');
% save('NetWeight2O_q.mat','Hw','Hb','Ow','Ob');   %%25个隐含层元比较好，5还不错
% save('Net2O_q.mat','net');
% save('NetWeight3O_q.mat','Hw','Hb','Ow','Ob');   %%25个隐含层元比较好，5还不错
% save('Net3O_q.mat','net');

%% 绘图
Ans1 = sim(net,Xc1);  %测试网络

figure(1);
plot(Tc1,'-r','LineWidth',2); 
hold on 
plot(Ans1,'--b','LineWidth',2); 
xlabel('test sample','FontSize',13);
ylabel('Angle (deg)','FontSize',13);
h1=legend('alp_{Exp}','alp_{Pre}','Location','NorthEast');
set(h1,'box','off');
set(gca,'FontSize',13);
% title('Dlt_alp','FontSize',13);

figure(2);
plot(Tc1-Ans1,':r','LineWidth',2);
xlabel('test sample','FontSize',13);
ylabel('Error (deg)','FontSize',13);
set(gca,'FontSize',13);

% load('Net2O_alp.mat','net');
Ans2 = sim(net,Xc2);  %测试网络

figure(3);
plot(Tc2,'-r','LineWidth',2); 
hold on 
plot(Ans2,'--b','LineWidth',2); 
xlabel('test sample','FontSize',13);
ylabel('Angle (deg)','FontSize',13);
h1=legend('Expect','Predict','Location','NorthEast');
set(h1,'box','off');
set(gca,'FontSize',13);
% title('Dlt_alp','FontSize',13);

figure(4);
plot(Tc2-Ans2,':r','LineWidth',2);
xlabel('test sample','FontSize',13);
ylabel('Error (deg)','FontSize',13);
set(gca,'FontSize',13);
