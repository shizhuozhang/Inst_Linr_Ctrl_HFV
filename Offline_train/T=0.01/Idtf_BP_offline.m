%��������
clear all;
close all;
%% ����ģ�����ݼ���
%�Ƕ���Ϊ���
%load Dt_alp_2O_sin.mat
% load Dt_alp_2O_puls.mat
 load Dt_alp_2O_train.mat

load Dt_alp_2O_tst.mat

% %���ٶ���Ϊ���
% load Dt_q_2O_sin.mat
% load Dt_q_2O_puls.mat
% load Dt_q_2O_train.mat
% 
% load Dt_q_2O_tst.mat

  %%����Ԥ����
X = Dt_alp_2O_train(1:4,:);  %%��������
T = Dt_alp_2O_train(5,:);  %%�������

Xc1 = Dt_alp_2O_tst1(1:4,:);
Tc1 = Dt_alp_2O_tst1(5,:);
Xc2 = Dt_alp_2O_tst2(1:4,:);
Tc2 = Dt_alp_2O_tst2(5,:);
 %% ����ģ�����ݼ���
% %�Ƕ���Ϊ���
% % load Dt_alp_3O_sin.mat
% % load Dt_alp_3O_puls.mat
% load Dt_alp_3O_train.mat
% 
% load Dt_alp_3O_tst.mat
% 
% % %���ٶ���Ϊ���
% % load Dt_q_3O_sin.mat
% % load Dt_q_3O_puls.mat
% % load Dt_q_3O_train.mat
% % 
% % load Dt_q_3O_tst.mat
% 
%   %%����Ԥ����
% X = Dt_alp_3O_train(1:6,:);  %%��������
% T = Dt_alp_3O_train(7,:);  %%�������
% 
% Xc1 = Dt_alp_3O_tst1(1:6,:);
% Tc1 = Dt_alp_3O_tst1(7,:);
% Xc2 = Dt_alp_3O_tst2(1:6,:);
% Tc2 = Dt_alp_3O_tst2(7,:);

%% ����ѵ��
[R,Q]= size(X); [S,~]= size(T); 
Hid_num = 2*R+1;  %���鹫ʽѡ������Ԫ��

 %%������ѵ��
% net = newff(X,T,Hid_num,{'tansig','purelin'});  %����BP�����磬 {'logsig','purelin'},'trainlm
net = newff(minmax(X), [Hid_num S], {'tansig' 'purelin'},'trainlm' );
net.trainParam.epochs = 1000;  %����ѵ������
net.trainParam.max_fail = 50;
net.trainParam.goal = 0.001;  %����mean square error�� �������,
net.trainParam.lr = 0.15; %����ѧϰ����
net.iw{1,1} = rands(Hid_num,R);  %��������Ԫ�ĳ�ʼȨֵ
net.lw{2,1} = rands(S,Hid_num);  %�������Ԫ�ĳ�ʼȨֵ
net.b{1} = rands(Hid_num,1);  %��������Ԫ�ĳ�ʼƫ��
net.b{2} = rands(S,1);  %�������Ԫ�ĳ�ʼƫ��

[net,tr]=train(net,X,T);  %ѵ������

% %% ����Ȩֵ
Hw = net.iw{1,1};
Hb = net.b{1};
Ow = net.lw{2,1};
Ob= net.b{2};

save('NetWeight2O_alp.mat','Hw','Hb','Ow','Ob');   %%25��������Ԫ�ȽϺã�5������
save('Net2O_alp.mat','net');
% save('NetWeight3O_alp.mat','Hw','Hb','Ow','Ob');   %%25��������Ԫ�ȽϺã�5������
% save('Net3O_alp.mat','net');
% save('NetWeight2O_q.mat','Hw','Hb','Ow','Ob');   %%25��������Ԫ�ȽϺã�5������
% save('Net2O_q.mat','net');
% save('NetWeight3O_q.mat','Hw','Hb','Ow','Ob');   %%25��������Ԫ�ȽϺã�5������
% save('Net3O_q.mat','net');

%% ��ͼ
Ans1 = sim(net,Xc1);  %��������

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
Ans2 = sim(net,Xc2);  %��������

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
