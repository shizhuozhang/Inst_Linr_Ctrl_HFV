%%training_data
clear all;
close all;

My_PI = 3.14159;
Rad2Deg = 180 / My_PI;

%% 不同信号的训练数据
%正弦波信号数据
load Motv_data11.mat
[Dt_alp_2O_11, Dt_alp_3O_11, Dt_q_2O_11, Dt_q_3O_11] = MotvData_procs(MtvData);

load Motv_data1.mat
[Dt_alp_2O_1, Dt_alp_3O_1, Dt_q_2O_1, Dt_q_3O_1] = MotvData_procs(MtvData);

load Motv_data2.mat
[Dt_alp_2O_2, Dt_alp_3O_2, Dt_q_2O_2, Dt_q_3O_2] = MotvData_procs(MtvData);

load Motv_data3.mat
[Dt_alp_2O_3, Dt_alp_3O_3, Dt_q_2O_3, Dt_q_3O_3] = MotvData_procs(MtvData);

load Motv_data4.mat
[Dt_alp_2O_4, Dt_alp_3O_4, Dt_q_2O_4, Dt_q_3O_4] = MotvData_procs(MtvData);

%方波信号数据
load Motv_data5.mat
[Dt_alp_2O_5, Dt_alp_3O_5, Dt_q_2O_5, Dt_q_3O_5] = MotvData_procs(MtvData);

load Motv_data6.mat
[Dt_alp_2O_6, Dt_alp_3O_6, Dt_q_2O_6, Dt_q_3O_6] = MotvData_procs(MtvData);

load Motv_data7.mat
[Dt_alp_2O_7, Dt_alp_3O_7, Dt_q_2O_7, Dt_q_3O_7] = MotvData_procs(MtvData);

load Motv_data8.mat
[Dt_alp_2O_8, Dt_alp_3O_8, Dt_q_2O_8, Dt_q_3O_8] = MotvData_procs(MtvData);

%测试信号数据
load Motv_data9.mat
[Dt_alp_2O_9, Dt_alp_3O_9, Dt_q_2O_9, Dt_q_3O_9] = MotvData_procs(MtvData);

load Motv_data0.mat
[Dt_alp_2O_0, Dt_alp_3O_0, Dt_q_2O_0, Dt_q_3O_0] = MotvData_procs(MtvData);
 

%%数据整合

%仅不同频率的正弦信号
Dt_alp_2O_sin = [Dt_alp_2O_11; Dt_alp_2O_1; Dt_alp_2O_2; Dt_alp_2O_3; Dt_alp_2O_4]';
save('Dt_alp_2O_sin.mat','Dt_alp_2O_sin');
Dt_alp_3O_sin = [Dt_alp_3O_11; Dt_alp_3O_1; Dt_alp_3O_2; Dt_alp_3O_3; Dt_alp_3O_4]';
save('Dt_alp_3O_sin.mat','Dt_alp_3O_sin');
Dt_q_2O_sin = [Dt_q_2O_11; Dt_q_2O_1; Dt_q_2O_2; Dt_q_2O_3; Dt_q_2O_4]';
save('Dt_q_2O_sin.mat','Dt_q_2O_sin'); 
Dt_q_3O_sin = [Dt_q_3O_11; Dt_q_3O_1; Dt_q_3O_2; Dt_q_3O_3; Dt_q_3O_4]';
save('Dt_q_3O_sin.mat','Dt_q_3O_sin');  

%仅不同信号的方波信号
Dt_alp_2O_puls = [Dt_alp_2O_5; Dt_alp_2O_6; Dt_alp_2O_7; Dt_alp_2O_8]';
save('Dt_alp_2O_puls.mat','Dt_alp_2O_puls');  
Dt_alp_3O_puls = [Dt_alp_3O_5; Dt_alp_3O_6; Dt_alp_3O_7; Dt_alp_3O_8]';
save('Dt_alp_3O_puls.mat','Dt_alp_3O_puls');  
Dt_q_2O_puls = [Dt_q_2O_5; Dt_q_2O_6; Dt_q_2O_7; Dt_q_2O_8]';
save('Dt_q_2O_puls.mat','Dt_q_2O_puls');  
Dt_q_3O_puls = [Dt_q_3O_5; Dt_q_3O_6; Dt_q_3O_7; Dt_q_3O_8]';
save('Dt_q_3O_puls.mat','Dt_q_3O_puls');  

%方波正弦信号均有
Dt_alp_2O_train = [Dt_alp_2O_sin, Dt_alp_2O_puls];
save('Dt_alp_2O_train.mat','Dt_alp_2O_train');  
Dt_alp_3O_train = [Dt_alp_3O_sin, Dt_alp_3O_puls];
save('Dt_alp_3O_train.mat','Dt_alp_3O_train');  
Dt_q_2O_train = [Dt_q_2O_sin, Dt_q_2O_puls];
save('Dt_q_2O_train.mat','Dt_q_2O_train');  
Dt_q_3O_train = [Dt_q_3O_sin, Dt_q_3O_puls];
save('Dt_q_3O_train.mat','Dt_q_3O_train');  

%%测试信号，分为两组测试信号，9是方波与正弦组合信号，0是弹道制导指令
Dt_alp_2O_tst1 = Dt_alp_2O_9';
Dt_alp_2O_tst2 = Dt_alp_2O_0';
Dt_alp_3O_tst1 = Dt_alp_3O_9';
Dt_alp_3O_tst2 = Dt_alp_3O_0';
Dt_q_2O_tst1 = Dt_q_2O_9';
Dt_q_2O_tst2 = Dt_q_2O_0';
Dt_q_3O_tst1 = Dt_q_3O_9';
Dt_q_3O_tst2 = Dt_q_3O_0';

save('Dt_alp_2O_tst.mat','Dt_alp_2O_tst1','Dt_alp_2O_tst2');
save('Dt_alp_3O_tst.mat','Dt_alp_3O_tst1','Dt_alp_3O_tst2');
save('Dt_q_2O_tst.mat','Dt_q_2O_tst1','Dt_q_2O_tst2');
save('Dt_q_3O_tst.mat','Dt_q_3O_tst1','Dt_q_3O_tst2');
