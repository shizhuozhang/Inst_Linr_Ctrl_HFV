%%激励数据的处理
%%输出有 关于舵偏与攻角，舵偏与角速度 的 二阶模型、三阶模型

function [Dt_alp_2O, Dt_alp_3O, Dt_q_2O, Dt_q_3O] = MotvData_procs( MtvData )

My_PI = 3.14159;
Rad2Deg = 180 / My_PI; 
% T = MtvData(:,1);
Dlt_alp = MtvData(:,2);  %%等效舵偏
alpha = Rad2Deg* MtvData(:,5);    %%姿态角
q = Rad2Deg* MtvData(:,8);   %%姿态角速度

N = length(Dlt_alp); %%当前数据个数
% buf1_alp = zeros(1,5);
% buf2_alp = zeros(1,7);
% buf1_q = zeros(1,5);
% buf2_q = zeros(1,7);
Dt_alp_2O = zeros(N-3,5);
Dt_alp_3O = zeros(N-3,7);
Dt_q_2O = zeros(N-3,5);
Dt_q_3O = zeros(N-3,7);

%% 数据处理  二阶模型和三阶模型，统一从第三个数据开始收集，这样二阶数据就少了一个样本
for i = 4: N 
    k = i-1;
    
    u_now = Dlt_alp( k);
    u_lst1 = Dlt_alp( k-1);
    u_lst2 = Dlt_alp( k-2);
    
    alp_1 = alpha( k +1);
    alp_now  = alpha( k );
    alp_lst1  = alpha( k -1);
    alp_lst2  = alpha( k -2);
    
    q_1 = q( k+1);
    q_now = q( k);
    q_lst1 = q( k-1);
    q_lst2 = q( k-2);
    
    buf1_alp = [u_lst1, u_now, alp_lst1, alp_now, alp_1];
    buf2_alp = [u_lst2, u_lst1, u_now, alp_lst2, alp_lst1, alp_now, alp_1];
    buf1_q = [u_lst1, u_now, q_lst1, q_now, q_1];
    buf2_q = [u_lst2, u_lst1, u_now, q_lst2, q_lst1, q_now, q_1];

    Dt_alp_2O(i-3,:) = buf1_alp;
    Dt_alp_3O(i-3,:) = buf2_alp;
    
    Dt_q_2O(i-3,:) = buf1_q;
    Dt_q_3O(i-3,:) = buf2_q;    
    
end

end