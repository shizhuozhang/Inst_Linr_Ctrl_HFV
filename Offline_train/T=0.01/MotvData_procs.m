%%�������ݵĴ���
%%����� ���ڶ�ƫ�빥�ǣ���ƫ����ٶ� �� ����ģ�͡�����ģ��

function [Dt_alp_2O, Dt_alp_3O, Dt_q_2O, Dt_q_3O] = MotvData_procs( MtvData )

My_PI = 3.14159;
Rad2Deg = 180 / My_PI; 
% T = MtvData(:,1);
Dlt_alp = MtvData(:,2);  %%��Ч��ƫ
alpha = Rad2Deg* MtvData(:,5);    %%��̬��
q = Rad2Deg* MtvData(:,8);   %%��̬���ٶ�

N = length(Dlt_alp); %%��ǰ���ݸ���
% buf1_alp = zeros(1,5);
% buf2_alp = zeros(1,7);
% buf1_q = zeros(1,5);
% buf2_q = zeros(1,7);
Dt_alp_2O = zeros(N-3,5);
Dt_alp_3O = zeros(N-3,7);
Dt_q_2O = zeros(N-3,5);
Dt_q_3O = zeros(N-3,7);

%% ���ݴ���  ����ģ�ͺ�����ģ�ͣ�ͳһ�ӵ��������ݿ�ʼ�ռ��������������ݾ�����һ������
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