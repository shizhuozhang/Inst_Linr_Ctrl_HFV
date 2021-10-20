%%Ϊ���������ѵ��׼������
function [DataPack] = Prepare_Data(DeltaE_1, AeroAng_1, Omega_1)

global Rad2Deg;  %%����ת��Ϊ��
global Time;   %%ʵ�ʷ���ʱ��
global DataSet_lst2;
global DataSet_lst1;
global DataSet_now;
global DataSet_1;

if Time==0
    DataSet_lst1 = [0,0,0];  %%����ֵ
    DataSet_now = [0,0,0];
    DataSet_1 = [0,0,0];
end

Dlt_alp = DeltaE_1(1);  %%��Ч��ƫ
alpha = Rad2Deg* AeroAng_1(1);    %%��̬��
q = Rad2Deg* Omega_1(1);   %%��̬���ٶ�
StateBuf = [Dlt_alp, alpha, q];

DataSet_lst2 = DataSet_lst1;
DataSet_lst1 = DataSet_now;
DataSet_now = DataSet_1;
DataSet_1 = StateBuf;  %%��ʾ��һ������

DataPack = [DataSet_1; DataSet_now; DataSet_lst1; DataSet_lst2];
end