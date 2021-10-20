%%为网络的在线训练准备数据
function [DataPack] = Prepare_Data(DeltaE_1, AeroAng_1, Omega_1)

global Rad2Deg;  %%弧度转换为度
global Time;   %%实际仿真时间
global DataSet_lst2;
global DataSet_lst1;
global DataSet_now;
global DataSet_1;

if Time==0
    DataSet_lst1 = [0,0,0];  %%赋初值
    DataSet_now = [0,0,0];
    DataSet_1 = [0,0,0];
end

Dlt_alp = DeltaE_1(1);  %%等效舵偏
alpha = Rad2Deg* AeroAng_1(1);    %%姿态角
q = Rad2Deg* Omega_1(1);   %%姿态角速度
StateBuf = [Dlt_alp, alpha, q];

DataSet_lst2 = DataSet_lst1;
DataSet_lst1 = DataSet_now;
DataSet_now = DataSet_1;
DataSet_1 = StateBuf;  %%表示下一步数据

DataPack = [DataSet_1; DataSet_now; DataSet_lst1; DataSet_lst2];
end