% 2018年11月19日 by zhangyaokun
% BP identification
clear all;
close all;
%% BP网络参数初始化
xite=0.30;  %学习速率
alfa=0.05;  %动量因子

w2=rands(9,1);  %随机初始化网络权值
w2_1=w2;w2_2=w2_1;  %网络结构，4-9-1
w1=rands(4,9);
w1_1=w1;w1_2=w1;
dw1=0*w1;

I=[0,0,0,0,0,0,0,0,0]';
Iout=[0,0,0,0,0,0,0,0,0]';
FI=[0,0,0,0,0,0,0,0,0]';

%% 训练网络，对非线性系统进行辨识
for count=1:100
    x=[0,0,0,0]';%网络的初始输入为0
 %   x=[0,0,0]';%网络的初始输入为0
    u_1=0;y_1=0;y_2=0;
    
    for  k=1:1:500   %使用500个样本点训练数据
        time(k)=k;
        u(k)=sin(2*pi/3*k) + 1/3*sin(2*pi/6*k);
        y(k) = u_1+ y_1 * (2*y_2 + 1) / (1+ y_1^2 + y_2^2);  %非线性系统

        for  j=1:1:9
             I(j)=x'*w1(:,j);  
  %           Iout(j)=(exp(I(j))-exp(-I(j)))/(exp(I(j))+exp(-I(j)));
             Iout(j)=1/(1+exp(-I(j))); %网络隐含层输出，使用sigmod函数
        end   
        yn(k)=w2'*Iout;         % 网络的输出层输出
        e(k)=y(k)-yn(k);    % 辨识误差

        %网络权值修正
        w2=w2_1+(xite*e(k))*Iout+alfa*(w2_1-w2_2);  %反向传播，梯度法算输出层权值修正值

        for j=1:1:9
          FI(j)=exp(-I(j))/(1+exp(-I(j)))^2;  %隐含层网络的导数
%             FI(j)=4/((exp(I(j))+exp(-I(j)))^2);
        end
        for i=1:1:4
           for j=1:1:9
              dw1(i,j)=xite*e(k)*FI(j)*w2(j)*x(i);
           end
        end
        w1=w1_1+dw1+alfa*(w1_1-w1_2);

        for j=1:1:9
           yu=w2(j)*w1(1,j)*FI(j);    %Jacobian
        end
        dyu(k)=yu;

        %各参数更新
        x(1)=u(k);
        x(2)=y(k);
         x(3)=y_1;
         x(4)=y_2;

        w1_2=w1_1;w1_1=w1;
        w2_2=w2_1;w2_1=w2;
        u_1=u(k);
        y_2=y_1;
        y_1=y(k);
    end
end
%% 网络训练情况曲线显示
figure(1);
plot(time,y,'r',time,yn,'b');
xlabel('times');ylabel('y and yn');
figure(2);
plot(time,e,'r');
xlabel('times');ylabel('error');
figure(3);
plot(time,dyu);
xlabel('times');ylabel('dyu');

%% 使用新的数据测试网络
x=[0,0,0,0]';  %网络初始输入为0
u_1=0;y_1=0;y_2=0;  %系统初始输出输入重新初始化为0
I=[0,0,0,0,0,0,0]';
Iout=[0,0,0,0,0,0,0]';
for k=1:1:200   %使用200个样本点
    time1(k)=k;   
   uc(k)=sin(2*pi/4*k) + 1/5*sin(2*pi/7*k);   %新的测试函数
 %   uc(k)=sin(2*pi/3*k*ts) + 1/3*sin(2*pi/6*k*ts);
    yc(k) = u_1+ y_1 * (2*y_2 + 1) / (1+ y_1^2 + y_2^2);  %非线性系统
    
    for  j=1:1:9   
         I(j)=x'*w1(:,j);   
  %        Iout(j)=(exp(I(j))-exp(-I(j)))/(exp(I(j))+exp(-I(j)));
         Iout(j)=1/(1+exp(-I(j))); %网络隐含层输出，使用sigmod函数
    end   
    ync(k)=w2'*Iout;         % 网络的输出层输出
    ec(k)=yc(k)-ync(k);    % 辨识误差
    
    %各参数更新
    x(1)=uc(k);
    x(2)=yc(k);
     x(3)=y_1;
     x(4)=y_2;
    
    u_1=uc(k);
    y_2=y_1;
    y_1=yc(k);
end
figure(4);
plot(time1,yc,'r',time1,ync,'b');
xlabel('times');ylabel('y and yn');
figure(5);
plot(time1,ec,'r');
xlabel('times');ylabel('error');