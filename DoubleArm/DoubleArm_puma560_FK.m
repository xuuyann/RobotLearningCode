clear all;
clc;
% 双臂 每个单臂是puma560构型
L(1)=Link([0       -0.25        0        pi/2      0     -pi/2  ],'modified'); % 关节1这里的最后一个量偏置-pi/2设置是为了将puma560构型的臂垂直下去
L(2)=Link([0       0          0        -pi/2      0    0  ],'modified');
L(3)=Link([0       0           0.2        0          0    -pi/2 ],'modified');
L(4)=Link([0       0.248        0           -pi/2      0     ],'modified');
L(5)=Link([0       0           0           -pi/2       0     ],'modified');
L(6)=Link([0       0            0           pi/2      0     ],'modified');
%                  0.262
p560L=SerialLink(L,'name','LEFT');
p560L.tool=[0 -1 0 0;
               1 0 0 0;
               0 0 1 0.262 ;
               0 0 0 1;]; 
           
R(1)=Link([0       0.25        0        pi/2      0     -pi/2   ],'modified');  % 关节1这里的最后一个量偏置-pi/2设置是为了将puma560构型的臂垂直下去
R(2)=Link([0      0          0           -pi/2      0    0  ],'modified');
R(3)=Link([0       0           0.2        0          0     -pi/2],'modified');
R(4)=Link([0       0.248         0           -pi/2      0     ],'modified');
R(5)=Link([0       0           0           -pi/2       0     ],'modified');
R(6)=Link([0       0           0           pi/2      0     ],'modified');
%                  0.262
p560R=SerialLink(R,'name','RIGHT');
p560R.tool=[0 -1 0 0;
               1 0 0 0;
               0 0 1 0.262 ;
               0 0 0 1;]; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   platform

platform=SerialLink([0 0 0 0],'name','platform','modified');%腰部关节
platform.base=[1 0 0 0;
               0 1 0 0;
               0 0 1 0 ;
               0 0 0 1;]; %基座高度
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   R

pR=SerialLink([platform,p560R],'name','R'); % 单独右臂模型，加装底座

view(3)
hold on
grid on
axis([-1.2, 1.2, -1.2, 1.2, -1.2, 1.2])

pR.plot([0 0 0 0 0 0 0]) % 第一个量固定为0，目的是为了模拟腰关节，左臂下同
hold on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   L
pL=SerialLink([platform,p560L],'name','L'); % 单独左臂模型，加装底座

pL.plot([0 0 0 0  0 0 0])
hold on
