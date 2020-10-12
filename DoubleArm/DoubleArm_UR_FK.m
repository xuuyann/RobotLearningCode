clear all;
clc;
% 双臂 每个单臂是UR构型
figure(2)
dis_shoulder = 0.2;
% theta d ,a , alpha
L(1)=Link([0       0.0891+dis_shoulder    0        pi/2      0     pi/2  ]); % 关节1这里的最后一个量偏置
L(2)=Link([0       0          -0.425      0         0     0]);
L(3)=Link([0       0          -0.392      0         0     0 ]);
L(4)=Link([0       0.1091        0        pi/2      0     -pi/2]);
L(5)=Link([0       0.0946        0       -pi/2      0     0]);
L(6)=Link([0       0.0823        0        0      0     0]);
%                  0.262
urL=SerialLink(L,'name','LEFT');
% urL.tool=[0 -1 0 0;
%                1 0 0 0;
%                0 0 1 0.262 ;
%                0 0 0 1;]; 
           
R(1)=Link([0       -0.0891-dis_shoulder   0        pi/2      0     pi/2  ]); % 关节1这里的最后一个量偏置
R(2)=Link([0       0          -0.425      0         0     0]);
R(3)=Link([0       0          -0.392      0         0     0 ]);
R(4)=Link([0       0.1091        0        pi/2      0     -pi/2]);
R(5)=Link([0       0.0946        0       -pi/2      0     0]);
R(6)=Link([0       0.0823        0        0      0     0]);
%                  0.262
urR=SerialLink(R,'name','RIGHT');
% urR.tool=[0 -1 0 0;
%                1 0 0 0;
%                0 0 1 0.262 ;
%                0 0 0 1;]; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   platform
offset_ang = 0;
dis_waist = 0;
platform=SerialLink([0 dis_waist 0 pi/2+offset_ang 0 0],'name','platform');%腰部关节
platform.base=[1 0 0 0;
               0 1 0 0;
               0 0 1 0 ;
               0 0 0 1;]; %基座高度
           
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   R
pR=SerialLink([platform,urR],'name','R'); % 单独右臂模型，加装底座

view(3)
hold on
grid on
axis([-1.5, 1.5, -1.5, 1.5, -1.5, 1.5])

pR.plot([0 0 0 0 0 0 0]) % 第一个量固定为0，目的是为了模拟腰关节，左臂下同
hold on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   L
pL=SerialLink([platform, urL],'name','L'); % 单独左臂模型，加装底座

pL.plot([0 0 0 0 0 0 0])
hold on
