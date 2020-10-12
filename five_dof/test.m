%---------------------------------以下为robotbox函数---------------------------%
%        theta    d           a        alpha    
ML1=Link([0       1.045        0           0             ],'modified'); 
ML2=Link([0       0        0.500         pi/2            ],'modified');
ML3=Link([0       0           1.300       0              ],'modified');
ML4=Link([0       1.025       0.055      pi/2            ],'modified');
ML5=Link([0       0           0          -pi/2           ],'modified');
ML6=Link([0       1.290       0          pi/2            ],'modified');
modrobot=SerialLink([ML1 ML2 ML3 ML4 ML5 ML6],'name','KUKA360-2');
modrobot.plot([0 pi/2  0 0 0 -pi ]);
hold on;
N=3000;                                              %随机次数
theta1=-185/180*pi+(185/180*pi+185/180*pi)*rand(N,1); %关节1限制
theta2=-20/180*pi+(130/180*pi+20/180*pi)*rand(N,1);   %关节2限制
theta3=-60/180*pi+(184/180*pi+60/180*pi)*rand(N,1);  %关节3限制
theta4=-350/180*pi+(350/180*pi+350/180*pi)*rand(N,1); %关节4限制
theta5=-118/180*pi+(118/180*pi+118/180*pi)*rand(N,1); %关节5限制
theta6=-170/180*pi+(530/180*pi+170/180*pi)*rand(N,1); %关节6限制
modmyt06 = {1,N};
for n=1:1:N
    modmyt06{n}=modrobot.fkine([theta1(n),theta2(n),theta3(n),theta4(n),theta5(n),theta6(n)]);
    plot3(modmyt06{n}(1,4),modmyt06{n}(2,4),modmyt06{n}(3,4),'b.','MarkerSize',0.5);
end
% maxx=2;maxy=2;maxz=2;
% for i=1:1:N
%     if( maxx < modmyt06{i}(1,4))
%         maxx = modmyt06{i}(1,4);
%     end
%     if( maxy < modmyt06{i}(2,4))
%         maxy = modmyt06{i}(2,4);
%     end
%     if( maxz < modmyt06{i}(3,4))
%         maxz = modmyt06{i}(3,4);
%     end
% end

