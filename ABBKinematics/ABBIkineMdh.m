% 适用于Modified DH参数法
function ik_T = ABBIkineMdh(fk_T)
d(1) = 0; a(1) = 0;
d(2) = 0; a(2) = 0.320;%a(2)=a1
d(3) = 0; a(3) = 0.975;%a(3)=a2
d(4) = 0.887; a(4) = 0.2;%a(4)=a3
d(5) = 0; a(5) = 0;
d(6) = 0; a(6) = 0;

nx = fk_T(1, 1); ox = fk_T(1, 2); ax = fk_T(1, 3); px = fk_T(1, 4);
ny = fk_T(2, 1); oy = fk_T(2, 2); ay = fk_T(2, 3); py = fk_T(2, 4);
nz = fk_T(3, 1); oz = fk_T(3, 2); az = fk_T(3, 3); pz = fk_T(3, 4);
%% theta1有两个解
theta1_1 = atan2((d(3)+d(2)), (px^2+py^2-(d(3)+d(2))^2)^0.5) + atan2(py, px);
theta1_2 = atan2((d(3)+d(2)), -(px^2+py^2-(d(3)+d(2))^2)^0.5) + atan2(py, px);

%% theta3有四个解
K_1 = ((a(2)-cos(theta1_1)*px-sin(theta1_1)*py)^2+pz^2-a(4)^2-d(4)^2-a(3)^2) / 2*a(3);
K_2 = ((a(2)-cos(theta1_2)*px-sin(theta1_2)*py)^2+pz^2-a(4)^2-d(4)^2-a(3)^2) / 2*a(3);
theta3_1 = atan2(K_1, (a(4)^2+d(4)^2-K_1^2)^0.5) - atan2(a(4), d(4));%theta1_1
theta3_2 = atan2(K_1, -(a(4)^2+d(4)^2-K_1^2)^0.5) - atan2(a(4), d(4));%theta1_1
theta3_3 = atan2(K_2, (a(4)^2+d(4)^2-K_2^2)^0.5) - atan2(a(4), d(4));%theta1_2
theta3_4 = atan2(K_2, -(a(4)^2+d(4)^2-K_2^2)^0.5) - atan2(a(4), d(4));%theta1_2

%% theta2(分别对应theta1和theta3，theta1和theta3各自也有对应)，有四个解
e21 = a(4)*cos(theta3_1) + d(4)*sin(theta3_1) + a(3);%theta3_1
f21 = a(4)*sin(theta3_1) - d(4)*cos(theta3_1);
e22 = a(4)*cos(theta3_2) + d(4)*sin(theta3_2) + a(3);%theta3_2
f22 = a(4)*sin(theta3_2) - d(4)*cos(theta3_2);
e23 = a(4)*cos(theta3_3) + d(4)*sin(theta3_3) + a(3);%theta3_3
f23 = a(4)*sin(theta3_3) - d(4)*cos(theta3_3);
e24 = a(4)*cos(theta3_4) + d(4)*sin(theta3_4) + a(3);%theta3_4
f24 = a(4)*sin(theta3_4) - d(4)*cos(theta3_4);

g1 = cos(theta1_1)*px + sin(theta1_1)*py - a(2);%theta1_1
g2 = cos(theta1_2)*px + sin(theta1_2)*py - a(2);%theta1_2

theta2_1 = atan2(pz*e21-g1*f21, g1*e21+pz*f21);%theta3_1,theta1_1
theta2_2 = atan2(pz*e22-g1*f22, g1*e22+pz*f22);%theta3_2,theta1_1
theta2_3 = atan2(pz*e23-g2*f23, g2*e23+pz*f23);%theta3_3,theta1_2
theta2_4 = atan2(pz*e24-g2*f24, g2*e24+pz*f24);%theta3_4,theta1_2

%% theta5,八个解
m51 = cos(theta1_1)*cos(theta2_1)*ax + sin(theta1_1)*cos(theta2_1)*ay + sin(theta2_1)*az;%theta3_1
n51 = cos(theta1_1)*sin(theta2_1)*ax + sin(theta1_1)*sin(theta2_1)*ay - cos(theta2_1)*az;
m52 = cos(theta1_1)*cos(theta2_2)*ax + sin(theta1_1)*cos(theta2_2)*ay + sin(theta2_2)*az;%theta3_2
n52 = cos(theta1_1)*sin(theta2_2)*ax + sin(theta1_1)*sin(theta2_2)*ay - cos(theta2_2)*az;
m53 = cos(theta1_2)*cos(theta2_3)*ax + sin(theta1_2)*cos(theta2_3)*ay + sin(theta2_3)*az;%theta3_3
n53 = cos(theta1_2)*sin(theta2_3)*ax + sin(theta1_2)*sin(theta2_3)*ay - cos(theta2_3)*az;
m54 = cos(theta1_2)*cos(theta2_4)*ax + sin(theta1_2)*cos(theta2_4)*ay + sin(theta2_4)*az;%theta3_4
n54 = cos(theta1_2)*sin(theta2_4)*ax + sin(theta1_2)*sin(theta2_4)*ay - cos(theta2_4)*az;

theta5_1 = atan2(((cos(theta3_1)*m51-sin(theta3_1)*n51)^2 + (cos(theta1_1)*ay-sin(theta1_1)*ax)^2)^0.5, sin(theta3_1)*m51 + cos(theta3_1)*n51);
theta5_2 = atan2(((cos(theta3_2)*m52-sin(theta3_2)*n52)^2 + (cos(theta1_1)*ay-sin(theta1_1)*ax)^2)^0.5, sin(theta3_2)*m52 + cos(theta3_2)*n52);
theta5_3 = atan2(((cos(theta3_3)*m53-sin(theta3_3)*n53)^2 + (cos(theta1_2)*ay-sin(theta1_2)*ax)^2)^0.5, sin(theta3_3)*m53 + cos(theta3_3)*n53);
theta5_4 = atan2(((cos(theta3_4)*m54-sin(theta3_4)*n54)^2 + (cos(theta1_2)*ay-sin(theta1_2)*ax)^2)^0.5, sin(theta3_4)*m54 + cos(theta3_4)*n54);
theta5_5 = atan2(-((cos(theta3_1)*m51-sin(theta3_1)*n51)^2 + (cos(theta1_1)*ay-sin(theta1_1)*ax)^2)^0.5, sin(theta3_1)*m51 + cos(theta3_1)*n51);
theta5_6 = atan2(-((cos(theta3_2)*m52-sin(theta3_2)*n52)^2 + (cos(theta1_1)*ay-sin(theta1_1)*ax)^2)^0.5, sin(theta3_2)*m52 + cos(theta3_2)*n52);
theta5_7 = atan2(-((cos(theta3_3)*m53-sin(theta3_3)*n53)^2 + (cos(theta1_2)*ay-sin(theta1_2)*ax)^2)^0.5, sin(theta3_3)*m53 + cos(theta3_3)*n53);
theta5_8 = atan2(-((cos(theta3_4)*m54-sin(theta3_4)*n54)^2 + (cos(theta1_2)*ay-sin(theta1_2)*ax)^2)^0.5, sin(theta3_4)*m54 + cos(theta3_4)*n54);

%% theta4，对应于theta5，八个解
if theta5_1 == 0 %m51,theta3_1
    theta4_1 = 0;
else
    theta4_1 = atan2((sin(theta1_1)*ax-cos(theta1_1)*ay)/sin(theta5_1), (cos(theta3_1)*m51-sin(theta3_1)*n51)/sin(theta5_1));
end
if theta5_2 == 0%m52,theta3_2
    theta4_2 = 0;
else
    theta4_2 = atan2((sin(theta1_1)*ax-cos(theta1_1)*ay)/sin(theta5_2), (cos(theta3_2)*m52-sin(theta3_2)*n52)/sin(theta5_2));
end
if theta5_3 == 0 %m53,theta3_3
    theta4_3 = 0;
else
    theta4_3 = atan2((sin(theta1_2)*ax-cos(theta1_2)*ay)/sin(theta5_3), (cos(theta3_3)*m53-sin(theta3_3)*n53)/sin(theta5_3));
end
if theta5_4 == 0 %m54,theta3_4
    theta4_4 = 0;
else
    theta4_4 = atan2((sin(theta1_2)*ax-cos(theta1_2)*ay)/sin(theta5_4), (cos(theta3_4)*m54-sin(theta3_4)*n51)/sin(theta5_4));
end 
if theta5_5 == 0 %m51, theta3_1
    theta4_5 = 0;
else
    theta4_5 = atan2((sin(theta1_1)*ax-cos(theta1_1)*ay)/sin(theta5_5), (cos(theta3_1)*m51-sin(theta3_1)*n51)/sin(theta5_5));
end 
if theta5_6 == 0 %m52, theta3_2
    theta4_6 = 0;
else
    theta4_6 = atan2((sin(theta1_1)*ax-cos(theta1_1)*ay)/sin(theta5_6), (cos(theta3_2)*m52-sin(theta3_2)*n52)/sin(theta5_6));
end
if theta5_7 == 0 %m53, theta3_3
    theta4_7 = 0;
else
    theta4_7 = atan2((sin(theta1_2)*ax-cos(theta1_2)*ay)/sin(theta5_7), (cos(theta3_3)*m53-sin(theta3_3)*n53)/sin(theta5_7));
end
if theta5_8 == 0 %m54, theta3_4
    theta4_8 = 0;
else
    theta4_8 = atan2((sin(theta1_2)*ax-cos(theta1_2)*ay)/sin(theta5_8), (cos(theta3_4)*m51-sin(theta3_4)*n54)/sin(theta5_8));
end
    
%% theta6
p61 = sin(theta1_1)*nx - cos(theta1_1)*ny;
q61 = sin(theta1_1)*ox - cos(theta1_1)*oy;
p62 = sin(theta1_2)*nx - cos(theta1_2)*ny;
q62 = sin(theta1_2)*ox - cos(theta1_2)*oy;

theta6_1 = atan2(cos(theta4_1)*p61-sin(theta4_1)*cos(theta5_1)*q61, cos(theta4_1)*q61+sin(theta4_1)*cos(theta5_1)*p61);
theta6_2 = atan2(cos(theta4_2)*p61-sin(theta4_2)*cos(theta5_2)*q61, cos(theta4_2)*q61+sin(theta4_2)*cos(theta5_2)*p61);
theta6_3 = atan2(cos(theta4_3)*p62-sin(theta4_3)*cos(theta5_3)*q62, cos(theta4_3)*q62+sin(theta4_3)*cos(theta5_3)*p62);
theta6_4 = atan2(cos(theta4_4)*p62-sin(theta4_4)*cos(theta5_4)*q62, cos(theta4_4)*q62+sin(theta4_4)*cos(theta5_4)*p62);
theta6_5 = atan2(cos(theta4_5)*p61-sin(theta4_5)*cos(theta5_5)*q61, cos(theta4_5)*q61+sin(theta4_5)*cos(theta5_5)*p61);
theta6_6 = atan2(cos(theta4_6)*p61-sin(theta4_6)*cos(theta5_6)*q61, cos(theta4_6)*q61+sin(theta4_6)*cos(theta5_6)*p61);
theta6_7 = atan2(cos(theta4_7)*p62-sin(theta4_7)*cos(theta5_7)*q62, cos(theta4_7)*q62+sin(theta4_7)*cos(theta5_7)*p62);
theta6_8 = atan2(cos(theta4_8)*p62-sin(theta4_8)*cos(theta5_8)*q62, cos(theta4_8)*q62+sin(theta4_8)*cos(theta5_8)*p62);

%% 最终得到八个解
ik_T = [theta1_1 theta2_1 theta3_1 theta4_1 theta5_1 theta6_1;
        theta1_1 theta2_2 theta3_2 theta4_2 theta5_2 theta6_2;
        theta1_2 theta2_3 theta3_3 theta4_3 theta5_3 theta6_3;
        theta1_2 theta2_4 theta3_4 theta4_4 theta5_4 theta6_4;
        theta1_1 theta2_1 theta3_1 theta4_5 theta5_5 theta6_5;
        theta1_1 theta2_2 theta3_2 theta4_6 theta5_6 theta6_6;
        theta1_2 theta2_3 theta3_3 theta4_7 theta5_7 theta6_7;
        theta1_2 theta2_4 theta3_4 theta4_8 theta5_8 theta6_8;];

end
