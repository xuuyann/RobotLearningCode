function R = RPY_rot(alp, beta, gama)
R = [cos(alp)*cos(beta),  cos(alp)*sin(beta)*sin(gama)-sin(alp)*cos(gama),  cos(alp)*sin(beta)*cos(gama)+sin(alp)*sin(gama);
     sin(alp)*cos(beta),  sin(alp)*sin(beta)*sin(gama)+cos(alp)*cos(gama),  sin(alp)*sin(beta)*cos(gama)-cos(alp)*sin(gama);
     -sin(beta),          cos(beta)*sin(gama),                                cos(beta)*cos(gama)                             ];
end