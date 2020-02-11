%Calculations Assignment 2
T = 0.002;
A_c = [0, 1, 0, 0;
    0, 0, -K_2, 0;
    0, 0, 0, 1;
    0, 0, -K_1*K_pp, -K_1*K_pd];
B_c = [0;0;0;K_1*K_pp];
A_d = T*A_c+eye(4);
B_d = T*B_c;