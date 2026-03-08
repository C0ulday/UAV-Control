function [ a_x , a_y ] = fcn (e , de , acc )

Kp = [1 1];
Kd = [2 2];
k = acc + Kp * e + Kd * de ;
a_x = k (1) ;
a_y = k (2) ;