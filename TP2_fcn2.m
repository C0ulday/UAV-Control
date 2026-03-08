function [ a_x , a_y ] = fcn2 ( ie , e , de , acc )

Kp =[3 3];
Kd =[3 3];
Ki =[1 1];
k = acc + Kp * e + Kd * de + Ki * ie ;
a_x = k (1) ; a_y = k (2) ;