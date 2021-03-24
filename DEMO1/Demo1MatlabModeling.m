close all;
data1=csvread('rohValuesCol1_2.csv');
x1=data1(:,1);
y1=data1(:,2);

data2=csvread('phiValuesCol1_3.csv');
x2=data2(:,1);
y2=data2(:,2);

k1=2.1;
sigma1=10;

k2=-.175;
sigma2=10;

open_system('Demo1MotorTuning');
out=sim('Demo1MotorTuning');

figure(1)
plot(out.simout1);

figure(2);
title('Rho Values');
plot(x1,y1);

figure(3);
title('Phi Values');
plot(x2,y2);

figure(4)
plot(out.simout2);



