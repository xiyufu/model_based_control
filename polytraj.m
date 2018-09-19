function [pos,vel,acc,jerk,px]= polytraj(d,Ts, nr, nr_tot);

% [pos,vel,acc,jerk]= polytraj(d,Ts, nr, nr_tot);
% generation of trajectory with 9th order polynomial
% d = distance
% Ts = sample period
% nr : number of samples of period of motion
% nr_tot = total number of samples of trajectory
% pos = position
% vel = velocity
% acc = acceleration
% jerk = jerk
% Jan Swevers March 2006

tm = nr*Ts;
t_tot = nr_tot*Ts;

% determine the coefficients of the polynomial trajectory
% p = a9 t^9 + a8 t^8 ... a1 t^1 + a0

a0 = 0; % p(0) = 0
a1 = 0; % v(0) = 0
a2 = 0; % a(0) = 0
a3 = 0; % j(0) = 0
a4 = 0; % diff(j(0)) = 0

%p(tm) = d;
A = [tm^9 tm^8 tm^7 tm^6 tm^5]; 
b = d; 

%v(tm) = 0;
A = [A; 9*tm^8 8*tm^7 7*tm^6 6*tm^5 5*tm^4];
b = [b;0];

%a(tm) = 0;
A = [A; 8*9*tm^7 7*8*tm^6 6*7*tm^5 5*6*tm^4 4*5*tm^3];
b = [b;0];

%j(tm) = 0;
A = [A; 7*8*9*tm^6 6*7*8*tm^5 5*6*7*tm^4 4*5*6*tm^3 3*4*5*tm^2];
b = [b;0];

%diff(j(tm)) = 0;
A = [A; 6*7*8*9*tm^5 5*6*7*8*tm^4 4*5*6*7*tm^3 3*4*5*6*tm^2 2*3*4*5*tm^1];
b = [b;0];


x = inv(A)*b;
x = x';

px = [x 0 0 0 0 0]';

t1 = (0:1:nr-1)'*Ts;

pos = polyval(px,t1);
pos = [pos;ones(nr_tot-nr,1)*d];

vx = [[9 8 7 6 5].*x 0 0 0 0]';
vel = polyval(vx,t1);
vel = [vel;zeros(nr_tot-nr,1)];

ax = [[9*8 8*7 7*6 6*5 5*4].*x 0 0 0]';
acc = polyval(ax,t1);
acc = [acc;zeros(nr_tot-nr,1)];

jx = [[9*8*7 8*7*6 7*6*5 6*5*4 5*4*3].*x 0 0]';
jerk = polyval(jx,t1);
jerk = [jerk;zeros(nr_tot-nr,1)];

t = (0:1:nr_tot-1)'*Ts;
% figure(1)
% plot(t,pos);xlabel('Time [sec]');ylabel('m');title('position in [m]');
% figure(2)
% plot(t,vel);xlabel('Time [sec]');ylabel('m/s');title('velocity');
% figure(3)
% plot(t,acc);xlabel('Time [sec]');ylabel('m/s^2');title('acceleration');
% figure(4)
% plot(t,jerk);xlabel('Time [sec]');ylabel('m/s^3');title('jerk');