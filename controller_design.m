
G = IOSystem(Gpar);
K = IOSystem(1,1);
r = Signal();
u = G.in();
y = G.out();
e = r - y;
connections = [K.in == e; K.out == u];
CL = IOSystem(G, K, connections);
S = e/r;
T = y/r;
U = u/r;

WS = Weight.LF(1, 1, -100); 
MS = Weight.DC(4,'dc_mode', 'db'); 
WT = Weight.HF(2, 3, -60);
WU = Weight.DC(0, 'dc_mode', 'db');

obj = WS*S;
% constraint = [MS*S <= 1; WT*T <= 1;WU*U<=1];
constraint = [MS*S <= 1; WT*T <= 1;];
CL.solve(obj, constraint, K);

controller = std(K.model());

closeloop = controller*sys/(1+sys*controller);
step(closeloop);

% Transformation from statespace to transfer function
[c_num, c_den] = ss2tf(controller.A, controller.B, controller.C, controller.D, 1);
Ks = tf(c_num, c_den);

% order reduction
zero_K = zero(Ks);
pole_K = pole(Ks);
% remove the high frequency poles and zeros, set the lowest to zero
% by hand...

Ks_clean = zpk(zero_K, pole_K, -0.09);


% Ksd = c2d(Ks, Ts, 'tustin');
% cd_num = cell2mat(Ksd.num);
% cd_den = cell2mat(Ksd.den);

Ksd_clean = c2d(Ks_clean, Ts, 'tustin');
[num_clean, den_clean] = tfdata(Ksd_clean);
cd_num_clean = cell2mat(num_clean);
cd_den_clean = cell2mat(den_clean);

% Getting a descrete system model
sysd = c2d(sys,Ts, 'tustin');
% [d_num, d_den] = ss2tf(sysd.A, sysd.B, sysd.C, sysd.D);
d_num = cell2mat(sysd.num);
d_den = cell2mat(sysd.den);

% figure; bode(K); title('Controller');
% figure; bodemag(CL(S)); title('Sensitivity');
% figure; bodemag(CL(T)); title('Complementary sensitivity');
% figure; bodemag(CL(r,u)); title('Input sensitivity (actuator effort)');