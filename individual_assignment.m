p1 = tf(1, [1, 0]);
p2 = tf(1, [1, 0.1]);
p3 = tf(1, [1, 2*0.01*2*pi*20, (2*pi*20)^2]);
z1_max = tf([1, -2*pi*17.5], 1);
z1_min = tf([1, -2*pi*2.5], 1);
z1 = tf([1, -2*pi*10], 1);
z1_lf = tf([1, -2*pi*5], 1);
z1_rf = tf([1, -2*pi*15], 1);

sys0 = p1*p2*p3;
sys_max = sys0*z1_max;
sys_min = sys0*z1_min;
sys = sys0*z1;
sys_lf = sys0*z1_lf;
sys_rf = sys0*z1_rf;

Gp = cell(1, 5);
Gp{1} = sys;
Gp{2} = sys_min;
Gp{3} = sys_lf;
Gp{4} = sys_rf;
Gp{5} = sys_max;

w = logspace(-2,3,200); % frequency range 1e-2 to 1e3 Hz
% w = linspace(2*pi*9, 2*pi*11, 1000);

figure; hold; bode(sys_max, w,'b'); bode(sys_min, w, 'r'); bode(sys, w, 'k');
bode(sys_rf, w); bode(sys_lf, w);

G_eval = zeros(5, length(w));
for i = 1:5
    temp_sys = Gp{i};
    temp_res = freqresp(temp_sys, w);
    for j = 1:length(w)
        G_eval(i, j) = temp_res(j);
    end
end

% % Plot the Nyquist diagram
% Gim_eval = imag(G_eval);
% Gre_eval = real(G_eval);
% figure; hold on;
% for i = 1:5
%     plot(Gre_eval(i, :), Gim_eval(i, :));
% end

l1 = abs( (G_eval(2, :) - G_eval(1, :))./G_eval(1, :) );
l2 = abs( (G_eval(5, :) - G_eval(1, :))./G_eval(1, :) );

wm = tf(2*pi*7.5, [1, -2*pi*10]);
wm_eval = zeros(size(w));
temp = freqresp(wm, w);
for i = 1:length(w)
    wm_eval(i) = temp(i);
end

% WM = 0.75;
% semilogx(w, l1);
% hold on;
% semilogx(w, WM*ones(size(w)));
%% Controller design
G = fromstd(sys);
G = IOSystem(G);
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

WS = Weight.LF(4, 1, -100); 
MS = Weight.DC(4,'dc_mode', 'db'); 
WT = Weight.HF(4, 2, -60);
WU = Weight.DC(0, 'dc_mode', 'db');
WM = Weight.DC(2.5, 'dc_mode', 'db');

obj = WS*S;
% constraint = [MS*S <= 1; WT*T <= 1;WU*U<=1];
constraint = [MS*S<=1; WT*T<=1; WM*T<=1; WU*U<=1];
CL.solve(obj, constraint, K);

%%
controller = std(K.model());
[num, den] = ss2tf(controller.A, controller.B, controller.C, controller.D);
Ks = tf(num, den);

pk = pole(Ks);
zk = zero(Ks);

Ks_clean = zpk(zk, pk, -1);
figure; hold;
bode(Ks_clean, 'r');
bode(Ks);

% Run this one to check which constraint is active
Tr = minreal(sys*Ks/(1+Ks*sys));
Sr = minreal(1/(1+Ks*sys));

Tr = minreal(sys*Ks_clean/(1+Ks_clean*sys));
Sr = minreal(1/(1+Ks_clean*sys));
Trp_min = minreal(sys_min*Ks_clean/(1+Ks_clean*sys_min));
Trp_max = minreal(sys_max*Ks_clean/(1+Ks_clean*sys_max));
Trp_lf = minreal(sys_lf*Ks_clean/(1+Ks_clean*sys_lf));
Trp_rf = minreal(sys_rf*Ks_clean/(1+Ks_clean*sys_rf));

figure; step(Tr);
figure; hold on; 
bodemag(Tr); bodemag(1/WM, 'r');bodemag(1/WT, 'k');
figure; hold on;
bodemag(Sr); bodemag(6.68/WS, 'r'); bodemag(1/MS, 'k');
figure;hold on;
step(Trp_min, 'r');
step(Trp_max, 'b');
step(Tr, 'k');
step(Trp_lf);
step(Trp_rf);

%% Nominal plant
WS = Weight.LF(4, 1, -100); 
MS = Weight.DC(4,'dc_mode', 'db'); 
WT = Weight.HF(4, 2, -60);
WU = Weight.DC(0, 'dc_mode', 'db');

obj = WS*S;
constraint = [MS*S <=1; WT*T<=1];
CL.solve(obj, constraint, K);

controller = std(K.model());
[num, den] = ss2tf(controller.A, controller.B, controller.C, controller.D);
Ks = tf(num, den);

pk = pole(Ks);
zk = zero(Ks);

Ks_clean = zpk(zk, pk, 2344);
figure; hold;
bode(Ks_clean, 'r');
bode(Ks);

Tr = minreal(sys*Ks/(1+Ks*sys));
Sr = minreal(1/(1+Ks*sys));

figure; step(Tr);