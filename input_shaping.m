[Wn, zeta] = damp(Tr);

% What is the natural frequency and damping of residual vibration? Don't
% konw...
Wd = min(Wn);
index = (Wn == Wd);
zetad = zeta(index);
zetad = zetad(1);

temp = exp(zetad*pi/sqrt(1-zetad^2));

Td = 2*pi/Wd;

A1 = 1/(1+temp);
A2 = temp/(1+temp);
t1 = 0;
t2 = 0.5*Td;

[ref,~,~,~,~] = polytraj(300, Ts, 6000, 10000);
n = floor(Td/Ts);
ref1 = [ref;ref(end)*ones(n, 1)];
ref2 = [zeros(n,1); ref];
ref_shaped = A1*ref1 + A2*ref2;

time = [0:length(ref)-1]*Ts;
time_shaped = [0:length(ref_shaped)-1]*Ts;

traj.time = time;
traj.signals.values = ref;
traj_shaped = traj;
traj_shaped.time = time_shaped;
traj_shaped.signals.values = ref_shaped;

% FIR coefficients
c_FIR = zeros(1,n);
c_FIR(1) = A1;
c_FIR(n) = A2;
