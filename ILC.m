% First run demo.m in identification to get a model

N = 9000;

iter = 50;

% sys = std(Gpar);
% sysd = c2d(sys,Ts, 'tustin');
% 
% A = sysd.A;
% B = sysd.B;
% C = sysd.C;
% D = sysd.D;

% PD type
kp = 1;
kd = 50;
[qb,qa] = butter(4, 10/(fs/2)); % 4th order butterworth filter, 10Hz

% Get ej and uj
for j = 1:iter
    if j == 1
        u1 = ut0.signals.values;
        e1 = et0.signals.values;
    else
        u1 = u2;
        e1 = et1.signals.values;
    end
    u2 = u1 + filter(num_F, den_F, e1);
    u2 = filtfilt(qb, qa, u2);
    input.time = input.time(1:length(u2));
    input.signals.values = u2;
    
    % sim('discrete_Hinf_control');
   j
    
end




