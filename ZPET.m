% Clean the controller by hand in controller_design.m before running this
% script

% pole zero cancellation of the close loop transfer function
closeloop = Ks_clean*sys/(Ks_clean*sys+1);
Tr = minreal(closeloop, 1e-3);

% Get the discrete model
Trd = c2d(Tr, Ts, 'matched');

zeros_Trd = Trd.Z; % zeros
zeros_K = Trd.K; % gain

% Find the non-minimum phase zeros
zeros_Trd = cell2mat(zeros_Trd);
% % nmpz = zeros(zeros_Trd);
% for i = 1:length(zeros_Trd)
%     if abs(zeros_Trd(i)) > 1
%         nmpz = zeros_Trd(i);
%     end
% end

% A, Bs, Bu
[~, A] = tfdata(Trd);
A = cell2mat(A);
Bs = zeros_K;
for i = 1:length(zeros_Trd)
    temp = zeros_Trd(i);
    if abs(temp) > 1
        nmpz = temp;
    else
        Bs = conv(Bs, [1, -temp]);
    end
end
Bu = [1, -nmpz];
Buf = [-nmpz, 1];

F = tf(conv(A, Buf), conv(Bs, (1-nmpz)^2*[1, 0]), Ts);
num_F = cell2mat(F.num);
den_F = cell2mat(F.den);
den_F = den_F(3:end); % The first two elements in den_F are 0, don't konw why

u_pi = filter(num_F, den_F, traj.signals.values);
input_pi = traj;
input_pi.signals.values = u_pi;
