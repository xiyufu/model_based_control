
fs = 1000;              % [Hz] sample frequency of the dSPACE module
Ts = 1/fs;              % [s] sample period

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Generating a multisine
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% exc = Multisine('label','excitation1', ...      % assign a name to the signal 
%                 'fs', fs, ...                   % the sample frequency [Hz]
%                 'fwindow', [0.1 10], ...         % the frequency range of the excitation signal [Hz]
%                 'freqres', 0.01);                  % the frequency resolution [Hz]
% 
% % check whether the multisine has a constant spectrum
% plotSpectrum(exc,'dft');
% 
% % obtain the signal
% [u,t] = signal(exc);
% 
% exc1 = load('exc2_01.mat');
% exc2 = load('exc2_02.mat');
% exc3 = load('exc2_03.mat');
% exc4 = load('exc2_04.mat');
% exc5 = load('exc2_05.mat');
exp3 = importDSPACE('DATA/exp2_03.mat', exc2_03, true);
exp3 = detrend(exp3,{'Position (mm)'});
exp2 = importDSPACE('DATA/exp2_02.mat', exc2_02, true);
exp2 = detrend(exp2,{'Position (mm)'});
exp1 = importDSPACE('DATA/exp2_01.mat', exc2_01, true);
exp1 = detrend(exp1,{'Position (mm)'});
exp4 = importDSPACE('DATA/exp2_04.mat', exc2_04, true);
exp4 = detrend(exp4,{'Position (mm)'});
exp5 = importDSPACE('DATA/exp2_05.mat', exc2_05, true);
exp5 = detrend(exp5,{'Position (mm)'});
clipped_exp1 = clip(exp1,'lastnper',3);
clipped_exp2 = clip(exp2,'lastnper',3);
clipped_exp3 = clip(exp3,'lastnper',3);
clipped_exp4 = clip(exp4,'lastnper',3);
clipped_exp5 = clip(exp5,'lastnper',3);

expc1 = exp1;   
expc1.data_(:,4) = exp1.data_(:,2)*350/180*pi + exp1.data_(:,4);
% expc1.data_(:,3) = expc1.data_(:,3) * 20;
expc2 = exp2;
expc2.data_(:,4) = exp2.data_(:,2)*350/180*pi + exp2.data_(:,4);
% expc2.data_(:,3) = expc2.data_(:,3) * 20;
expc3 = exp3;
expc3.data_(:,4) = exp3.data_(:,2)*350/180*pi + exp3.data_(:,4);
% expc3.data_(:,3) = expc3.data_(:,3) * 20;
expc4 = exp4;
expc4.data_(:,4) = exp4.data_(:,2)*350/180*pi + exp4.data_(:,4);
% expc4.data_(:,3) = expc4.data_(:,3) * 20;
expc5 = exp5;
expc5.data_(:,4) = exp5.data_(:,2)*350/180*pi + exp5.data_(:,4);
% expc5.data_(:,3) = expc5.data_(:,3) * 20;

clipped_expc1 = clip(expc1,'lastnper',3);
clipped_expc2 = clip(expc2,'lastnper',3);
clipped_expc3 = clip(expc3,'lastnper',3);
clipped_expc4 = clip(expc4,'lastnper',3);
clipped_expc5 = clip(expc5,'lastnper',3);


IOcart.input = 'Input speed';
IOcart.output = 'Position (mm)';
Gcart = nonpar_ident(clipped_exp1,clipped_exp2, clipped_exp3, clipped_exp4, clipped_exp5, IOcart, 'time2frf');

IOmass.input = 'Input speed';
IOmass.output = 'Angle (deg)';
Gmass = nonpar_ident(clipped_exp1,clipped_exp2, clipped_exp3, clipped_exp4, clipped_exp5, IOmass, 'time2frf');

Gcombined = nonpar_ident(clipped_expc1,clipped_expc2, clipped_expc3, clipped_expc4,clipped_expc5, IOcart, 'time2frf');

figure; bode(Gcombined);
% figure; bode(Gcart); title('transfer function to cart');
% figure; bode(Gmass); title('transfer function to mass');
figure; bode(Gcart+350/180*pi*Gmass); title('transfer function to mass position');

opts.nh = 3;
opts.nl = 1;
opts.Bh = 0;
opts.Bl = 0;
opts.cORd = 'c';
Gpar = param_ident('data',Gcombined,'method','nllsfdi','options',opts);
figure; bode(Gpar); title('TF to mass pos, paramatric model')

sys = std(Gpar);
[num, den] = ss2tf(sys.A, sys.B, sys.C, sys.D, 1);
sys = tf(num,den);
%% Uncertainties

% G_uncertain = nonpar_ident(clipped_expc1, IOcart, 'time2frf');
% freq = G_uncertain.Frequency;
% freq = 2*pi*freq; % Hz -> rad/s
% val = G_uncertain.ResponseData;
% FRF_uncertain = zeros(size(freq));
% for ii = 1:length(freq)
%     FRF_uncertain(ii) = val(1,1,ii);
% end
% 
% [mag, phase,wout] = bode(Gpar, freq);
% FRF_model = mag.*exp(1i*phase);
% FRF_fitted = zeros(size(freq));
% for ii = 1:length(freq)
%     FRF_fitted(ii) = FRF_model(1,1,ii);
% end
% 
% uncertainty = FRF_fitted - FRF_uncertain;
% lm = abs(uncertainty./FRF_fitted);
% plot(log10(freq), 20*log10(abs(lm)));

%% Time domain validation
t = expc1.data_(:, 1);
u = expc1.data_(:, 3);
y_real = expc1.data_(:, 4);
y_sim = lsim(sys, u, t);
e = y_real - y_sim;

figure; hold on;
plot( t(1:end/3), y_real(1:end/3));
plot( t(1:end/3), y_sim(1:end/3), 'r');
