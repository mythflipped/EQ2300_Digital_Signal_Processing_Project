%% close all windows
clc
clear;
close all;

%% Task 2 : Low-pass filter design

fc = 1/16; %cut-off frequency
fs = 1/8; %suppressed frequency
N = 51; % Length = 51; type I FIR filter;
pad_N = N +10000; % length afer zero padding;


df = 1/N; 
df_pad = 1/pad_N;


f = (0:df:1-df)';
f_pad = (0:df_pad:1-df_pad)';


% Generate window function
%w = window(@bartlett, N);
w = window(@hamming, N);
%w = window(@hanning, N);
%w = window(@blackman, N);
%w = window(@chebwin, N);
%w = window(@kaiser, N);


H = ones(N,1);
H(abs(f)>fc & abs(1-f)>fc)=0;

h = real(ifft(H));
h = fftshift(h);

%get impulse response

h_lpf = h.*w;
h_lpf_pad = cat(1, h_lpf, zeros(pad_N - N,1));

figure(1)
stem(0:N-1, h_lpf);
title('Impulse response')


%get frequency response
H_LPF = abs(fft(h_lpf));
H_LPF_PAD = abs(fft(h_lpf_pad));


%normalized frequency response
H_LPF = H_LPF / max(H_LPF);
H_LPF_PAD = H_LPF_PAD / max(H_LPF_PAD);

figure(2)

plot(f_pad(f_pad<0.5), H_LPF_PAD(f_pad<0.5))
ylim([0 1.3])
hold on;
plot([fc fc],[0 1.3], '-- k')
title('Frequency response')
grid on; 
grid minor;

%normalized in dB scale
H_LPF_dB=20*log10(abs(H_LPF));
H_LPF_PAD_dB = mag2db(H_LPF_PAD);

%draw half of normalized frequency
f_half = f(f<0.5);
f_pad_half = f_pad(f_pad<0.5);
H_LPF_dB_half = H_LPF_dB(f<0.5);
H_LPF_PAD_dB_half = H_LPF_PAD_dB(f_pad<0.5);

figure(3)
hold on;
grid on; 

plot(f_pad_half, H_LPF_PAD_dB_half,'linewidth', 1.5); % draw the frequency response for filter(with zero padding)
plot(f_half, H_LPF_dB_half, '.black','linewidth', 1.5); % draw the frequency response for the filter(without zero padding)

xline(fc, '-- red','linewidth', 1.5)
xline(fs, '-- blue','linewidth', 1.5)
yline(-40, '-- black','linewidth', 1.5)

xlabel('Normalized frequency \nu') 
ylabel('Magnitude (dB)')

legend({'frequncy response(zero padding)', ...
    'frequency response(without zero padding)', 'cutoff frequency', ...
    'stop band frequency', 'supression level (dB) '})

%% Task 3 : High-pass filter design

M = (N-1)/2;

delta = zeros(N,1);
delta(M+1) = 1;

hh=delta-h_lpf;
h_h_pad = cat(1, hh, zeros(pad_N - N,1));

figure(4)

stem(0:N-1, hh);
title ('impulse response');

HH = abs(fft(hh));
H_H_PAD = abs(fft(h_h_pad));

H_H_PAD_dB = mag2db(H_H_PAD(f_pad<0.5));
HH_dB = mag2db(HH(f<0.5));


figure(5);
hold on
plot(f_pad(f_pad<0.5), H_H_PAD_dB,'r')
plot(f(f<0.5), HH_dB, '.black');
xline(fs, '-- blue','linewidth', 1.5);

xlabel('Normalized frequency \nu') % cycles/sample
ylabel('Magnitude(dB)')
grid on;
box on;

legend('Frequency response(zero padding)', ...
    'Frequency response (without zero padding)'...
    ,'Stop band frequency')


%% Task 4: Quantization

F=12;
% quantized impulsee response
h_lpf_Q = round(h_lpf*2^F);
h_lpf_Q = h_lpf_Q*2^-F;
h_lpf_pad_Q = cat(1, h_lpf_Q, zeros(pad_N - N,1));

%frequency response

H_LPF_Q = abs(fft(h_lpf_Q));
H_LPF_PAD_Q = abs(fft(h_lpf_pad_Q));

%normalized frequency response

H_LPF_Q = H_LPF_Q / max(H_LPF_Q);
H_LPF_PAD_Q = H_LPF_PAD_Q / max(H_LPF_PAD_Q);


H_LFP_Q_dB = mag2db(H_LPF_Q);
H_LPF_PAD_Q_dB = mag2db(H_LPF_PAD_Q);


H_LFP_Q_dB = H_LFP_Q_dB(f<0.5);
H_LPF_PAD_Q_dB = H_LPF_PAD_Q_dB(f_pad<0.5);

figure(6)
hold on
plot(f_pad(f_pad<0.5), H_LPF_PAD_Q_dB,'linewidth', 1.5);
plot(f(f<0.5),H_LFP_Q_dB,'.black','linewidth', 1.5);

xline(fc, '-- red','linewidth', 1.5)
xline(fs, '-- blue','linewidth', 1.5)
yline(-40, '-- black','linewidth', 1.5)

xlabel('Normalised frequency \nu') % cycles/sample
ylabel('Magnitude(dB)')
% title('Frequency respo')
legend({'frequncy response(zero padding)', ...
    'frequency response(without zero padding)', 'cutoff frequency', ...
    'stop band frequency', 'supression level (dB) '})
grid on;
grid minor;
box on;


% h_quant = 2^(-F) * round(h * 2^F);
% H_quant = fft(h_quant, N);
% H_quant_dB = 20 * log10(abs(H_quant));

%% Task 5:  signal to quantization noise ratio (SQNR)

E_up = sum(H_LPF.^2);
E_down = sum((H_LPF - H_LPF_Q).^2);

SQNR = 10*log10(E_up / E_down)
