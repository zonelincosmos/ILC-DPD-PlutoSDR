% ILC-DPD on ADALM-Pluto SDR  (hardware-in-loop, model-free)
%
% Iterative Learning Control digital predistortion with a real Pluto SDR
% TX->RX loopback. Multiplicative complex-ratio update:
%     P_{k+1} = P_k .* ((1-mu) + mu * T ./ M_avg)
%
% Algorithm reference (method is from published work, independently
% re-implemented here for hardware):
%   J. Chani-Cahuana et al., "Iterative Learning Control for RF Power
%   Amplifier Linearization," IEEE Trans. MTT, 64(9), 2016.
%   M. Schoukens et al., "Obtaining the Preinverse of a Power Amplifier
%   Using Iterative Learning Control," IEEE Trans. MTT, 65(11), 2017.
%
% License: MIT (see LICENSE).

%% ILC-DPD (Iterative Learning Control DPD), hardware-in-loop on Pluto SDR
clear all;clc;close all

%% -- load waveform --
load('RefSignal.mat')
RefSignalX = RefSignal;

%% -- Setup Pluto SDR --
CenterFrequency    = 2000; % MHz
BasebandSampleRate = 1;  % MHz
TxGain = -3; 
RxGain =  0;

x    = RefSignalX;
N    = length(x);
pad          = 1000;
tx_len       = N + 2*pad;
n_frames_mul = 4;                  % capture 4 periods (sync to clean middle copy)
frameSize    = tx_len * n_frames_mul;

rx = sdrrx('Pluto', ...
    'CenterFrequency',    (CenterFrequency) * 1e6, ...
    'SamplesPerFrame',    frameSize, ...
    'OutputDataType',     'double', ...
    'BasebandSampleRate', BasebandSampleRate * 1e6, ...
    'EnableBurstMode',    false, ...
    'GainSource',         'Manual', ...
    'Gain',               RxGain);
disp(info(rx));

tx = sdrtx('Pluto', ...
    'CenterFrequency',    (CenterFrequency) * 1e6, ...
    'Gain',              TxGain, ...
    'BasebandSampleRate', BasebandSampleRate * 1e6);

fprintf('Signal N = %d, TX len = %d, frameSize = %d\n', N, tx_len, frameSize);

%% Cubic Lagrange Farrow coefficient matrix (fractional resync).
Cfarrow = [ -1/6,  1/2, -1/2,  1/6;
             1/2, -1,    1/2,  0;
            -1/3, -1/2,  1,   -1/6;
             0,    1,    0,    0   ];

%% -- ILC-DPD parameters --
G_target = 1.0;
mu       = 0.5;                    % learning rate
N_avg    = 16;                   % I/Q averaged captures per iteration
n_iter   = 10;

%% -- Baseline capture (no DPD, peak=1) --
fprintf('\n--- Baseline capture (no DPD) ---\n');
[y0, soPrev] = captureAndSync(x, rx, tx, RefSignalX, pad, frameSize, Cfarrow, []);
NMSE0 = 10*log10(mean(abs(y0 - G_target*x).^2) / mean(abs(G_target*x).^2));
fprintf('Baseline NMSE = %.2f dB\n', NMSE0);

%% -- ILC iterations --
P    = x;                          % P_0 = x
T    = G_target * x;       % target output (no Psat clip)
nmse = zeros(n_iter,1);
for k = 1:n_iter
    % (a) I/Q averaged capture of PA(P)
    Msum = zeros(N,1);
    for j = 1:N_avg
        [yj, soPrev] = captureAndSync(P, rx, tx, RefSignalX, pad, frameSize, Cfarrow, soPrev);
        Msum = Msum + yj;
    end
    M_avg = Msum / N_avg;

    % (b) NMSE of current P_k (measured)
    nmse(k) = 10*log10(mean(abs(M_avg - G_target*x).^2) / mean(abs(G_target*x).^2));
    fprintf('Iter %d  NMSE = %.2f dB   max|P| = %.3f\n', k, nmse(k), max(abs(P)));

    % (c) Multiplicative complex-ratio ILC update
    corr = T ./ max(abs(M_avg),1e-12) .* exp(-1j*angle(M_avg));
    P    = P .* ((1-mu) + mu*corr);

    % (d) Per-iteration sync / linearization check
    fh = figure('Visible','off','Position',[100 100 1100 700]);
    subplot(2,1,1);
    plot(abs(RefSignalX)); hold on; plot(abs(M_avg));
    legend('|RefSignalX|','|M_{avg}|'); grid on;
    title(sprintf('ILC iter %d  NMSE=%.2f dB  max|P|=%.3f', k, nmse(k), max(abs(P))));
    subplot(2,1,2);
    plot(abs(x), abs(M_avg), '.'); hold on; plot(abs(x), abs(x), 'k-');
    grid on; xlabel('|RefSignalX|'); ylabel('|M_{avg}|'); title('AM-AM');
    exportgraphics(fh, sprintf('ilc_iter_%02d.png', k));
    close(fh);
end
y_final = M_avg;

%% -- Release hardware --
release(rx);
release(tx);

%% -- Results --
figure;
plot(0:n_iter, [NMSE0; nmse], '-o'); grid on;
xlabel('Iteration'); ylabel('NMSE (dB)');
title('ILC-DPD convergence');

figure;
plot(abs(x), abs(y0),      '.'); hold on;
plot(abs(x), abs(y_final), '.');
plot(abs(x), abs(x),       'k-');
grid on; xlabel('|RefSignalX|'); ylabel('|CompressedSignalY|');
legend('Before DPD','After DPD','Ideal','Location','NorthWest');
title('ILC-DPD  AM-AM');


sa1 = dsp.SpectrumAnalyzer('SampleRate',BasebandSampleRate*1e6,'SpectralAverages',5, ...
    'ShowLegend',true,'ChannelNames',{'RefSignalX','Without ILC-DPD','With ILC-DPD'});
sa1.YLimits = [-10,100];
sa1([RefSignalX(:), y0(:), y_final(:)]);

%%
% =============================================================================
% Local functions
% =============================================================================

function [CompressedSignalY, SyncOffset] = captureAndSync(s, rx, tx, RefSignalX, pad, frameSize, Cfarrow, soPrev)
% Transmit zero-padded s via Pluto, capture one frame, integer (xcorr) +
% fractional (Farrow) sync, return DC-removed RMS-normalized CompressedSignalY (Nx1).
% soPrev: previous SyncOffset for tracking ([] = full search on baseline).
    txWave = complex([zeros(pad,1); s(:); zeros(pad,1)]);
    release(tx);
    tx.transmitRepeat(txWave);

    for i = 1:2, rx(); end                 % warm-up: discard settling frames
    [d, ~, of] = rx();
    if of, fprintf('  (RX overflow)\n'); end
    dataVec = d(:);

    % --- Integer sync: collect candidate xcorr peaks (Farrow-safe range) ---
    [corr_vals, lags] = xcorr(dataVec, RefSignalX);
    cv  = abs(corr_vals);
    so  = lags + 1;
    Nr     = length(RefSignalX);
    period = 2*pad + numel(s);
    valid = (so >= period + 2) & (so <= 3*period - Nr - 2);
    cv(~valid) = -inf;
    gmax = max(cv);
    cand = [];
    for p = 1:5
        [pk, ii] = max(cv);
        if pk < 0.6 * gmax, break; end
        cand(end+1) = so(ii);              %#ok<AGROW>
        lo = max(1, ii - round(Nr/2));
        hi = min(numel(cv), ii + round(Nr/2));
        cv(lo:hi) = -inf;
    end

    % --- Pick (SyncOffset, FracDelay) minimising post-Farrow envelope error ---
    n      = (0:Nr-1).';
    RmsX0  = rms(RefSignalX);
    absRef = abs(RefSignalX);
    muGrid = (-1:0.002:1).';
    bestErr = inf; SyncOffset = cand(1); FracDelay = 0;
    for ci = 1:numel(cand)
        so_c = cand(ci);
        for kk = 1:numel(muGrid)
            yk = farrowEval(dataVec, so_c, n, muGrid(kk), Cfarrow);
            yk = yk - mean(yk);
            yk = yk * (RmsX0 / rms(yk));
            e  = norm(absRef - abs(yk));
            if e < bestErr
                bestErr = e; SyncOffset = so_c; FracDelay = muGrid(kk);
            end
        end
    end
    fprintf('  SyncOffset = %d, FracDelay = %.4f, err = %.4g\n', ...
            SyncOffset, FracDelay, bestErr);

    CompressedSignalY = farrowEval(dataVec, SyncOffset, n, FracDelay, Cfarrow);
    CompressedSignalY = CompressedSignalY - mean(CompressedSignalY);              % Remove DC
    CompressedSignalY = CompressedSignalY * (rms(RefSignalX) / rms(CompressedSignalY)); % RMS normalize to RefSignalX scale
    phi  = angle(CompressedSignalY' * RefSignalX);            % constant loopback phase rotation
    CompressedSignalY = CompressedSignalY * exp(1j*phi);             % de-rotate to align with RefSignalX
end

% --- Cubic Farrow fractional resampler ---
function y = farrowEval(dataVec, SyncOffset, n, mu, Cfarrow)
    di = floor(mu);
    f  = mu - di;
    m  = SyncOffset + n + di;
    X  = [dataVec(m-1), dataVec(m), dataVec(m+1), dataVec(m+2)];
    P  = X * Cfarrow.';
    y  = ((P(:,1)*f + P(:,2))*f + P(:,3))*f + P(:,4);
end
