function [Pxx,f,fbin] = pwelchrun(x,na,fsamp, mode)
%PWELCHRUN Compute power spectral density (PSD).
% 
% Calls pwelch to compute the one-sided PSD of signal x, with an averaging
% factor of na and a sampling frequency fsamp.
%
% INPUT
%   x       signal vector
%   na      averaging factor
%   fsamp   sampling frequency (Hz)
%   mode    string argument to define PSD mode, choose among 'onesided
%           (default) or 'twosided'
% 
% OUTPUT
%   Pxx     power spectral density
%   f       frequency grid (Hz)
%   fbin    frequency grid resolution (Hz)

if (nargin < 4)
    mode = 'onesided';
end

%Window
nx = max(size(x));
w = hanning(floor(nx/na));

[Pxx,f] = pwelch(x,w,0,[],fsamp, mode);

fbin = f(2) - f(1);
