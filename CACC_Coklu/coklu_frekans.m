function r = coklu_frekans(p, h, tauc, w)
% Çok araç dinleyen CACC için lineer frekans analizi (doyma yok, sabit gecikme).
% p: tau, Kp, kv = [Kd kv2 kv3 ...], ka = [Kf ka2 ka3 ...], ff_u, N
% Takipçi i, min(i, M) öncülünü dinler; kv(1) radardan (gecikmesiz), diğerleri V2V.
%
% (tau*s + 1)*s^2*V_i = Kp*(V_{i-1} - V_i) - Kp*h*s*V_i + Kd*s*(V_{i-1} - V_i)
%     + sum_{j>=2} kv_j*s*(E*V_{i-j} - V_i) + sum_{j>=1} ka_j*s*E*U_{i-j},   E = exp(-tauc*s)
% U = (tau*s + 1)*s*V (u mesajı) veya s*V (a mesajı).
% p.lider_lag = 0 (varsayılan): liderde u = a, yani U_0 = s*V_0 (CACC_konvoy.slx gibi).
% p.lider_lag = 1: lider de tau*a' + a = u ile sürülür.
%
% Çıktılar:
%   sup_T(i)  : sup|V_i/V_0|, lider-takipçi i arası büyütme
%   sup_G(i)  : sup|V_i/V_{i-1}|, bu konvoydaki araçtan araca büyütme
%   sup_kuyruk: uzun homojen konvoyda araç başına büyüme, sup max|xi|; V_i ~ xi^i V_0
%   D_kararli : her takipçinin kendi kutupları sol yarı düzlemde mi

if nargin < 4, w = logspace(-3, 2, 800); end
tau = p.tau;  Kp = p.Kp;  Kd = p.kv(1);
kv = p.kv;  ka = p.ka;  N = p.N;
lider_lag = isfield(p, 'lider_lag') && p.lider_lag;
M = find(kv ~= 0 | ka ~= 0, 1, 'last');
kv = kv(1:M);  ka = ka(1:M);

s = 1j*w;  E = exp(-tauc*s);
F_arac = s;
if p.ff_u == 1, F_arac = (tau*s + 1).*s; end

% Lider -> takipçi i aktarımları
T = zeros(N + 1, numel(w));  T(1, :) = 1;
r.D_kararli = true;
for i = 1:N
    m = min(i, M);
    top_kv = sum(kv(1:m));
    D_kat = [tau 1 top_kv + Kp*h Kp];
    r.D_kararli = r.D_kararli && all(real(roots(D_kat)) < 0);
    pay = (Kd*s + Kp).*T(i, :);
    for j = 1:m
        if i - j == 0 && ~lider_lag, F = s; else, F = F_arac; end
        if j >= 2, pay = pay + kv(j)*s.*E.*T(i - j + 1, :); end
        pay = pay + ka(j)*s.*F.*E.*T(i - j + 1, :);
    end
    T(i + 1, :) = pay ./ polyval(D_kat, s);
end
r.sup_T = max(abs(T(2:end, :)), [], 2)';
r.sup_G = max(abs(T(2:end, :) ./ T(1:end-1, :)), [], 2)';

% Uzun konvoy: D(s) = sum_j N_j(s)*rho^j,  xi = 1/rho
D = tau*s.^3 + s.^2 + (sum(kv) + Kp*h)*s + Kp;
Nj = zeros(M, numel(w));
Nj(1, :) = Kd*s + Kp + ka(1)*s.*F_arac.*E;
for j = 2:M
    Nj(j, :) = kv(j)*s.*E + ka(j)*s.*F_arac.*E;
end
xi = zeros(size(w));
for k = 1:numel(w)
    if M == 1
        xi(k) = abs(Nj(1, k)/D(k));
    else
        rho = roots([flipud(Nj(:, k)); -D(k)]);
        xi(k) = max(1 ./ abs(rho));
    end
end
r.sup_kuyruk = max(xi);
r.w = w;  r.T = T;
end
