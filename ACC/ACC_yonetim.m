%% ACC KONVOY - YÖNETİM DOSYASI
% 1 lider + 3 takipçi, 3. mertebe araç modeli, sabit time-headway politikası.
% Kullanım: Bölüm 1-2'deki değerleri değiştir, dosyayı çalıştır (F5).
% Model (ACC_konvoy.slx) yoksa acc_model_olustur.m ile otomatik kurulur.
% Denklemler: teknik/1D_Arac_Dinamigi_Denklemler.html

clear; clc;

%% 1. Araç ve kontrol parametreleri
tau   = 0.5;     % Aktüatör zaman sabiti (s)
Kp    = 0.4;     % Oransal kazanç (1/s^2)
Kd    = 0.8;     % Türevsel kazanç (1/s)
h     = 1.2;     % Zaman aralığı / time-headway (s); ACC için h >= 2*tau gerekir
d0    = 5;       % Durma mesafesi (m)
L     = 4.5;     % Araç uzunluğu (m)
u_min = -5;      % En büyük frenleme (m/s^2)
u_max = 3;       % En büyük hızlanma (m/s^2)

%% 2. Senaryo
v0     = 15;     % Başlangıç hızı, bütün araçlar (m/s)
T_stop = 40;     % Simülasyon süresi (s)
dt     = 0.01;   % Sabit adım (s)

% Lider ivme profili: her satır [başlangıç(s) bitiş(s) ivme(m/s^2)]
lider_profil = [ 5   8   1     % 5-8 s: hızlanma
                12  15  -1 ];  % 12-15 s: yavaşlama

%% 3. Türetilen değerler (genelde değiştirmeye gerek yok)
model = 'ACC_konvoy';
N     = 3;                                   % takipçi sayısı
d_ss  = d0 + h*v0;                           % başlangıçtaki istenen mesafe (m)
p0    = -(1:N) * (d_ss + L);                 % sıfır hatayla başlangıç konumları (m)

% Profil -> From Workspace matrisi [t a]; aynı zamanda iki nokta = basamak
lider_ivme = [0 0];
for k = 1:size(lider_profil, 1)
    t1 = lider_profil(k, 1); t2 = lider_profil(k, 2); a = lider_profil(k, 3);
    lider_ivme = [lider_ivme; t1 0; t1 a; t2 a; t2 0]; %#ok<AGROW>
end
lider_ivme = [lider_ivme; T_stop 0];

%% 4. Ön kontroller (analitik)
% Tek araç kararlılığı (Routh): Kd + Kp*h > tau*Kp
if Kd + Kp*h <= tau*Kp
    warning('Tek araç kararsız: Kd + Kp*h = %.2f <= tau*Kp = %.2f', Kd + Kp*h, tau*Kp);
end
% String stability: Gamma(s) = (Kd*s + Kp) / (tau*s^3 + s^2 + (Kd + Kp*h)*s + Kp)
w = logspace(-3, 2, 5000);
s = 1j*w;
Gamma = (Kd*s + Kp) ./ (tau*s.^3 + s.^2 + (Kd + Kp*h)*s + Kp);
[sup_Gamma, k_tepe] = max(abs(Gamma));
string_stable = sup_Gamma <= 1 + 1e-6;

%% 5. Simülasyon
klasor = fileparts(mfilename('fullpath'));
if ~isfile(fullfile(klasor, [model '.slx']))
    acc_model_olustur(model);
end
load_system(fullfile(klasor, [model '.slx']));
out = sim(model);

t = out.tout;
v = out.v_log;      % [lider T1 T2 T3] (m/s)
a = out.a_log;      % [lider T1 T2 T3] (m/s^2)
d = out.d_log;      % [d12 d23 d34] (m)
e = out.e_log;      % [e12 e23 e34] (m)

%% 6. Metrikler
a_max   = max(abs(a));                       % her aracın en büyük |ivme|
a_oran  = a_max(2:end) ./ a_max(1:end-1);    % ardışık oran: > 1 ise bozucu büyüyor
e_max   = max(abs(e));
e_rmse  = sqrt(mean(e.^2));
d_min   = min(d);
e_son   = e(end, :);

fprintf('=== ACC konvoy | tau=%.2f s, Kp=%.2f, Kd=%.2f, h=%.2f s ===\n', tau, Kp, Kd, h);
if string_stable
    fprintf('sup|Gamma| = %.3f  -> string-STABLE\n', sup_Gamma);
else
    fprintf('sup|Gamma| = %.3f @ w = %.2f rad/s  -> string-UNSTABLE\n', sup_Gamma, w(k_tepe));
end
sonuc = table(a_max(2:end)', a_oran', e_max', e_rmse', d_min', e_son', ...
    'VariableNames', {'max_a', 'a_orani', 'max_e', 'RMSE_e', 'min_d', 'son_e'}, ...
    'RowNames', {'Takipci 1', 'Takipci 2', 'Takipci 3'});
disp(sonuc);
if any(d_min <= 0), warning('Çarpışma: en az bir mesafe sıfırın altına indi.'); end
if any(a(:, 2:end) >= 0.99*u_max | a(:, 2:end) <= 0.99*u_min, 'all')
    warning('İvme sınırına ulaşıldı; lineer analiz (sup|Gamma|) bu koşulda geçerli değil.');
end

%% 7. Grafikler
etiket_arac = {'Lider', 'Takipçi 1', 'Takipçi 2', 'Takipçi 3'};
etiket_ara  = {'d_{12}', 'd_{23}', 'd_{34}'};
figure('Name', 'ACC konvoy', 'Color', 'w');
tiledlayout(2, 2, 'TileSpacing', 'compact');

nexttile; plot(t, v, 'LineWidth', 1.2); grid on;
ylabel('Hız (m/s)'); title('Hızlar'); legend(etiket_arac, 'Location', 'best');

nexttile; plot(t, a, 'LineWidth', 1.2); grid on;
ylabel('İvme (m/s^2)'); title('İvmeler'); legend(etiket_arac, 'Location', 'best');

nexttile; plot(t, d, 'LineWidth', 1.2); grid on;
ylabel('Mesafe (m)'); xlabel('Zaman (s)'); title('Araçlar arası mesafe');
legend(etiket_ara, 'Location', 'best');

nexttile; plot(t, e, 'LineWidth', 1.2); grid on;
ylabel('e = d - d_{des} (m)'); xlabel('Zaman (s)'); title('Mesafe hatası');
legend(etiket_ara, 'Location', 'best');
