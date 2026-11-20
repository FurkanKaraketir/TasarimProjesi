%% CACC KONVOY - YÖNETİM DOSYASI
% 1 lider + 3 takipçi, 3. mertebe araç modeli, time-headway politikası,
% V2V feedforward ve haberleşme gecikmesi tauc (sabit veya zamanla değişen).
% Aynı h ile ACC (Kf = 0) da çalıştırılır ve iki yapı karşılaştırılır.
% Kullanım: Bölüm 1-3'teki değerleri değiştir, dosyayı çalıştır (F5).
% Model (CACC_konvoy.slx) yoksa cacc_model_olustur.m ile otomatik kurulur.
% Denklemler: teknik/1D_Arac_Dinamigi_Denklemler.html

clear; clc;

%% 1. Araç ve kontrol parametreleri
tau   = 0.5;     % Aktüatör zaman sabiti (s)
Kp    = 0.4;     % Oransal kazanç (1/s^2)
Kd    = 0.8;     % Türevsel kazanç (1/s)
h     = 0.6;     % Zaman aralığı / time-headway (s)
Kf    = 0.8;     % Feedforward kazancı (-); 0 = ACC
ff_u  = 1;       % V2V mesajı: 1 = öncülün istenen ivmesi u, 0 = ölçülen ivmesi a
d0    = 5;       % Durma mesafesi (m)
L     = 4.5;     % Araç uzunluğu (m)
u_min = -5;      % En büyük frenleme (m/s^2)
u_max = 3;       % En büyük hızlanma (m/s^2)

%% 2. V2V haberleşme gecikmesi
tauc_mod   = 'sabit';   % 'sabit' veya 'degisken'
tauc_sabit = 0.100;     % Sabit gecikme (s)
% 'degisken' için: [tauc_alt, tauc_ust] aralığında, her tutma_suresi boyunca
% sabit kalan rastgele gecikme (ortalama yaklaşık (alt + üst)/2)
tauc_alt     = 0.050;   % (s)
tauc_ust     = 0.150;   % (s)
tutma_suresi = 0.5;     % Gecikmenin sabit kaldığı süre (s)
tohum        = 1;       % Rastgele sayı tohumu (tekrarlanabilir sonuç için)

%% 3. Senaryo
v0     = 15;     % Başlangıç hızı, bütün araçlar (m/s)
T_stop = 40;     % Simülasyon süresi (s)
dt     = 0.01;   % Sabit adım (s)

% Lider ivme profili: her satır [başlangıç(s) bitiş(s) ivme(m/s^2)]
lider_profil = [ 5   8   1     % 5-8 s: hızlanma
                12  14  -2 ];  % 12-14 s: frenleme

harita_ciz = true;              % h - tauc string stability haritası (analitik)
harita_h    = 0.2:0.1:1.6;      % (s)
harita_tauc = 0:0.025:0.3;      % (s)

%% 4. Türetilen değerler (genelde değiştirmeye gerek yok)
model = 'CACC_konvoy';
N     = 3;                                   % takipçi sayısı
d_ss  = d0 + h*v0;                           % başlangıçtaki istenen mesafe (m)
p0    = -(1:N) * (d_ss + L);                 % sıfır hatayla başlangıç konumları (m)

% Lider profili -> From Workspace matrisi [t a]
lider_ivme = [0 0];
for k = 1:size(lider_profil, 1)
    t1 = lider_profil(k, 1); t2 = lider_profil(k, 2); a = lider_profil(k, 3);
    lider_ivme = [lider_ivme; t1 0; t1 a; t2 a; t2 0]; %#ok<AGROW>
end
lider_ivme = [lider_ivme; T_stop 0];

% Gecikme profili -> From Workspace matrisi [t tauc] (basamaklı, enterpolasyonsuz)
switch tauc_mod
    case 'sabit'
        tauc_profil = [0 tauc_sabit; T_stop tauc_sabit];
    case 'degisken'
        rng(tohum);
        t_g = (0:tutma_suresi:T_stop)';
        tauc_profil = [t_g, tauc_alt + (tauc_ust - tauc_alt)*rand(size(t_g))];
    otherwise
        error('tauc_mod ''sabit'' veya ''degisken'' olmalı.');
end
tauc_ort = mean(tauc_profil(:, 2));
tauc_max = max(tauc_profil(:, 2)) + dt;     % gecikme bloğunun tampon sınırı

%% 5. Ön kontroller (analitik, ortalama gecikmeyle)
if Kd + Kp*h <= tau*Kp
    warning('Tek araç kararsız: Kd + Kp*h = %.2f <= tau*Kp = %.2f', Kd + Kp*h, tau*Kp);
end
sup_cacc = sup_gamma(tau, Kp, Kd, h, Kf, ff_u, tauc_ort);
sup_acc  = sup_gamma(tau, Kp, Kd, h, 0, ff_u, 0);

%% 6. Simülasyon: önce ACC (Kf = 0), sonra CACC
klasor = fileparts(mfilename('fullpath'));
if ~isfile(fullfile(klasor, [model '.slx']))
    cacc_model_olustur(model);
end
load_system(fullfile(klasor, [model '.slx']));

Kf_cacc = Kf;
Kf = 0;        out_acc  = sim(model);
Kf = Kf_cacc;  out_cacc = sim(model);

%% 7. Metrikler
t = out_cacc.tout;
m_acc  = metrikler(out_acc, u_min, u_max);
m_cacc = metrikler(out_cacc, u_min, u_max);

mesaj_turu = {'ölçülen ivme a', 'istenen ivme u'};
fprintf('=== CACC konvoy | tau=%.2f s, Kp=%.2f, Kd=%.2f, h=%.2f s, Kf=%.2f, mesaj: %s ===\n', ...
    tau, Kp, Kd, h, Kf, mesaj_turu{(ff_u == 1) + 1});
fprintf('V2V gecikmesi: %s, ortalama %.0f ms (min %.0f, max %.0f ms)\n', tauc_mod, ...
    1e3*tauc_ort, 1e3*min(tauc_profil(:, 2)), 1e3*max(tauc_profil(:, 2)));
fprintf('sup|Gamma|  ACC = %.3f (%s)   CACC = %.3f (%s)\n', ...
    sup_acc, durum(sup_acc), sup_cacc, durum(sup_cacc));
if strcmp(tauc_mod, 'degisken')
    fprintf('Not: CACC sup|Gamma| ortalama gecikmeyle hesaplandı; değişken gecikmede geçerli ölçüt zaman alanı oranlarıdır.\n');
end

satirlar = {'Takipci 1', 'Takipci 2', 'Takipci 3'};
sonuc = table(m_acc.a_oran', m_cacc.a_oran', m_acc.e_max', m_cacc.e_max', ...
    m_acc.e_rmse', m_cacc.e_rmse', m_acc.d_min', m_cacc.d_min', ...
    'VariableNames', {'a_orani_ACC', 'a_orani_CACC', 'max_e_ACC', 'max_e_CACC', ...
                      'RMSE_e_ACC', 'RMSE_e_CACC', 'min_d_ACC', 'min_d_CACC'}, ...
    'RowNames', satirlar);
disp(sonuc);
degisim = 100*(sum(m_cacc.e_rmse)/sum(m_acc.e_rmse) - 1);   % negatif = CACC daha iyi
fprintf('CACC''nin toplam RMSE(e) değeri ACC''ye göre: %+.0f%%\n', degisim);
if any([m_acc.d_min m_cacc.d_min] <= 0), warning('Çarpışma: en az bir mesafe sıfırın altına indi.'); end
if m_acc.doyma || m_cacc.doyma
    warning('İvme sınırına ulaşıldı; lineer analiz (sup|Gamma|) bu koşulda geçerli değil.');
end

%% 8. Grafikler
etiket_arac = {'Lider', 'Takipçi 1', 'Takipçi 2', 'Takipçi 3'};
renk = lines(3);
figure('Name', 'CACC konvoy', 'Color', 'w');
tiledlayout(2, 2, 'TileSpacing', 'compact');

nexttile; plot(t, out_cacc.v_log, 'LineWidth', 1.2); grid on;
ylabel('Hız (m/s)'); title('CACC: hızlar'); legend(etiket_arac, 'Location', 'best');

nexttile; plot(t, out_cacc.a_log, 'LineWidth', 1.2); grid on;
ylabel('İvme (m/s^2)'); title('CACC: ivmeler'); legend(etiket_arac, 'Location', 'best');

nexttile; hold on; grid on;
for k = 1:N
    plot(t, out_acc.e_log(:, k), '--', 'Color', renk(k, :), 'LineWidth', 1);
    plot(t, out_cacc.e_log(:, k), '-', 'Color', renk(k, :), 'LineWidth', 1.4);
end
ylabel('e = d - d_{des} (m)'); xlabel('Zaman (s)');
title('Mesafe hatası: ACC (kesikli) / CACC (düz)');
legend({'e_{12} ACC', 'e_{12} CACC', 'e_{23} ACC', 'e_{23} CACC', 'e_{34} ACC', 'e_{34} CACC'}, ...
    'Location', 'best', 'NumColumns', 2);

nexttile; stairs(out_cacc.tout, 1e3*out_cacc.tauc_log, 'LineWidth', 1.2); grid on;
ylabel('\tau_c (ms)'); xlabel('Zaman (s)'); title('V2V gecikmesi');
ylim([0, 1e3*tauc_max*1.2 + 1]);

%% 9. Gecikme - headway haritası (analitik, sabit gecikme)
if harita_ciz
    harita = zeros(numel(harita_h), numel(harita_tauc));
    for i = 1:numel(harita_h)
        for j = 1:numel(harita_tauc)
            harita(i, j) = sup_gamma(tau, Kp, Kd, harita_h(i), Kf, ff_u, harita_tauc(j));
        end
    end
    figure('Name', 'Gecikme-headway haritası', 'Color', 'w');
    imagesc(1e3*harita_tauc, harita_h, harita); axis xy; colorbar;
    clim([1 max(1.3, max(harita(:)))]);
    hold on;
    contour(1e3*harita_tauc, harita_h, harita, [1.001 1.001], 'k', 'LineWidth', 2);
    plot(1e3*tauc_ort, h, 'wp', 'MarkerSize', 14, 'MarkerFaceColor', 'w');
    xlabel('\tau_c (ms)'); ylabel('h (s)');
    title(sprintf('sup|\\Gamma(j\\omega)| (siyah çizgi = sınır, yıldız = seçilen nokta), Kf = %.2f', Kf));
end

%% Yerel fonksiyonlar
function g = sup_gamma(tau, Kp, Kd, h, Kf, ff_u, tauc)
% Homojen konvoyda Gamma(s) = V_i/V_{i-1} tepe değeri.
% V2V terimi: a mesajı -> Kf*s^2,  u mesajı -> Kf*(tau*s + 1)*s^2  (u = (tau*s+1)*a)
w = logspace(-3, 2, 5000);
s = 1j*w;
ff = Kf * ((1 - ff_u) + ff_u*(tau*s + 1)) .* s.^2 .* exp(-tauc*s);
G = (ff + Kd*s + Kp) ./ (tau*s.^3 + s.^2 + (Kd + Kp*h)*s + Kp);
g = max(abs(G));
end

function m = metrikler(out, u_min, u_max)
a = out.a_log;  e = out.e_log;  d = out.d_log;
a_max    = max(abs(a));
m.a_oran = a_max(2:end) ./ a_max(1:end-1);   % > 1 ise bozucu büyüyor
m.e_max  = max(abs(e));
m.e_rmse = sqrt(mean(e.^2));
m.d_min  = min(d);
m.doyma  = any(a(:, 2:end) >= 0.99*u_max | a(:, 2:end) <= 0.99*u_min, 'all');
end

function s = durum(g)
if g <= 1 + 1e-3, s = 'string-stable'; else, s = 'string-UNSTABLE'; end
end
