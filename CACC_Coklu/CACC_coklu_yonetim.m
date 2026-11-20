%% ÇOK ARAÇ DİNLEYEN CACC - YÖNETİM DOSYASI
% 1 lider + 3 takipçi. Her takipçi en fazla M öncülünü dinler (Bouadi et al., 2024
% yapısı): en yakın aracın mesafe ve hız farkı radardan, diğerleri V2V üzerinden.
% M = 1 iken mevcut CACC_konvoy.slx ile aynı modeldir.
% Kullanım: Bölüm 1-3'teki değerleri değiştir, dosyayı çalıştır (F5).
% Toolbox gerekmez; model coklu_sim.m, analiz coklu_frekans.m içindedir.

clear; clc;

%% 1. Araç ve kontrol parametreleri
p.tau   = 0.5;     % Aktüatör zaman sabiti (s)
p.Kp    = 0.4;     % Oransal kazanç (1/s^2)
p.kv    = [0.8 0 0];       % Hız farkı kazançları (1/s): [Kd, 2. öncül, 3. öncül]
p.ka    = [0.5 0.2 0.1];   % İvme kazançları (-): [Kf, 2. öncül, 3. öncül]
p.ff_u  = 1;       % V2V mesajı: 1 = öncülün istenen ivmesi u, 0 = ölçülen ivmesi a
p.d0    = 5;       % Durma mesafesi (m)
p.L     = 4.5;     % Araç uzunluğu (m)
p.u_min = -5;      % En büyük frenleme (m/s^2)
p.u_max = 3;       % En büyük hızlanma (m/s^2)
p.v0    = 15;      % Başlangıç hızı, bütün araçlar (m/s)
p.N     = 3;       % Takipçi sayısı
p.lider_lag = 0;   % 0 = liderde a = u (CACC_konvoy.slx gibi), 1 = lider de tau gecikmeli

h    = 0.6;        % Zaman aralığı / time-headway (s)
tauc = 0.100;      % Sabit V2V gecikmesi (s); 0 veya en az dt

%% 2. Senaryo
T_stop = 40;       % Simülasyon süresi (s)
dt     = 0.01;     % Sabit adım (s)
% Lider ivme profili: her satır [başlangıç(s) bitiş(s) ivme(m/s^2)]
lider_profil = [ 5   8   1     % 5-8 s: hızlanma
                12  14  -2 ];  % 12-14 s: frenleme

%% 3. Karşılaştırma için tek öncüllü yapı
p1 = p;  p1.kv = p.kv(1);  p1.ka = sum(p.ka);

%% 4. Türetilen değerler
lider_ivme = [0 0];
for k = 1:size(lider_profil, 1)
    t1 = lider_profil(k, 1); t2 = lider_profil(k, 2); a = lider_profil(k, 3);
    lider_ivme = [lider_ivme; t1 0; t1 a; t2 a; t2 0]; %#ok<AGROW>
end
lider_ivme = [lider_ivme; T_stop 0];

%% 5. Frekans analizi
r  = coklu_frekans(p, h, tauc);
r1 = coklu_frekans(p1, h, tauc);
fprintf('=== Çok araç dinleyen CACC | h=%.2f s, tauc=%.0f ms ===\n', h, 1e3*tauc);
fprintf('Çok öncül : sup|V_i/V_{i-1}| = [%s], uzun konvoy = %.3f\n', num2str(r.sup_G, '%.3f '), r.sup_kuyruk);
fprintf('Tek öncül : sup|V_i/V_{i-1}| = [%s], uzun konvoy = %.3f\n', num2str(r1.sup_G, '%.3f '), r1.sup_kuyruk);

%% 6. Simülasyon ve metrikler
o  = coklu_sim(p, h, tauc, lider_ivme, T_stop, dt);
o1 = coklu_sim(p1, h, tauc, lider_ivme, T_stop, dt);
m  = metrikler(o, p);
m1 = metrikler(o1, p);

sonuc = table(m1.a_oran', m.a_oran', m1.e_max', m.e_max', m1.d_min', m.d_min', ...
    'VariableNames', {'a_orani_tek', 'a_orani_cok', 'max_e_tek', 'max_e_cok', 'min_d_tek', 'min_d_cok'}, ...
    'RowNames', {'Takipci 1', 'Takipci 2', 'Takipci 3'});
disp(sonuc);
if m.doyma || m1.doyma
    warning('İvme sınırına ulaşıldı; lineer analiz bu koşulda geçerli değil.');
end

%% 7. Grafikler
etiket = {'Lider', 'Takipçi 1', 'Takipçi 2', 'Takipçi 3'};
renk = lines(3);
figure('Name', 'Çok araç dinleyen CACC', 'Color', 'w');
tiledlayout(2, 2, 'TileSpacing', 'compact');

nexttile; plot(o.tout, o.a_log, 'LineWidth', 1.2); grid on;
ylabel('İvme (m/s^2)'); title('Çok öncül: ivmeler'); legend(etiket, 'Location', 'best');

nexttile; plot(o1.tout, o1.a_log, 'LineWidth', 1.2); grid on;
ylabel('İvme (m/s^2)'); title('Tek öncül: ivmeler'); legend(etiket, 'Location', 'best');

nexttile; hold on; grid on;
for k = 1:p.N
    plot(o1.tout, o1.e_log(:, k), '--', 'Color', renk(k, :), 'LineWidth', 1);
    plot(o.tout, o.e_log(:, k), '-', 'Color', renk(k, :), 'LineWidth', 1.4);
end
ylabel('e = d - d_{des} (m)'); xlabel('Zaman (s)');
title('Mesafe hatası: tek öncül (kesikli) / çok öncül (düz)');

nexttile; semilogx(r.w, abs(r.T(2:end, :)), 'LineWidth', 1.2); grid on; hold on;
yline(1, 'k--');
xlabel('\omega (rad/s)'); ylabel('|V_i/V_0|'); title('Liderden takipçilere büyütme');
legend(etiket(2:end), 'Location', 'best');

%% Yerel fonksiyonlar
function m = metrikler(o, p)
a_max    = max(abs(o.a_log));
m.a_oran = a_max(2:end) ./ a_max(1:end-1);   % > 1 ise bozucu büyüyor
m.e_max  = max(abs(o.e_log));
m.d_min  = min(o.d_log);
m.doyma  = any(o.a_log(:, 2:end) >= 0.99*p.u_max | o.a_log(:, 2:end) <= 0.99*p.u_min, 'all');
end
