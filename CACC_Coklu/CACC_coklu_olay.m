%% ÇOK ARAÇ DİNLEYEN CACC - OLAY SENARYOLARI
% ACC, tek öncül ve çok öncül aynı olay altında karşılaştırılır.
% Kullanım: Bölüm 1'de senaryoyu seç, dosyayı çalıştır (F5).
% Senaryolar: normal, rastgele_gecikme, paket_kaybi, haberlesme_kesintisi, kesinti_son_deger,
%             arac_cikisi, araya_girme, radar_gurultu, aktuator_ariza, ani_fren,
%             lider_ani_fren, hepsi
% Tanımlar: coklu_olay_hazirla.m. Model: coklu_olay_sim.m.
% Not: Olaylar konvoyu lineer çalışma noktasından çıkarır; sup|Gamma| burada geçerli ölçüt değildir.

if ~exist('senaryo', 'var'), senaryo = 'arac_cikisi'; end

%% 1. Parametreler
t_olay  = 18;      % Olayın zamanı (s)
n_tohum = 10;      % Rastgele senaryolarda tekrar sayısı
T_stop  = 40;      % Simülasyon süresi (s)
dt      = 0.01;    % Sabit adım (s)
h       = 0.6;     % Zaman aralığı (s)

b.tau = 0.5;  b.Kp = 0.4;  b.ff_u = 1;  b.d0 = 5;  b.L = 4.5;
b.u_min = -5;  b.u_max = 3;  b.v0 = 15;  b.N = 3;
b.lider_lag = 1;   % 1 = lider de tau gecikmeli, 0 = liderde a = u (CACC_konvoy.slx gibi)

lider_profil = [ 5   8   1     % [başlangıç(s) bitiş(s) ivme(m/s^2)]
                12  14  -2 ];

ad   = {'ACC', 'Tek öncül', 'Çok öncül'};
yapi = {[0.8], [0]; [0.8], [0.8]; [0.8 0 0], [0.5 0.2 0.1]};   % {kv, ka}

%% 2. Türetilen değerler
lider_ivme = [0 0];
for k = 1:size(lider_profil, 1)
    t1 = lider_profil(k, 1); t2 = lider_profil(k, 2); u1 = lider_profil(k, 3);
    lider_ivme = [lider_ivme; t1 0; t1 u1; t2 u1; t2 0]; %#ok<AGROW>
end
lider_ivme = [lider_ivme; T_stop 0];

ol0 = coklu_olay_hazirla(senaryo, t_olay);
rastgele = isfield(ol0, 'tauc_rastgele') || isfield(ol0, 'kayip') || isfield(ol0, 'gurultu');
if ~rastgele, n_tohum = 1; end

%% 3. Simülasyon
n_yapi = numel(ad);
o = cell(1, n_yapi);
R = zeros(n_tohum, 5, n_yapi);      % [RMSE, e_max, d_min, a_max, doyma süresi]
for y = 1:n_yapi
    p = b;  p.kv = yapi{y, 1};  p.ka = yapi{y, 2};
    for s = n_tohum:-1:1
        ol = ol0;  ol.tohum = s;
        c = coklu_olay_sim(p, h, ol, lider_ivme, T_stop, dt);
        R(s, :, y) = metrikler(c, b, dt);
    end
    o{y} = c;                       % grafik için ilk tohum
end

%% 4. Sonuçlar
fprintf('=== Senaryo: %s | h=%.2f s, %d tekrar ===\n', senaryo, h, n_tohum);
ort = squeeze(mean(R, 1))';   std_ = squeeze(std(R, 0, 1))';
sonuc = table(ort(:, 1), std_(:, 1), ort(:, 2), ort(:, 3), squeeze(min(R(:, 3, :), [], 1)), ...
    ort(:, 4), ort(:, 5), 'VariableNames', {'RMSE_e', 'RMSE_std', 'max_e', 'd_min_ort', 'd_min_enkotu', 'a_max', 'doyma_s'}, ...
    'RowNames', ad);
disp(sonuc);
if any(R(:, 3, :) <= 0, 'all'), warning('Çarpışma: en az bir mesafe sıfırın altına indi.'); end
if any(cellfun(@(c) any(c.v_log(:) < 0), o)), warning('Negatif hız oluştu; modelde v >= 0 sınırı yok.'); end

%% 5. Grafikler
c_y = [0.45 0.45 0.45; 0.00 0.45 0.74; 0.85 0.33 0.10];
figure('Name', ['Olay: ' senaryo], 'Color', 'w', 'Position', [60 60 1300 720]);
tl = tiledlayout(2, 3, 'TileSpacing', 'compact');
for y = 1:n_yapi
    ax = nexttile(tl);
    plot(ax, o{y}.tout, o{y}.e_log, 'LineWidth', 1.2); grid(ax, 'on');
    title(ax, [ad{y} ': mesafe hatası']);  ylabel(ax, 'e (m)');  xlabel(ax, 'Zaman (s)');
    xline(ax, t_olay, 'k--');
end
ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
for y = 1:n_yapi
    plot(ax, o{y}.tout, min(o{y}.d_log, [], 2), 'Color', c_y(y, :), 'LineWidth', 1.3);
end
yline(ax, 0, 'r-');  xline(ax, t_olay, 'k--');
title(ax, 'En küçük mesafe');  ylabel(ax, 'd (m)');  xlabel(ax, 'Zaman (s)');  legend(ax, ad, 'Location', 'best');
ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
for y = 1:n_yapi
    plot(ax, o{y}.tout, max(abs(o{y}.a_log(:, 2:end)), [], 2), 'Color', c_y(y, :), 'LineWidth', 1.3);
end
xline(ax, t_olay, 'k--');
title(ax, 'Takipçilerde en büyük |a|');  ylabel(ax, '|a| (m/s^2)');  xlabel(ax, 'Zaman (s)');
ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
yyaxis(ax, 'left');   stairs(ax, o{3}.tout, 1e3*o{3}.tauc_log, 'LineWidth', 1);  ylabel(ax, '\tau_c (ms)');
yyaxis(ax, 'right');  plot(ax, o{3}.tout, o{3}.v2v_log, 'LineWidth', 1);  ylabel(ax, 'Geçerli V2V oranı');  ylim(ax, [-0.05 1.05]);
title(ax, 'Haberleşme (çok öncül)');  xlabel(ax, 'Zaman (s)');
title(tl, ['Senaryo: ' strrep(senaryo, '_', ' ')]);

%% Yerel fonksiyonlar
function m = metrikler(c, b, dt)
e = c.e_log;  a = c.a_log(:, 2:end);
sat = any(a >= 0.99*b.u_max | a <= 0.99*b.u_min, 2);
m = [sum(sqrt(mean(e.^2, 'omitnan')), 'omitnan'), max(abs(e), [], 'all', 'omitnan'), ...
     min(c.d_log, [], 'all', 'omitnan'), max(abs(a), [], 'all', 'omitnan'), dt*sum(sat)];
end
