%% ÇOK ARAÇ DİNLEYEN CACC - 3B GÖRSELLEŞTİRME
% ACC, tek öncüllü CACC ve çok öncüllü CACC aynı lider ve aynı h ile karşılaştırılır.
% Şekil 1: kararlılık yüzeyi, sup|xi| (h, tauc düzleminde, uzun konvoy)
% Şekil 2: liderden takipçilere büyütme, |V_i/V_0| (frekans - araç)
% Şekil 3: ivme dalgası, a(t) (zaman - araç)
% Şekil 4: üç şeritli 3B oynatma (Automated Driving Toolbox; yoksa atlanır)
% Toolbox olmadan ilk üç şekil çalışır. Model: coklu_sim.m, analiz: coklu_frekans.m

if ~exist('oynat', 'var'), oynat = true; end             % false -> Şekil 4 oynatılmaz
if ~exist('png_klasor', 'var'), png_klasor = ''; end     % dolu ise şekiller PNG kaydedilir

%% 1. Parametreler
b.tau = 0.5;  b.Kp = 0.4;  b.ff_u = 1;  b.d0 = 5;  b.L = 4.5;
b.u_min = -5;  b.u_max = 3;  b.v0 = 15;  b.N = 3;
b.lider_lag = 1;   % 1 = lider de tau gecikmeli, 0 = liderde a = u (CACC_konvoy.slx gibi)

h    = 0.6;        % Zaman aralığı (s)
tauc = 0.100;      % V2V gecikmesi (s)
T_stop = 40;  dt = 0.01;
lider_profil = [ 5   8   1     % [başlangıç(s) bitiş(s) ivme(m/s^2)]
                12  14  -2 ];

ad = {'ACC', 'Tek öncül', 'Çok öncül'};
yapi = {[0.8], [0]; [0.8], [0.8]; [0.8 0 0], [0.5 0.2 0.1]};   % {kv, ka}
renk = [0.45 0.45 0.45; 0.00 0.45 0.74; 0.85 0.33 0.10];

hiz_carpani = 2;   fps = 25;

%% 2. Simülasyon ve frekans analizi
lider_ivme = [0 0];
for k = 1:size(lider_profil, 1)
    t1 = lider_profil(k, 1); t2 = lider_profil(k, 2); u1 = lider_profil(k, 3);
    lider_ivme = [lider_ivme; t1 0; t1 u1; t2 u1; t2 0]; %#ok<AGROW>
end
lider_ivme = [lider_ivme; T_stop 0];

n_yapi = numel(ad);
o = cell(1, n_yapi);  r = cell(1, n_yapi);  par = cell(1, n_yapi);
for y = 1:n_yapi
    par{y} = b;  par{y}.kv = yapi{y, 1};  par{y}.ka = yapi{y, 2};
    o{y} = coklu_sim(par{y}, h, tauc, lider_ivme, T_stop, dt);
    r{y} = coklu_frekans(par{y}, h, tauc);
end

%% 3. Şekil 1: kararlılık yüzeyi
h_iz = 0.2:0.05:3;   tc_iz = 0:0.0125:0.3;
w_iz = logspace(-3, 2, 400);
f1 = figure('Name', 'Kararlılık yüzeyi', 'Color', 'w', 'Position', [60 60 1500 520]);
tl = tiledlayout(f1, 1, n_yapi, 'TileSpacing', 'compact');
for y = 1:n_yapi
    S = zeros(numel(tc_iz), numel(h_iz));
    for i = 1:numel(tc_iz)
        for j = 1:numel(h_iz)
            S(i, j) = coklu_frekans(par{y}, h_iz(j), tc_iz(i), w_iz).sup_kuyruk;
        end
    end
    ax = nexttile(tl);
    surf(ax, h_iz, 1e3*tc_iz, min(S, 1.4), 'EdgeColor', 'none');
    hold(ax, 'on');
    surf(ax, h_iz([1 end]), 1e3*tc_iz([1 end]), ones(2), 'FaceColor', [0.2 0.2 0.2], ...
        'FaceAlpha', 0.12, 'EdgeColor', 'none');
    S1 = coklu_frekans(par{y}, h, tauc, w_iz).sup_kuyruk;
    plot3(ax, [h h], 1e3*tauc*[1 1], [min(S1, 1.4) 1.4], 'k', 'LineWidth', 1.2);   % seçilen nokta: iğne
    plot3(ax, h, 1e3*tauc, 1.4, 'kp', 'MarkerFaceColor', 'y', 'MarkerSize', 13);
    clim(ax, [1 1.3]);  colormap(ax, [0.80 0.90 0.80; hot(64)*0.15 + flipud(autumn(64))*0.85]);
    zlim(ax, [0.95 1.4]);  view(ax, -50, 28);  grid(ax, 'on');
    xlabel(ax, 'h (s)');  ylabel(ax, '\tau_c (ms)');  zlabel(ax, 'araç başına büyüme');
    title(ax, sprintf('%s (iğne: h=%.1f, %.0f ms, değer %.3f)', ad{y}, h, 1e3*tauc, S1));
end
title(tl, 'Uzun konvoyda string stability: yeşil düzlük kararlı, yükselen bölge kararsız');

%% 4. Şekil 2: frekans - araç - büyütme
f2 = figure('Name', 'Büyütme', 'Color', 'w', 'Position', [60 60 1500 520]);
tl = tiledlayout(f2, 1, n_yapi, 'TileSpacing', 'compact');
for y = 1:n_yapi
    ax = nexttile(tl);
    surf(ax, log10(r{y}.w), 0:b.N, abs(r{y}.T), 'EdgeColor', 'none');
    hold(ax, 'on');
    surf(ax, log10(r{y}.w([1 end])), [0 b.N], ones(2), 'FaceColor', 'k', 'FaceAlpha', 0.12, 'EdgeColor', 'none');
    for i = 0:b.N
        plot3(ax, log10(r{y}.w), i*ones(size(r{y}.w)), abs(r{y}.T(i + 1, :)), 'k', 'LineWidth', 1);
    end
    xlim(ax, [-1.5 1.2]);  zlim(ax, [0 2]);  clim(ax, [0 2]);  colormap(ax, turbo);
    view(ax, -38, 30);  grid(ax, 'on');
    yticks(ax, 0:b.N);  yticklabels(ax, {'Lider', 'T1', 'T2', 'T3'});
    xlabel(ax, 'log_{10} \omega (rad/s)');  zlabel(ax, '|V_i/V_0|');
    title(ax, sprintf('%s: T3''te tepe %.2f', ad{y}, r{y}.sup_T(end)));
end
title(tl, sprintf('Bozucunun konvoyda ilerleyişi (h=%.1f s, \\tau_c=%.0f ms); gri düzlem = 1', h, 1e3*tauc));

%% 5. Şekil 3: zaman - araç - ivme
f3 = figure('Name', 'İvme dalgası', 'Color', 'w', 'Position', [60 60 1500 520]);
tl = tiledlayout(f3, 1, n_yapi, 'TileSpacing', 'compact');
for y = 1:n_yapi
    ax = nexttile(tl);
    surf(ax, o{y}.tout(1:10:end), 0:b.N, o{y}.a_log(1:10:end, :)', 'EdgeColor', 'none');
    hold(ax, 'on');
    for i = 0:b.N
        plot3(ax, o{y}.tout, i*ones(size(o{y}.tout)), o{y}.a_log(:, i + 1), 'k', 'LineWidth', 1);
    end
    xlim(ax, [0 25]);  zlim(ax, [-2.5 1.5]);  clim(ax, [-2 2]);  colormap(ax, turbo);
    view(ax, -30, 35);  grid(ax, 'on');
    yticks(ax, 0:b.N);  yticklabels(ax, {'Lider', 'T1', 'T2', 'T3'});
    xlabel(ax, 'Zaman (s)');  zlabel(ax, 'a (m/s^2)');
    A = max(abs(o{y}.a_log));
    title(ax, sprintf('%s: a_{T3}/a_{lider} = %.2f', ad{y}, A(end)/A(1)));
end
title(tl, 'İvme dalgası araçtan araca');

if ~isempty(png_klasor)
    exportgraphics(f1, fullfile(png_klasor, 'kararlilik_yuzeyi.png'), 'Resolution', 110);
    exportgraphics(f2, fullfile(png_klasor, 'buyutme.png'), 'Resolution', 110);
    exportgraphics(f3, fullfile(png_klasor, 'ivme_dalgasi.png'), 'Resolution', 110);
end

%% 6. Şekil 4: üç şeritli 3B oynatma
if ~oynat || ~license('test', 'automated_driving_toolbox') || isempty(which('drivingScenario'))
    if oynat, warning('Automated Driving Toolbox yok; 3B oynatma atlandı.'); end
    return
end

% Konumlar: lider p = integral(v_lider), takipçiler p_i = p_(i-1) - d_i - L  (p: ön tampon)
t = o{1}.tout;
p_arac = zeros(numel(t), b.N + 1, n_yapi);
for y = 1:n_yapi
    p_arac(:, 1, y) = cumtrapz(t, o{y}.v_log(:, 1));
    for i = 1:b.N
        p_arac(:, i + 1, y) = p_arac(:, i, y) - o{y}.d_log(:, i) - b.L;
    end
end
t_g = (t(1):hiz_carpani/fps:t(end))';

senaryo = drivingScenario;
w_serit = 3.6;   y_serit = [w_serit 0 -w_serit];   arka = 1.0;
road(senaryo, [min(p_arac, [], 'all') - 60 0 0; max(p_arac, [], 'all') + 120 0 0], ...
    'Lanes', lanespec(3, 'Width', w_serit));
araclar = cell(n_yapi, b.N + 1);
for y = 1:n_yapi
    for i = 1:b.N + 1
        ton = renk(y, :) + (1 - renk(y, :))*0.18*(i - 1);
        araclar{y, i} = vehicle(senaryo, 'ClassID', 1, 'Length', b.L, 'RearOverhang', arka, ...
            'PlotColor', ton, 'Mesh', driving.scenario.carMesh, ...
            'Position', [p_arac(1, i, y) - (b.L - arka), y_serit(y), 0]);
    end
end

f4 = figure('Name', '3B konvoy: ACC / tek öncül / çok öncül', 'Color', 'w', 'Position', [60 60 1300 760]);
pnl_cam = uipanel(f4, 'Position', [0 0.28 1 0.72], 'Title', 'Takip kamerası (orta şerit, Takipçi 3 arkası)');
pnl_alt = uipanel(f4, 'Position', [0 0 1 0.28], 'BackgroundColor', 'w');
ax_cam = axes(pnl_cam);
chasePlot(araclar{2, end}, 'Parent', ax_cam, 'ViewLocation', [-22 0], 'ViewHeight', 9, ...
    'ViewPitch', 12, 'Meshes', 'on');
ax_e = axes(pnl_alt);  hold(ax_e, 'on');  grid(ax_e, 'on');
for y = 1:n_yapi
    plot(ax_e, t, o{y}.e_log(:, end), 'Color', renk(y, :), 'LineWidth', 1.3);
end
ylabel(ax_e, 'e_{34} (m)');  xlabel(ax_e, 'Zaman (s)');
legend(ax_e, ad, 'Location', 'eastoutside');
imlec = xline(ax_e, 0, 'k-', 'HandleVisibility', 'off');

for k = 1:numel(t_g)
    if ~isvalid(f4), break; end
    tic;
    ek = interp1(t, 1:numel(t), t_g(k), 'nearest');
    for y = 1:n_yapi
        for i = 1:b.N + 1
            araclar{y, i}.Position = [p_arac(ek, i, y) - (b.L - arka), y_serit(y), 0];
            araclar{y, i}.Velocity = [o{y}.v_log(ek, i), 0, 0];
        end
    end
    updatePlots(senaryo);
    imlec.Value = t_g(k);
    title(ax_e, sprintf('t = %.1f s   |   e_{34}:  ACC %+.2f m   tek öncül %+.2f m   çok öncül %+.2f m', t_g(k), ...
        o{1}.e_log(ek, end), o{2}.e_log(ek, end), o{3}.e_log(ek, end)));
    drawnow limitrate;
    if ~isempty(png_klasor) && abs(t_g(k) - 15) < hiz_carpani/fps/2
        imwrite(getframe(f4).cdata, fullfile(png_klasor, 'oynatma_t15.png'));
    end
    pause(max(0, 1/fps - toc));
end
