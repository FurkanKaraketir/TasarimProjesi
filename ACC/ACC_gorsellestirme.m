%% ACC KONVOY - 3B GÖRSELLEŞTİRME (Automated Driving Toolbox)
% ACC_yonetim.m'in simülasyon sonucunu drivingScenario üzerinde oynatır.
% Önce ACC_yonetim.m çalıştırılır; workspace'te sonuç yoksa bu dosya onu kendisi çağırır.
% Görünümler: takip kamerası (Takipçi 3 arkasından), kuşbakışı, mesafe-zaman grafiği.
% Konumlar: lider p = integral(v_lider), takipçiler p_i = p_(i-1) - d_i - L  (p: ön tampon)

if ~exist('out', 'var') || ~exist('d', 'var')
    ACC_yonetim;
end

%% 1. Oynatma ayarları
hiz_carpani = 2;      % 1 = gerçek zaman, 2 = iki kat hızlı ...
fps         = 25;     % görüntü kare hızı (Hz)
video_kaydi = false;  % true -> ACC_konvoy.mp4 kaydedilir

%% 2. Konumların yeniden kurulması
p      = zeros(numel(t), N + 1);
p(:,1) = cumtrapz(t, v(:,1));
for i = 1:N
    p(:,i+1) = p(:,i) - d(:,i) - L;
end

% Gösterim için sabit kare adımında örnekleme
t_g = (t(1):hiz_carpani/fps:t(end))';
p_g = interp1(t, p, t_g);
v_g = interp1(t, v, t_g);
a_g = interp1(t, a, t_g);
d_g = interp1(t, d, t_g);
e_g = interp1(t, e, t_g);

%% 3. Senaryo: düz, iki şeritli yol + 4 araç
senaryo = drivingScenario;
yol_bas = min(p(:)) - 60;
yol_son = max(p(:)) + 120;
road(senaryo, [yol_bas 0 0; yol_son 0 0], 'Lanes', lanespec(2, 'Width', 3.6));

renkler = [0.85 0.33 0.10      % lider: turuncu
           0.00 0.45 0.74      % takipçiler: mavi tonları
           0.30 0.60 0.85
           0.55 0.75 0.95];
arka_cikinti = 1.0;            % arka aks - arka tampon (m)
y_serit      = -1.8;           % sağ şerit ortası
araclar = cell(1, N + 1);
for i = 1:N + 1
    araclar{i} = vehicle(senaryo, 'ClassID', 1, 'Length', L, ...
        'RearOverhang', arka_cikinti, 'Name', etiket_arac{i}, ...
        'PlotColor', renkler(i,:), ...
        'Position', [p(1,i) - (L - arka_cikinti), y_serit, 0]);
end

%% 4. Pencere düzeni
fig = figure('Name', 'ACC konvoy - Automated Driving Toolbox', 'Color', 'w', ...
    'Position', [80 80 1400 800]);
pnl_takip = uipanel(fig, 'Position', [0.00 0.30 0.50 0.70], 'Title', 'Takip kamerası (Takipçi 3)');
pnl_graf  = uipanel(fig, 'Position', [0.50 0.30 0.50 0.70], 'BackgroundColor', 'w');
pnl_kus   = uipanel(fig, 'Position', [0.00 0.00 1.00 0.30], 'Title', 'Kuşbakışı');

ax_takip = axes(pnl_takip);
chasePlot(araclar{end}, 'Parent', ax_takip, 'ViewLocation', [-14 0], ...
    'ViewHeight', 4.5, 'ViewPitch', 6, 'Meshes', 'on');

ax_kus = axes(pnl_kus);
plot(senaryo, 'Parent', ax_kus, 'Meshes', 'on');
view(ax_kus, 2);
etiket_txt = gobjects(N, 1);
for i = 1:N
    etiket_txt(i) = text(ax_kus, 0, 3.2, '', 'HorizontalAlignment', 'center', ...
        'FontWeight', 'bold', 'FontSize', 10, 'Color', renkler(i+1,:), 'BackgroundColor', 'w');
end
bilgi_txt = uicontrol(pnl_takip, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.01 0.90 0.90 0.09], 'FontName', 'Consolas', 'FontSize', 10, ...
    'HorizontalAlignment', 'left', 'BackgroundColor', 'w');

tl = tiledlayout(pnl_graf, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_d = nexttile(tl); plot(ax_d, t, d, 'LineWidth', 1.2); grid(ax_d, 'on');
ylabel(ax_d, 'Mesafe (m)'); title(ax_d, 'Araçlar arası mesafe');
legend(ax_d, etiket_ara, 'Location', 'best');
ax_v = nexttile(tl); plot(ax_v, t, v, 'LineWidth', 1.2); grid(ax_v, 'on');
ylabel(ax_v, 'Hız (m/s)'); xlabel(ax_v, 'Zaman (s)'); title(ax_v, 'Hızlar');
legend(ax_v, etiket_arac, 'Location', 'best');
imlec = [xline(ax_d, 0, 'k-', 'LineWidth', 1, 'HandleVisibility', 'off'), ...
         xline(ax_v, 0, 'k-', 'LineWidth', 1, 'HandleVisibility', 'off')];

if video_kaydi
    vw = VideoWriter(fullfile(fileparts(mfilename('fullpath')), 'ACC_konvoy.mp4'), 'MPEG-4');
    vw.FrameRate = fps;
    open(vw);
end

%% 5. Oynatma döngüsü
for k = 1:numel(t_g)
    if ~isvalid(fig), break; end
    tic;
    for i = 1:N + 1
        araclar{i}.Position     = [p_g(k,i) - (L - arka_cikinti), y_serit, 0];
        araclar{i}.Velocity     = [v_g(k,i), 0, 0];
        araclar{i}.Acceleration = [a_g(k,i), 0, 0];
    end
    updatePlots(senaryo);

    % Kuşbakışı pencereyi konvoyla birlikte kaydır
    xlim(ax_kus, [p_g(k,end) - L - 10, p_g(k,1) + 10]);
    ylim(ax_kus, [-5 5]);
    for i = 1:N
        x_orta = (p_g(k,i) - L + p_g(k,i+1)) / 2;
        etiket_txt(i).Position = [x_orta, 3.2, 0];
        etiket_txt(i).String   = sprintf('%s = %.1f m', erase(etiket_ara{i}, {'_', '{', '}'}), d_g(k,i));
    end
    bilgi_txt.String = sprintf(['t = %5.2f s   lider v = %5.2f m/s   lider a = %+5.2f m/s^2\n' ...
        'e = [%+5.2f %+5.2f %+5.2f] m'], t_g(k), v_g(k,1), a_g(k,1), e_g(k,:));

    set(imlec, 'Value', t_g(k));
    drawnow limitrate;

    if video_kaydi
        writeVideo(vw, getframe(fig));
    else
        pause(max(0, 1/fps - toc));
    end
end

if video_kaydi
    close(vw);
    fprintf('Video kaydedildi: %s\n', vw.Filename);
end
