%% ÇOK ARAÇ DİNLEYEN CACC - UNREAL ENGINE VİDEOSU (Automated Driving Toolbox)
% ACC, tek öncüllü CACC ve çok öncüllü CACC aynı lider ve aynı h ile üç şeritte oynatılır,
% kamera görüntüsü MP4 olarak kaydedilir. Araç hareketi coklu_olay_sim.m sonucudur; Unreal
% yalnızca görüntüler. Tekerlek dönüşü ve gövde eğimi görsel efekttir, modelde yoktur.
% Şeritler (soldan sağa): çok öncül (kırmızı), tek öncül (mavi), ACC (gümüş).
% Üçüncü şerit için karşı şerit de kullanılır, çünkü sahnede tek yönde iki şerit var.
% Olaylar: senaryo değişkeni (bkz. coklu_olay_hazirla.m). Çıkan araç bir anda kaybolur, araya
% giren siyah araç bir anda belirir; modelde de geçiş anlıktır.

if ~exist('kare_klasor', 'var'), kare_klasor = ''; end     % dolu ise örnek kareler JPG kaydedilir
if ~exist('senaryo', 'var'), senaryo = 'normal'; end       % olay senaryosu

%% 1. Parametreler
b.tau = 0.5;  b.Kp = 0.4;  b.ff_u = 1;  b.d0 = 5;  b.L = 4.5;
b.u_min = -5;  b.u_max = 3;  b.v0 = 15;  b.N = 3;
b.lider_lag = 1;   % 1 = lider de tau gecikmeli, 0 = liderde a = u (CACC_konvoy.slx gibi)

h    = 0.6;        % Zaman aralığı (s)
tauc = 0.100;      % V2V gecikmesi (s)
t_olay = 18;       % Olayın zamanı (s)
T_video = 30;      % Video süresi (s)
dt   = 0.01;       % Simülasyon adımı (s)
lider_profil = [ 5   8   1     % [başlangıç(s) bitiş(s) ivme(m/s^2)]
                12  14  -2 ];

ad    = {'ACC', 'Tek öncül', 'Çok öncül'};
yapi  = {[0.8], [0]; [0.8], [0.8]; [0.8 0 0], [0.5 0.2 0.1]};   % {kv, ka}
renk  = {'silver', 'blue', 'red'};
y_serit = [5.4 1.8 -1.8];    % Unreal'da +y sağ taraf (m)
x_bas   = -320;              % Liderin yol üzerindeki başlangıcı (m)

fps = 30;  boyut = [1080 1920];  n_isinma = 20;

% Kamera anahtar kareleri: [t(s) dx(m) y(m) z(m) pitch(rad) yaw(rad)], dx konvoy ortasına göre
kamera = [ 0   -62   1.8  13.0  -0.20   0
           4.5 -62   1.8  13.0  -0.20   0
           7     0  -72   27.0  -0.40   pi/2
          11     0  -72   27.0  -0.40   pi/2
          13   -66   1.8  12.0  -0.19   0
          20   -66   1.8  12.0  -0.19   0
          23     0  -15   72.0  -1.35   pi/2
          30     0  -15   72.0  -1.35   pi/2 ];

%% 2. Simülasyon
lider_ivme = [0 0];
for k = 1:size(lider_profil, 1)
    t1 = lider_profil(k, 1); t2 = lider_profil(k, 2); u1 = lider_profil(k, 3);
    lider_ivme = [lider_ivme; t1 0; t1 u1; t2 u1; t2 0]; %#ok<AGROW>
end
lider_ivme = [lider_ivme; T_video 0];

n_yapi = numel(ad);
t_v = (0:1/fps:T_video)';
for y = n_yapi:-1:1
    p = b;  p.kv = yapi{y, 1};  p.ka = yapi{y, 2};
    ol = coklu_olay_hazirla(senaryo, t_olay);  ol.tauc = tauc;
    o = coklu_olay_sim(p, h, ol, lider_ivme, T_video, dt);
    k_oran = o.tout < t_olay | ~(isfield(ol, 'cikis') || isfield(ol, 'giris'));   % boşluk kapatma sayılmaz
    A = max(abs(o.a_log(k_oran, 1:b.N + 1)));
    D(y).x = x_bas + interp1(o.tout, o.x_log, t_v);   % ön tampon konumu
    D(y).v = interp1(o.tout, o.v_log, t_v);
    D(y).a = interp1(o.tout, o.a_log, t_v);
    D(y).d = interp1(o.tout, o.d_log, t_v);
    D(y).e = interp1(o.tout, o.e_log, t_v);
    D(y).s = cumtrapz(t_v, fillmissing(D(y).v, 'constant', 0));   % tekerlek dönüşü için yol
    D(y).tauc = interp1(o.tout, o.tauc_log, t_v, 'previous');
    D(y).oran = A(end)/A(1);
end

%% 3. Alt grafik (bir kez çizilir, her karede imleç eklenir)
g_y = 190;
fg = figure('Visible', 'off', 'Color', [0.08 0.09 0.11], 'Units', 'pixels', 'Position', [50 50 boyut(2) g_y]);
ax = axes(fg, 'Position', [0.05 0.24 0.93 0.66], 'Color', [0.08 0.09 0.11], 'XColor', [0.8 0.8 0.8], ...
    'YColor', [0.8 0.8 0.8], 'FontSize', 11);
hold(ax, 'on');  grid(ax, 'on');  ax.GridColor = [0.6 0.6 0.6];
c_g = [0.72 0.75 0.78; 0.25 0.55 1.00; 1.00 0.30 0.25];
for y = 1:n_yapi
    plot(ax, t_v, D(y).e(:, b.N), 'Color', c_g(y, :), 'LineWidth', 2);
end
if ~strcmp(senaryo, 'normal'), xline(ax, t_olay, '--', 'Color', [1 0.86 0.47], 'LineWidth', 1.2); end
xlim(ax, [0 T_video]);
ylabel(ax, 'e_{34} (m)');  xlabel(ax, 'Zaman (s)');
grafik = imresize(getframe(fg).cdata, [g_y boyut(2)]);
imlec_x = @(t) round(boyut(2)*(ax.Position(1) + ax.Position(3)*t/T_video));
imlec_y = round(g_y*[1 - ax.Position(2) - ax.Position(4), 1 - ax.Position(2)]);
x_imlec = arrayfun(imlec_x, t_v);
close(fg);

%% 4. Unreal sahnesi
w = sim3d.World('Scene', 'StraightRoad');
n_arac = size(D(1).x, 2);
A_v = cell(n_yapi, n_arac);
for y = 1:n_yapi
    for i = 1:n_arac
        rk = renk{y};
        if i > b.N + 1, rk = 'black'; end       % araya giren araç
        A_v{y, i} = sim3d.auto.PassengerVehicle(sprintf('A%d_%d', y, i), 'Sedan', ...
            'Translation', [-5000, y_serit(y), 0], 'Rotation', [0 0 0], 'Color', rk);
        w.add(A_v{y, i});
    end
end
cam = sim3d.sensors.IdealCamera('ImageSize', boyut, 'Translation', [x_bas - 80, 1.8, 9], ...
    'Rotation', [0 -0.16 0], 'HorizontalFieldOfView', 62);
w.add(cam);

klasor = fileparts(mfilename('fullpath'));
dosya = 'CACC_coklu_unreal.mp4';
if ~strcmp(senaryo, 'normal'), dosya = ['CACC_coklu_unreal_' senaryo '.mp4']; end
vw = VideoWriter(fullfile(klasor, dosya), 'MPEG-4');
vw.FrameRate = fps;  vw.Quality = 85;
open(vw);

U = struct('n', 0, 'n_isinma', n_isinma, 'fps', fps, 'D', D, 'A_v', {A_v}, 'cam', cam, 'kamera', kamera, ...
    'L', b.L, 'y_serit', y_serit, 'ad', {ad}, 'c_g', c_g, 'vw', vw, 'grafik', grafik, 'x_imlec', x_imlec, ...
    'imlec_y', imlec_y, 'h', h, 'tauc', tauc, 'lider_profil', lider_profil, 'kare_klasor', kare_klasor, ...
    'n_kare', numel(t_v), 'senaryo', senaryo, 'N', b.N);
w.UserData = U;
w.OutputImpl = @kare_guncelle;

%% 5. Çalıştır ve kaydet
w.run(1/fps, T_video + (n_isinma + 2)/fps);
close(vw);
delete(w);
fprintf('Video kaydedildi: %s\n', vw.Filename);

%% Yerel fonksiyonlar
function kare_guncelle(w)
    U = w.UserData;
    U.n = U.n + 1;
    k = min(max(U.n - U.n_isinma, 1), U.n_kare);
    t = (k - 1)/U.fps;
    
    % Araçlar
    for y = 1:numel(U.D)
        for i = 1:size(U.A_v, 2)
            arac = U.A_v{y, i};
            T = arac.Translation;  R = arac.Rotation;
            if isnan(U.D(y).x(k, i))
                T(1, 1) = -5000;                         % şeritte olmayan araç kadraj dışında
            else
                T(1, :) = [U.D(y).x(k, i) - U.L/2, U.y_serit(y), 0];
                R(1, 2) = 0.012*U.D(y).a(k, i);          % gövde eğimi (görsel)
                R(2:5, 2) = -U.D(y).s(k, i)/0.33;        % tekerlek dönüşü (görsel)
            end
            arac.Translation = T;  arac.Rotation = R;
        end
    end
    
    % Kamera
    xc = (max(U.D(2).x(k, :)) + min(U.D(2).x(k, :)))/2;
    q = kamera_pozu(U.kamera, t);
    U.cam.Translation = [xc + q(1), q(2), q(3)];
    U.cam.Rotation = [0 q(4) q(5)];
    
    % Görüntü
    if U.n > U.n_isinma && U.n - U.n_isinma <= U.n_kare
        img = U.cam.read();
        img = hud_ekle(img, U, k, t);
        writeVideo(U.vw, img);
        if ~isempty(U.kare_klasor) && any(abs(t - [2 9 14 17 18 19 22 26]) < 1e-6)
            imwrite(img, fullfile(U.kare_klasor, sprintf('kare_t%04.1f.jpg', t)), 'Quality', 92);
        end
    end
    w.UserData = U;
end

function q = kamera_pozu(K, t)
    i = find(K(:, 1) <= t, 1, 'last');
    if i == size(K, 1), q = K(i, 2:end); return; end
    s = (t - K(i, 1))/(K(i + 1, 1) - K(i, 1));
    s = s*s*(3 - 2*s);                                  % yumuşak geçiş
    q = K(i, 2:end) + s*(K(i + 1, 2:end) - K(i, 2:end));
end

function img = hud_ekle(img, U, k, t)
    [n_y, n_x, ~] = size(img);
    
    % Alt grafik
    g = U.grafik;
    g(U.imlec_y(1):U.imlec_y(2), U.x_imlec(k):U.x_imlec(k) + 2, :) = 255;
    img(n_y - size(g, 1) + 1:n_y, 1:n_x, :) = g;
    
    % Üst panel
    pan_x = 860;  pan_y = 214;
    img(11:10 + pan_y, 11:10 + pan_x, :) = uint8(0.25*double(img(11:10 + pan_y, 11:10 + pan_x, :)) + 0.75*20);
    yazi = @(im, x, y, s, r, b) insertText(im, [x y], s, 'Font', 'Calibri', 'FontSize', b, 'TextColor', r, ...
        'BoxOpacity', 0, 'AnchorPoint', 'LeftTop');
    u_l = 0;
    for r = 1:size(U.lider_profil, 1)
        if t >= U.lider_profil(r, 1) && t < U.lider_profil(r, 2), u_l = U.lider_profil(r, 3); end
    end
    if u_l > 0, faz = sprintf('Lider hızlanıyor (%+g m/s²)', u_l);
    elseif u_l < 0, faz = sprintf('Lider FRENLİYOR (%+g m/s²)', u_l);
    else, faz = 'Lider sabit komutta'; end
    img = yazi(img, 24, 18, sprintf('t = %5.1f s    h = %.1f s    gecikme = %.0f ms', t, U.h, 1e3*U.D(end).tauc(k)), 'white', 26);
    if ~strcmp(U.senaryo, 'normal'), faz = [faz '  |  ' strrep(U.senaryo, '_', ' ')]; end
    img = yazi(img, 24, 54, faz, [255 220 120], 24);
    img = yazi(img, 330, 90, 'e34 (m)      d34 (m)     aT3/aL (son)', [190 190 190], 18);
    img = yazi(img, 668, 90, 'L  T1 T2 T3 (fren)', [190 190 190], 18);
    for y = 1:numel(U.D)
        sat = 90 + 30*y;
        r = round(255*U.c_g(y, :));
        img = yazi(img, 24, sat - 4, U.ad{y}, r, 26);
        img = yazi(img, 330, sat, sprintf('%+6.2f', U.D(y).e(k, U.N)), 'white', 22);
        img = yazi(img, 450, sat, sprintf('%6.2f', U.D(y).d(k, U.N)), 'white', 22);
        img = yazi(img, 590, sat, sprintf('%.2f', U.D(y).oran), 'white', 22);
        for i = 1:U.N + 1                               % frenleyen araç kırmızı (a < -0.3 m/s^2)
            kx = 670 + 25*(i - 1);
            if U.D(y).a(k, i) < -0.3, c = [255 40 40]; else, c = [70 75 80]; end
            for ch = 1:3
                img(sat + 5:sat + 22, kx:kx + 17, ch) = c(ch);
            end
        end
    end
end
