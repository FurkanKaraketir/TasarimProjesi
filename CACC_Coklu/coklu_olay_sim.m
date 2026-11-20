function o = coklu_olay_sim(p, h, ol, lider_ivme, T_stop, dt)
% Olay eklenebilen çok araç dinleyen CACC modeli (RK4, sabit adım).
% Olay verilmezse coklu_sim.m ile aynı sonucu verir. Parametreler p için coklu_sim.m'e bak.
% Araç numaraları: 0 = lider, 1..N = takipçiler. Bütün ol alanları isteğe bağlıdır.
%
% Haberleşme
%   ol.tauc          sabit V2V gecikmesi (s); 0 veya en az dt
%   ol.tauc_rastgele [alt ust] (s): her ol.tutma saniyede yeniden çekilen rastgele gecikme
%   ol.arac_bazli    true: her alıcının gecikmesi ayrı, false: hepsi aynı
%   ol.f_mesaj       mesaj hızı (Hz); 0 = sürekli sinyal, kayıp ve kesinti için 0'dan büyük olmalı
%   ol.kayip         paket kayıp olasılığı (0-1), alıcı ve gönderici çifti başına bağımsız
%   ol.kesinti       [t1 t2; ...] (s): bu aralıkta gönderilen hiçbir paket ulaşmaz
%   ol.zaman_asimi   (s): bundan eski mesaj kullanılmaz, araç o terim için ACC'ye düşer
%                    (Inf = son gelen mesaj sonsuza kadar tutulur)
% Konvoy
%   ol.cikis         [arac t; ...]: araç t anında bir anda çıkar, arkası yeni öncülünü izler
%   ol.giris         struct(once, t, dv, oran): V2V'siz bir araç, 'once' takipçisinin önüne
%                    girer. Hızı öncülün hızı + dv; oran = öndeki boşluğun aralığa oranı
% Araç ve algılayıcı
%   ol.gurultu       [d_std v_std] (m, m/s): radar mesafe ve hız farkı gürültüsü
%   ol.gurultu_tutma gürültü örnekleme aralığı (s)
%   ol.ariza         [arac t tau_kat; ...]: t anından sonra aracın tau'su tau_kat katına çıkar
%   ol.ani_fren      [arac t1 t2 u; ...]: bu aralıkta aracın komutu u olur (kontrolör devre dışı)
%   ol.tohum         rastgele sayı tohumu
%
% Çıktılar coklu_sim ile aynı, ek olarak x_log, tauc_log, v2v_log (geçerli V2V mesaj oranı).
% Devre dışı araçların x, v, a, d, e değerleri NaN'dır. Araya giren araç son sütundadır.

ol = varsayilan(ol);
if ol.tauc > 0 && ol.tauc < dt, error('tauc 0 veya en az dt olmalı.'); end
if ~isempty(ol.tauc_rastgele) && min(ol.tauc_rastgele) < dt, error('Rastgele gecikmenin alt sınırı en az dt olmalı.'); end
if ol.f_mesaj == 0 && (ol.kayip > 0 || ~isempty(ol.kesinti))
    error('Paket kaybı ve kesinti için ol.f_mesaj > 0 olmalı.');
end
if ~isempty(ol.giris) && ol.giris.oran <= 0, error('ol.giris.oran 0 ile 1 arasında olmalı.'); end

lider_lag = isfield(p, 'lider_lag') && p.lider_lag;
N = p.N;  M = numel(p.kv);
giren_var = ~isempty(ol.giris);
nv = N + 1 + giren_var;                     % toplam araç sayısı
t = (0:dt:T_stop)';  n = numel(t);
d_ss = p.d0 + h*p.v0;

x = zeros(3*nv, 1);                         % [p; v; a], 1 = lider
x(1:N + 1) = -(0:N)' * (d_ss + p.L);
x(nv + 1:nv + N + 1) = p.v0;
i_v = nv;  i_a = 2*nv;

akis = RandStream('twister', 'Seed', ol.tohum);

% Rastgele gecikme tablosu (alıcı başına)
tab_tauc = [];
if ~isempty(ol.tauc_rastgele)
    n_t = ceil(T_stop/ol.tutma) + 2;
    tab_tauc = ol.tauc_rastgele(1) + diff(ol.tauc_rastgele)*rand(akis, n_t, nv);
    if ~ol.arac_bazli, tab_tauc = repmat(tab_tauc(:, 1), 1, nv); end
end

% Paket kayıp tablosu (paket, alıcı, gönderici)
paketli = ol.f_mesaj > 0;
if paketli
    n_p = ceil(T_stop*ol.f_mesaj) + 2;
    tab_kayip = rand(akis, n_p, nv, nv) < ol.kayip;
    for r = 1:size(ol.kesinti, 1)
        t_p = (0:n_p - 1)/ol.f_mesaj;
        tab_kayip(t_p >= ol.kesinti(r, 1) & t_p < ol.kesinti(r, 2), :, :) = true;
    end
end

% Radar gürültü tablosu (örnek, araç)
tab_gd = zeros(1, nv);  tab_gv = zeros(1, nv);
if any(ol.gurultu ~= 0)
    n_g = ceil(T_stop/ol.gurultu_tutma) + 2;
    tab_gd = ol.gurultu(1)*randn(akis, n_g, nv);
    tab_gv = ol.gurultu(2)*randn(akis, n_g, nv);
end

hV = zeros(n, nv);  hU = zeros(n, nv);
o.tout = t;
o.x_log = NaN(n, nv);  o.v_log = NaN(n, nv);  o.a_log = NaN(n, nv);
o.d_log = NaN(n, nv - 1);  o.e_log = NaN(n, nv - 1);
o.tauc_log = zeros(n, 1);  o.v2v_log = ones(n, 1);
ta = lider_ivme(:, 1);  aa = lider_ivme(:, 2);

zincir = 1:N + 1;                           % o anda şeritte sırayla giden araçlar
v2v_var = true(1, nv);
if giren_var, v2v_var(nv) = false; end
cikis_k = [];  if ~isempty(ol.cikis), cikis_k = round(ol.cikis(:, 2)/dt) + 1; end
giris_k = [];  if giren_var, giris_k = round(ol.giris.t/dt) + 1; end

for k = 1:n
    % Konvoy değişiklikleri adım başında uygulanır
    for r = find(cikis_k == k)'
        zincir(zincir == ol.cikis(r, 1) + 1) = [];
    end
    if k == giris_k
        arka = ol.giris.once + 1;
        yer = find(zincir == arka);
        on = zincir(yer - 1);
        bosluk = x(on) - p.L - x(arka) - p.L;       % iki aracın arasında kalan boşluk
        x(nv) = x(on) - p.L - ol.giris.oran*bosluk;
        x(i_v + nv) = x(i_v + on) + ol.giris.dv;
        x(i_a + nv) = 0;
        zincir = [zincir(1:yer - 1), nv, zincir(yer:end)];
    end

    [~, u, gecerli, tauc_k] = turev(t(k), x, k);
    v = x(i_v + 1:i_v + nv);  a = x(i_a + 1:i_a + nv);
    if ~lider_lag, a(1) = u(1); end
    hV(k, :) = v';  hU(k, :) = u';
    if p.ff_u == 0, hU(k, :) = a'; end

    akt = zincir;
    o.x_log(k, akt) = x(akt)';  o.v_log(k, akt) = v(akt)';  o.a_log(k, akt) = a(akt)';
    for s = 2:numel(akt)
        d_s = x(akt(s - 1)) - x(akt(s)) - p.L;
        o.d_log(k, akt(s) - 1) = d_s;
        o.e_log(k, akt(s) - 1) = d_s - p.d0 - h*v(akt(s));
    end
    o.tauc_log(k) = tauc_k;  o.v2v_log(k) = gecerli;
    if k == n, break; end

    k1 = turev(t(k), x, k);
    k2 = turev(t(k) + dt/2, x + dt/2*k1, k);
    k3 = turev(t(k) + dt/2, x + dt/2*k2, k);
    k4 = turev(t(k) + dt, x + dt*k3, k);
    x = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end

    function [dx, u, gecerli, tauc_k] = turev(ts, xs, ks)
        pos = xs(1:nv);  vs = xs(i_v + 1:i_v + nv);  as = xs(i_a + 1:i_a + nv);
        u = zeros(nv, 1);
        u(1) = fren_uygula(lider(ts), 1, ts);
        n_top = 0;  n_iyi = 0;  tauc_k = tauc_oku(ts, 2);
        for s = 2:numel(zincir)
            arac = zincir(s);
            ig = 1;
            if size(tab_gd, 1) > 1, ig = floor(ts/ol.gurultu_tutma) + 1; end
            d_i = pos(zincir(s - 1)) - pos(arac) - p.L + tab_gd(ig, arac);
            ui = p.Kp*(d_i - p.d0 - h*vs(arac)) + p.kv(1)*(vs(zincir(s - 1)) - vs(arac) + tab_gv(ig, arac));
            if v2v_var(arac)
                for j = 1:min(s - 1, M)
                    gon = zincir(s - j);
                    if ~v2v_var(gon) || (p.ka(j) == 0 && (j == 1 || p.kv(j) == 0)), continue; end
                    [vg, ug, iyi] = mesaj(gon, arac, ts, ks, vs, u, as);
                    n_top = n_top + 1;  n_iyi = n_iyi + iyi;
                    if ~iyi, continue; end
                    if j >= 2, ui = ui + p.kv(j)*(vg - vs(arac)); end
                    ui = ui + p.ka(j)*ug;
                end
            end
            u(arac) = fren_uygula(min(max(ui, p.u_min), p.u_max), arac, ts);
        end
        gecerli = 1;  if n_top > 0, gecerli = n_iyi/n_top; end

        da = (u - as)./tau_oku(ts);
        dv = as;
        if ~lider_lag, da(1) = 0;  dv(1) = u(1); end
        dx = [vs; dv; da];
    end

    function ta_v = tau_oku(ts)
        ta_v = p.tau*ones(nv, 1);
        for r2 = 1:size(ol.ariza, 1)
            if ts >= ol.ariza(r2, 2), ta_v(ol.ariza(r2, 1) + 1) = p.tau*ol.ariza(r2, 3); end
        end
    end

    function ui = fren_uygula(ui, arac, ts)
        for r2 = 1:size(ol.ani_fren, 1)
            if ol.ani_fren(r2, 1) + 1 == arac && ts >= ol.ani_fren(r2, 2) && ts < ol.ani_fren(r2, 3)
                ui = ol.ani_fren(r2, 4);
            end
        end
    end

    function tc = tauc_oku(ts, alici)
        tc = ol.tauc;
        if ~isempty(tab_tauc), tc = tab_tauc(floor(ts/ol.tutma) + 1, alici); end
    end

    function [vg, ug, iyi] = mesaj(no, alici, ts, ks, vs, u, as)
        iyi = true;
        tc = tauc_oku(ts, alici);
        if paketli
            tg = ts - tc;
            pk = floor(tg*ol.f_mesaj + 1e-9);
            if pk < 0
                vg = p.v0;  ug = 0;  return
            end
            while pk >= 0 && tab_kayip(pk + 1, alici, no), pk = pk - 1; end
            if pk < 0
                vg = p.v0;  ug = 0;  return
            end
            if ts - pk/ol.f_mesaj > ol.zaman_asimi
                vg = 0;  ug = 0;  iyi = false;  return
            end
            ik = min(round(pk/ol.f_mesaj/dt) + 1, ks);
            vg = hV(ik, no);  ug = hU(ik, no);
            return
        end
        if tc == 0
            vg = vs(no);
            if p.ff_u == 1 || no == 1, ug = u(no); else, ug = as(no); end
            return
        end
        tg = ts - tc;
        if tg <= 0
            vg = p.v0;  ug = 0;  return       % t < 0: seyir hâli
        end
        i0 = min(floor(tg/dt + 1e-9) + 1, ks - 1);
        i0 = max(i0, 1);
        if ks == 1, vg = hV(1, no); ug = hU(1, no); return; end
        oran = (tg - t(i0))/dt;
        vg = hV(i0, no) + oran*(hV(i0 + 1, no) - hV(i0, no));
        ug = hU(i0, no) + oran*(hU(i0 + 1, no) - hU(i0, no));
    end

    function a = lider(ts)
        i = find(ta <= ts, 1, 'last');
        if isempty(i), a = 0; else, a = aa(i); end
    end
end

function ol = varsayilan(ol)
v = struct('tauc', 0.1, 'tauc_rastgele', [], 'tutma', 0.5, 'arac_bazli', true, 'f_mesaj', 0, ...
    'kayip', 0, 'kesinti', [], 'zaman_asimi', Inf, 'cikis', [], 'giris', [], 'gurultu', [0 0], ...
    'gurultu_tutma', 0.05, 'ariza', [], 'ani_fren', [], 'tohum', 1);
for f = fieldnames(v)'
    if ~isfield(ol, f{1}), ol.(f{1}) = v.(f{1}); end
end
end
