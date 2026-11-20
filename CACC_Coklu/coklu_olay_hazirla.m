function ol = coklu_olay_hazirla(senaryo, t_olay)
% Hazır olay senaryoları. Çıktı coklu_olay_sim.m'e verilir; alanların anlamı orada yazılıdır.
% Değerleri değiştirmek için bu dosyayı düzenle veya dönen yapının alanlarını sonradan ata.
% t_olay: olayın başlangıç zamanı (s)

if nargin < 2, t_olay = 18; end
ol = struct('tauc', 0.100);

switch senaryo
    case 'normal'
    case 'rastgele_gecikme'      % 50-250 ms arası, alıcı başına, 0.5 s'de bir değişen gecikme
        ol.tauc_rastgele = [0.050 0.250];
    case 'paket_kaybi'           % 10 Hz mesaj, %30 kayıp, 0.5 s'den eski mesajda ACC'ye düşüş
        ol.f_mesaj = 10;  ol.kayip = 0.3;  ol.zaman_asimi = 0.5;
    case 'haberlesme_kesintisi'  % liderin freni sırasında 5 s V2V kesintisi, ACC'ye düşüş
        ol.f_mesaj = 10;  ol.kesinti = [13 18];  ol.zaman_asimi = 0.5;
    case 'kesinti_son_deger'     % aynı kesinti, ama araç son gelen mesajı kullanmaya devam eder
        ol.f_mesaj = 10;  ol.kesinti = [13 18];  ol.zaman_asimi = Inf;
    case 'arac_cikisi'           % T2 bir anda çıkar, T3 boşluğu kapatır
        ol.cikis = [2 t_olay];
    case 'araya_girme'           % V2V'siz bir araç T2'nin önüne girer
        ol.giris = struct('once', 2, 't', t_olay, 'dv', 0, 'oran', 0.5);
    case 'radar_gurultu'         % radar: 0.3 m ve 0.1 m/s standart sapma
        ol.gurultu = [0.3 0.1];
    case 'aktuator_ariza'        % liderin manevralarından önce T1'in aktüatörü 4 kat yavaşlar
        ol.ariza = [1 3 4];
    case 'ani_fren'              % T2 2 s boyunca -4 m/s^2 frenler (kontrolör devre dışı)
        ol.ani_fren = [2 t_olay t_olay + 2 -4];
    case 'lider_ani_fren'        % lider 2 s boyunca -4 m/s^2 frenler
        ol.ani_fren = [0 t_olay t_olay + 2 -4];
    case 'hepsi'                 % rastgele gecikme + kayıp + gürültü + T2 çıkışı
        ol.tauc_rastgele = [0.050 0.250];
        ol.f_mesaj = 10;  ol.kayip = 0.1;  ol.zaman_asimi = 0.5;
        ol.gurultu = [0.1 0.05];
        ol.cikis = [2 t_olay];
    otherwise
        error('Bilinmeyen senaryo: %s', senaryo);
end
end
