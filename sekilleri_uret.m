function sekilleri_uret()
%SEKILLERI_URET  Notlarda ve raporda kullanılan şekilleri yeniden üretir.
% Yönetim betiklerini varsayılan parametrelerle çalıştırır ve açılan her
% şekli teknik/sekiller/ altına PNG olarak kaydeder.
% Kullanım (teknik/kodlar içinde):  sekilleri_uret
% Gerekenler: Simulink (ACC, CACC). Çok öncül betikleri yalnızca MATLAB ister.

kok   = fileparts(mfilename('fullpath'));
hedef = fullfile(kok, '..', 'sekiller');
if ~exist(hedef, 'dir'), mkdir(hedef); end

isler = {
    % klasör        betik                  ön komut                          şekil dosya adları (açılış sırasıyla)
    'ACC',          'ACC_yonetim',         '',                               {'acc_konvoy'}
    'CACC',         'CACC_yonetim',        '',                               {'cacc_konvoy', 'gecikme_headway_haritasi'}
    'CACC_Coklu',   'CACC_coklu_yonetim',  '',                               {'coklu_oncul'}
    'CACC_Coklu',   'CACC_coklu_olay',     'senaryo = ''arac_cikisi'';',     {'olay_arac_cikisi'}
    'CACC_Coklu',   'CACC_coklu_olay',     'senaryo = ''lider_ani_fren'';',  {'olay_lider_ani_fren'}
};

for k = 1:size(isler, 1)
    calistir(fullfile(kok, isler{k, 1}), isler{k, 2}, isler{k, 3}, isler{k, 4}, hedef);
end
fprintf('Şekiller kaydedildi: %s\n', hedef);
end

function calistir(klasor, betik, on_komut, adlar, hedef)
eski = cd(klasor);
geri = onCleanup(@() cd(eski));
close all force
evalin('base', 'clear');
if ~isempty(on_komut), evalin('base', on_komut); end
evalin('base', betik);          % betikler base çalışma alanında koşar (içlerinde clear var)

sekiller = findall(groot, 'Type', 'figure');
[~, sira] = sort([sekiller.Number]);
sekiller  = sekiller(sira);
for i = 1:numel(sekiller)
    if i <= numel(adlar)
        ad = adlar{i};
    else
        ad = sprintf('%s_%d', adlar{end}, i);
    end
    exportgraphics(sekiller(i), fullfile(hedef, [ad '.png']), 'Resolution', 130);
    fprintf('  %s.png  <- %s\n', ad, sekiller(i).Name);
end
close all force
end
