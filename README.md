# KOM4321 CACC Tasarımı — MATLAB/Simulink Kodları

1 lider + 3 takipçili konvoy için ACC ve CACC modelleri, V2V gecikmesi, çok öncül CACC ve olay senaryoları. MATLAB R2026b ile denendi.

| Klasör | İçerik | Gereken |
|--------|--------|---------|
| `ACC/` | `ACC_yonetim.m` (parametre + simülasyon + metrik), `ACC_konvoy.slx`, `ACC_gorsellestirme.m` | Simulink |
| `CACC/` | `CACC_yonetim.m` (ACC/CACC karşılaştırması, sabit/değişken gecikme, gecikme-headway haritası), `CACC_konvoy.slx`, `CACC_gorsellestirme.m` | Simulink |
| `CACC_Coklu/` | Simulink gerektirmeyen çok öncül modeli ve olay senaryoları | Yalnızca MATLAB |

Çalıştırma: ilgili klasöre geç, yönetim dosyasını çalıştır (F5).

```matlab
cd CACC
CACC_yonetim

cd ../CACC_Coklu
senaryo = 'arac_cikisi';  CACC_coklu_olay
```

`sekilleri_uret` bütün yönetim betiklerini koşturur ve şekilleri bu deponun bir üst klasörüne, `../sekiller/` altına PNG olarak yazar (proje notları oradan okur).

Denklemler, parametre seçimleri ve sonuçların açıklaması depo dışındaki proje notlarındadır (`teknik/1D_Arac_Dinamigi_Denklemler.html`). `*.mp4` videoları ve Simulink önbelleği Git'e girmez; betiklerle yeniden üretilir.
