# TEKNOFEST 2025 İnsansız Kara Aracı Otonomi Projesi

Bu proje, TEKNOFEST 2025 İnsansız Kara Aracı Yarışması için otonom, paletli bir kara aracının yazılım ve donanım mimarisini içerir. Sistem, Python ve C/C++ dillerinde modüler olarak geliştirilmiştir.// Proje derinlik algısı olmadığından doğru responselar veremiyor bunu algoritmayı değiştirerek düzenleyebiliriz ancak yterince efektif olmadığını düşünerek burada bırakma kararı aldım.  

## Klasör Yapısı ve Dosyalar

- `main_controller.py` : Ana kontrol ve durum makinesi (Python)
- `image_processing.py` : Görüntü işleme algoritmaları (Python)
- `stm32_comm.py` : STM32 ile seri haberleşme (Python)
- `logger.py` : Loglama modülü (Python)
- `robot_control.h` : STM32 donanım kontrol header dosyası (C)
- `robot_control.c` : STM32 donanım kontrol implementasyonu (C)
- `c_cpp_properties.json` : VSCode C/C++ IntelliSense ayarları

## Sistem Özeti

- **Mini PC (Jetson/Raspberry Pi)**: Durum makinesi, görüntü işleme, STM32 ile haberleşme
- **STM32**: Motor, fren, lazer, sensör okuma ve güvenlik
- **Kameralar**: Yol, engel, tabela ve hedef algılama
- **Sensörler**: IMU, enkoder

## Çalıştırma
- Python modülleri için: `python main_controller.py`
- STM32 kodu için: STM32CubeIDE ile derleyip karta yükleyin.

## Notlar
- Tüm dosyalar ve yollar C:\test\ika altında olmalıdır.
- STM32 HAL ve CubeMX dosyaları STM32CubeIDE ile oluşturulmalıdır.
- Detaylı algoritma ve fonksiyon açıklamaları dosya içinde mevcuttur.

---
Proje ile ilgili sorularınız için: canakbasforspecial@gmail.com


