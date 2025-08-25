# logger.py

import time

class Logger:
    def log_info(self, message):
        """Bilgilendirici mesajları kaydeder."""
        print(f"[INFO] {time.strftime('%Y-%m-%d %H:%M:%S')} {message}")

    def log_warning(self, message):
        """Uyarı mesajlarını kaydeder."""
        print(f"[WARNING] {time.strftime('%Y-%m-%d %H:%M:%S')} {message}")

    def log_error(self, message):
        """Hata mesajlarını kaydeder ve ciddi sorunları belirtir."""
        print(f"[ERROR] {time.strftime('%Y-%m-%d %H:%M:%S')} {message}")

    def log_debug(self, message):
        """
        Detaylı hata ayıklama mesajlarını kaydeder.
        Genellikle sadece geliştirme aşamasında etkindir.
        """
        # Debug mesajlarını görmek istersen aşağıdaki satırın yorumunu kaldırabilirsin.
        # print(f"[DEBUG] {time.strftime('%Y-%m-%d %H:%M:%S')} {message}")
        pass # Şimdilik pasif, çok fazla çıktı vermemek için

# Logger sınıfının bir örneğini oluşturup global olarak erişilebilir yapıyoruz
logger = Logger()