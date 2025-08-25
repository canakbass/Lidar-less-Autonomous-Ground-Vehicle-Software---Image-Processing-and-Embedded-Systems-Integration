# stm32_comm.py

import serial
import threading
import time
import json # İleride daha karmaşık veri yapıları için kullanılabilir
from logger import logger # Loglama modülünü içe aktarıyoruz

# STM32 Seri Port Ayarları
SERIAL_PORT = '/dev/ttyUSB0' # Ubuntu'da STM32'nin göründüğü port
BAUD_RATE = 115200 # Haberleşme hızı
SERIAL_TIMEOUT = 0.1 # Saniye cinsinden okuma zaman aşımı

# Global değişkenler (STM32'den gelen verileri saklamak için)
_imu_data = {'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0}
_encoder_data = {'left_count': 0, 'right_count': 0, 'distance_since_last_label': 0.0}
_vehicle_speed = 0.0 # m/s
_stm32_ready = False
_last_received_time = time.time() # En son ne zaman veri alındığını takip etmek için

# Seri port nesnesi
ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT)
    logger.log_info(f"Seri port {SERIAL_PORT} {BAUD_RATE} baudrate ile açıldı.")
    # İlk başta seri portun boşalmasını bekleyebiliriz
    time.sleep(1) 
    ser.flushInput()
    ser.flushOutput()
except serial.SerialException as e:
    logger.log_error(f"Seri port açılamadı: {e}. Lütfen bağlantıyı ve izinleri kontrol edin.")
    ser = None

def _read_from_stm32_thread():
    """Arka planda STM32'den sürekli veri okuyan thread fonksiyonu."""
    if ser is None:
        logger.log_error("Seri port açık değil, okuma thread'i başlatılamıyor.")
        return

    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                global _imu_data, _encoder_data, _vehicle_speed, _stm32_ready, _last_received_time
                _last_received_time = time.time() # Veri alındı, zamanı güncelle
                # Gelen veriyi ayrıştır (örneğin "IMU:1.2,3.4,5.6,ENC:1000,1050,15.2,SPD:2.5")
                parts = line.split(',')
                for part in parts:
                    if part.startswith("IMU:"):
                        try:
                            angles = list(map(float, part[4:].split(',')))
                            if len(angles) == 3:
                                _imu_data['pitch'] = angles[0]
                                _imu_data['roll'] = angles[1]
                                _imu_data['yaw'] = angles[2]
                        except ValueError:
                            logger.log_warning(f"IMU verisi ayrıştırılamadı: {part}")
                    elif part.startswith("ENC:"):
                        try:
                            counts = list(map(float, part[4:].split(','))) 
                            if len(counts) == 3:
                                _encoder_data['left_count'] = int(counts[0])
                                _encoder_data['right_count'] = int(counts[1])
                                _encoder_data['distance_since_last_label'] = counts[2]
                        except ValueError:
                            logger.log_warning(f"Enkoder verisi ayrıştırılamadı: {part}")
                    elif part.startswith("SPD:"):
                        try:
                            _vehicle_speed = float(part[4:])
                        except ValueError:
                            logger.log_warning(f"Hız verisi ayrıştırılamadı: {part}")
                    elif part == "READY":
                        _stm32_ready = True
                        logger.log_info("STM32 hazır sinyali alındı.")
            else:
                pass 
        except serial.SerialException:
            logger.log_error("Seri port bağlantısı kesildi.")
            break 
        except Exception as e:
            logger.log_error(f"STM32 okuma hatası: {e}")
        time.sleep(0.005) 

# Arka planda okuma thread'ini başlat
if ser:
    read_thread = threading.Thread(target=_read_from_stm32_thread, daemon=True)
    read_thread.start()
    logger.log_info("STM32 okuma thread'i başlatıldı.")

# Sensör verilerini dışarıya sunan fonksiyonlar
def get_imu_data():
    """En son okunan IMU verisini döndürür."""
    class IMUData:
        def __init__(self, pitch, roll, yaw):
            self.pitch = pitch
            self.roll = roll
            self.yaw = yaw
    return IMUData(_imu_data['pitch'], _imu_data['roll'], _imu_data['yaw'])

def get_encoder_data():
    """En son okunan enkoder verisini döndürür."""
    class EncoderData:
        def __init__(self, left_count, right_count, distance_since_last_label):
            self.left_count = left_count
            self.right_count = right_count
            self.distance_since_last_label = distance_since_last_label
    return EncoderData(_encoder_data['left_count'], _encoder_data['right_count'], _encoder_data['distance_since_last_label'])

def get_vehicle_speed():
    """En son okunan araç hızını döndürür."""
    return _vehicle_speed

def is_ready():
    """STM32'nin hazır olup olmadığını döndürür."""
    return _stm32_ready

def send_command(command_string):
    """STM32'ye komut stringi gönderir."""
    if ser and ser.is_open:
        try:
            ser.write(command_string.encode('utf-8'))
            logger.log_debug(f"STM32'ye gönderilen: {command_string.strip()}")
        except serial.SerialException as e:
            logger.log_error(f"Seri porta yazma hatası: {e}")
    else:
        logger.log_warning("Seri port kapalı veya mevcut değil, komut gönderilemedi.")

# Mini PC'den STM32'ye gönderilecek kontrol fonksiyonları
def set_motor_speed_and_direction(speed_percentage, steering_angle):
    """
    Motor hızını ve direksiyon açısını STM32'ye gönderir.
    speed_percentage: 0-100 arası (% olarak hız)
    steering_angle: -1.0 (tam sol) ile 1.0 (tam sağ) arası
    """
    command = f"MOTOR:{int(speed_percentage)},{steering_angle:.2f}\n"
    send_command(command)

def apply_brake():
    """Fren uygulama komutu gönderir."""
    send_command(b"BRAKE:APPLY\n")

def release_brake():
    """Fren bırakma komutu gönderir."""
    send_command(b"BRAKE:RELEASE\n")

def set_lazer_yonu(pan_angle, tilt_angle):
    """
    Lazerin pan/tilt açılarını STM32'ye gönderir.
    pan_angle, tilt_angle: Derece cinsinden açılar
    """
    command = f"LASER_POS:{pan_angle:.2f},{tilt_angle:.2f}\n"
    send_command(command)

def activate_laser():
    """Lazer aktif etme komutu gönderir."""
    send_command(b"LASER:ON\n")

def deactivate_laser():
    """Lazer pasif etme komutu gönderir."""
    send_command(b"LASER:OFF\n")