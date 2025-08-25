# main_controller.py

import time
import cv2
import numpy as np
import threading 

# Diğer modüllerimizi içe aktarıyoruz
# STM32 iletişim fonksiyonları
from stm32_comm import get_imu_data, get_encoder_data, get_vehicle_speed, is_ready, \
                               set_motor_speed_and_direction, apply_brake, release_brake, \
                               set_lazer_yonu, activate_laser, deactivate_laser
        
# Görüntü işleme fonksiyonları
from image_processing import detect_parkour_label, find_path_center, detect_aim_target, \
                                     detect_cones, detect_bumps, detect_finish_line, check_for_crossed_line_over_4 # check_for_crossed_line_over_4 eklendi
        
from logger import logger 

# Sabit Değerler (Şartnameden ve Tasarımdan Gelenler)
MAX_SPEED_PERCENTAGE = 100 
RAMP_CATALOG_PITCH_ANGLE = 24.5 
ANGLE_TOLERANCE_DEG = 1.0 
MIN_VEHICLE_SPEED_THRESHOLD = 0.1 
ATIS_TARGET_DISTANCE_M = 10.0 
RAMP_WAIT_DURATION_SEC = 2.0 
LOOP_RATE_HZ = 50 

# Durum Tanımları (Okunabilirliği artırmak için)
STATE_INITIALIZING = "INITIALIZING"
STATE_START = "START"
STATE_DIK_ENGEL_GECIS = "DIK_ENGEL_GECIS" # Dik Engel (1)
STATE_CAKIL_TASI_YOL_GECIS = "CAKIL_TASI_YOL_GECIS" # Taşlı/Çakıllı Yol (2)
STATE_YAN_EGIM_GECIS = "YAN_EGIM_GECIS" # Yan Eğim (3)
STATE_HIZLANMA_BOLGESI = "HIZLANMA_BOLGESI" # Hızlanma (4)
STATE_SIG_SU_GECIS = "SIG_SU_GECIS" # Sığ Su / Sudan Geçiş (5)
STATE_TRAFIK_KONILERI_GECIS = "TRAFIK_KONILERI_GECIS" # Trafik Konileri (6)
STATE_ENGEBELI_ARAZI_GECIS = "ENGEBELI_ARAZI_GECIS" # Engebeli Arazi (7)
STATE_DIK_EGIM_CIKIS_DURMA = "DIK_EGIM_CIKIS_DURMA" # Dik Eğim ve Atış - Çıkış Durma (8)
STATE_ATIS_BOLGESI = "ATIS_BOLGESI" # Dik Eğim ve Atış - Atış (9)
STATE_DIK_EGIM_INIS_DURMA = "DIK_EGIM_INIS_DURMA" # Dik Eğim ve Atış - İniş Durma (10)
STATE_FINISH = "FINISH" # Bitiş (11)
STATE_EMERGENCY_STOP = "EMERGENCY_STOP" # Acil durdurma durumu

class RobotController:
    def __init__(self):
        self.current_state = STATE_INITIALIZING
        self.system_terminated = False
        self.hizlanma_sure_baslangic = None
        self.last_ramp_stop_time = 0 
        self.total_path_distance = 0 
        self.parkur_mesafe_thresholds = {
            "YAN_EGIM_BITIS": 20.0, 
        }
        
        self.front_camera = cv2.VideoCapture(0) 
        self.aim_camera = cv2.VideoCapture(1) 
        
        self.front_camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.front_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.aim_camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.aim_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        if not self.front_camera.isOpened() or not self.aim_camera.isOpened():
            logger.log_error("Kameralar başlatılamadı! Lütfen bağlantıları ve ID'leri kontrol edin.")
            self.system_terminated = True
        else:
            logger.log_info("Kameralar başarıyla başlatıldı.")

        logger.log_info("Robot Kontrol Sistemi Başlatılıyor...")

    def read_sensor_data(self):
        imu_data = get_imu_data()
        encoder_data = get_encoder_data()
        vehicle_speed = get_vehicle_speed() 
        
        return imu_data, encoder_data, vehicle_speed

    def process_camera_data(self):
        ret_front, frame_front = self.front_camera.read()
        ret_aim, frame_aim = self.aim_camera.read()

        if not ret_front or not ret_aim:
            logger.log_warning("Kamera görüntüsü alınamadı! Bazı algılamalar atlanabilir.")
            return None, None, None, None, None, None 

        detected_label, label_center = detect_parkour_label(frame_front)
        path_center_offset = find_path_center(frame_front)
        aim_target_coords = detect_aim_target(frame_aim)
        
        return frame_front, frame_aim, detected_label, label_center, path_center_offset, aim_target_coords

    def update_state(self, current_frame_front, detected_label, current_vehicle_speed, imu_data, encoder_data):
        next_state = self.current_state 

        if self.current_state == STATE_INITIALIZING:
            logger.log_info("Sistem başlatılıyor, hazır ol komutu bekleniyor.")
            if is_ready(): 
                next_state = STATE_START
                logger.log_info("Sistem hazır. Başlangıç durumuna geçiliyor.")

        elif self.current_state == STATE_START:
            if detected_label == "DIK_ENGEL_1": 
                next_state = STATE_DIK_ENGEL_GECIS
                logger.log_info("Geçiş: Başlangıç -> Dik Engel aşaması (1).")

        elif self.current_state == STATE_DIK_ENGEL_GECIS:
            if detected_label == "CAKIL_TASI_YOL_2": 
                next_state = STATE_CAKIL_TASI_YOL_GECIS
                logger.log_info("Geçiş: Dik Engel -> Çakıllı/Taşlı Yol aşamasına geçiliyor (2).")

        elif self.current_state == STATE_CAKIL_TASI_YOL_GECIS:
            if detected_label == "YAN_EGIM_3": 
                next_state = STATE_YAN_EGIM_GECIS
                logger.log_info("Geçiş: Çakıllı/Taşlı Yol -> Yan Eğim aşamasına geçiliyor (3).")

        elif self.current_state == STATE_YAN_EGIM_GECIS:
            if encoder_data.distance_since_last_label > self.parkur_mesafe_thresholds["YAN_EGIM_BITIS"] or \
               detect_finish_line(current_frame_front): 
                next_state = STATE_HIZLANMA_BOLGESI
                logger.log_info("Geçiş: Yan Eğim -> Hızlanma Bölgesi aşamasına geçiliyor (4).")

        elif self.current_state == STATE_HIZLANMA_BOLGESI:
            if detected_label == "HIZLANMA_BITIS_UZERI_CIZILI_4":
                if self.hizlanma_sure_baslangic is not None:
                    hizlanma_suresi = time.time() - self.hizlanma_sure_baslangic
                    logger.log_info(f"Hızlanma Bölgesi tamamlandı! Süre: {hizlanma_suresi:.2f} saniye.")
                    self.hizlanma_sure_baslangic = None 
                next_state = STATE_SIG_SU_GECIS
                logger.log_info("Geçiş: Hızlanma Bölgesi -> Sığ Su aşamasına geçiliyor (5).")

        elif self.current_state == STATE_SIG_SU_GECIS:
            if detected_label == "TRAFIK_KONILERI_6": 
                next_state = STATE_TRAFIK_KONILERI_GECIS
                logger.log_info("Geçiş: Sığ Su -> Trafik Konileri aşamasına geçiliyor (6).")

        elif self.current_state == STATE_TRAFIK_KONILERI_GECIS:
            if detected_label == "ENGEBELI_ARAZI_7": 
                next_state = STATE_ENGEBELI_ARAZI_GECIS
                logger.log_info("Geçiş: Trafik Konileri -> Engebeli Arazi aşamasına geçiliyor (7).")

        elif self.current_state == STATE_ENGEBELI_ARAZI_GECIS:
            if detected_label == "DIK_EGIM_CIKIS_STOP_8": 
                next_state = STATE_DIK_EGIM_CIKIS_DURMA
                logger.log_info("Geçiş: Engebeli Arazi -> Dik Eğim Çıkış ve Durma aşamasına geçiliyor (8).")

        elif self.current_state == STATE_DIK_EGIM_CIKIS_DURMA:
            pass

        elif self.current_state == STATE_ATIS_BOLGESI:
            if detected_label == "DIK_EGIM_INIS_STOP_10": 
                next_state = STATE_DIK_EGIM_INIS_DURMA
                logger.log_info("Geçiş: Atış Bölgesi -> Dik Eğim İniş ve Durma aşamasına geçiliyor (10).")

        elif self.current_state == STATE_DIK_EGIM_INIS_DURMA:
            pass

        elif self.current_state == STATE_FINISH:
            if detected_label == "BITIS_11" or detect_finish_line(current_frame_front): 
                self.system_terminated = True
                logger.log_info("Parkur tamamlandı! Sistem sonlandırılıyor.")
        
        return next_state

    def execute_state_actions(self, frame_front, frame_aim, detected_label, path_center_offset, imu_data, vehicle_speed, aim_target_coords):
        global hizlanma_sure_baslangic # Global değişkene erişim için

        if self.current_state == STATE_INITIALIZING:
            set_motor_speed_and_direction(0, 0) 
            logger.log_info("Başlatma: Sistem bekleniyor...")

        elif self.current_state == STATE_START:
            target_speed = MAX_SPEED_PERCENTAGE * 0.7 
            steering_correction = self._calculate_steering_correction(path_center_offset)
            set_motor_speed_and_direction(target_speed, steering_correction)
            logger.log_debug(f"Başlangıç: Hız={target_speed}%, Yön={steering_correction:.2f}")

        elif self.current_state == STATE_DIK_ENGEL_GECIS:
            target_speed = MAX_SPEED_PERCENTAGE * 0.5 
            steering_correction = self._calculate_steering_correction(path_center_offset)
            set_motor_speed_and_direction(target_speed, steering_correction)
            logger.log_debug(f"Dik Engel: Hız={target_speed}%, Yön={steering_correction:.2f}")

        elif self.current_state == STATE_CAKIL_TASI_YOL_GECIS:
            target_speed = MAX_SPEED_PERCENTAGE * 1.0 
            steering_correction = self._calculate_steering_correction(path_center_offset)
            set_motor_speed_and_direction(target_speed, steering_correction)
            logger.log_debug(f"Çakıllı Yol: Hız={target_speed}%, Yön={steering_correction:.2f}")

        elif self.current_state == STATE_YAN_EGIM_GECIS:
            target_speed = MAX_SPEED_PERCENTAGE * 0.5 
            steering_correction = self._calculate_steering_correction(path_center_offset)
            
            roll_angle = imu_data.roll 
            balance_correction = self._calculate_balance_correction(roll_angle) 
            
            set_motor_speed_and_direction(target_speed, steering_correction + balance_correction)
            logger.log_debug(f"Yan Eğim: Hız={target_speed}%, Yön={steering_correction+balance_correction:.2f}, Roll={roll_angle:.2f}°")

        elif self.current_state == STATE_HIZLANMA_BOLGESI:
            if self.hizlanma_sure_baslangic is None: 
                self.hizlanma_sure_baslangic = time.time()
                logger.log_info("Hızlanma süresi başlatıldı.")
            
            target_speed = MAX_SPEED_PERCENTAGE * 1.0 
            steering_correction = self._calculate_steering_correction(path_center_offset)
            set_motor_speed_and_direction(target_speed, steering_correction)
            logger.log_debug(f"Hızlanma Bölgesi: Hız={target_speed}%, Yön={steering_correction:.2f}")

        elif self.current_state == STATE_SIG_SU_GECIS:
            target_speed = MAX_SPEED_PERCENTAGE * 0.2 
            steering_correction = self._calculate_steering_correction(path_center_offset)
            set_motor_speed_and_direction(target_speed, steering_correction)
            logger.log_debug(f"Sığ Su: Hız={target_speed}%, Yön={steering_correction:.2f}")

        elif self.current_state == STATE_TRAFIK_KONILERI_GECIS:
            target_speed = MAX_SPEED_PERCENTAGE * 0.3 
            detected_cones = detect_cones(frame_front) 
            
            steering_correction = self._calculate_cone_avoidance(path_center_offset, detected_cones)
            
            set_motor_speed_and_direction(target_speed, steering_correction)
            logger.log_debug(f"Trafik Konileri: Hız={target_speed}%, Yön={steering_correction:.2f}, Koni Sayısı={len(detected_cones)}")

        elif self.current_state == STATE_ENGEBELI_ARAZI_GECIS:
            target_speed = MAX_SPEED_PERCENTAGE * 0.4 
            steering_correction = self._calculate_steering_correction(path_center_offset)
            
            detected_bumps = detect_bumps(frame_front) 
            
            set_motor_speed_and_direction(target_speed, steering_correction)
            logger.log_debug(f"Engebeli Arazi: Hız={target_speed}%, Yön={steering_correction:.2f}")

        elif self.current_state == STATE_DIK_EGIM_CIKIS_DURMA:
            target_speed = MAX_SPEED_PERCENTAGE * 0.3 
            current_pitch = imu_data.pitch 
            
            if abs(current_pitch - RAMP_CATALOG_PITCH_ANGLE) < ANGLE_TOLERANCE_DEG and vehicle_speed > MIN_VEHICLE_SPEED_THRESHOLD:
                set_motor_speed_and_direction(0, 0) 
                apply_brake() 
                logger.log_info(f"Dik Eğim Çıkışında duruldu (Pitch: {current_pitch:.2f}°). {RAMP_WAIT_DURATION_SEC} saniye bekleniyor.")
                time.sleep(RAMP_WAIT_DURATION_SEC) 
                release_brake() 
                logger.log_info("Rampada bekleme tamamlandı. Atış Bölgesine geçiliyor.")
                self.current_state = STATE_ATIS_BOLGESI 
            else:
                steering_correction = self._calculate_steering_correction(path_center_offset)
                set_motor_speed_and_direction(target_speed, steering_correction)
                logger.log_debug(f"Dik Eğim Çıkış: Hız={target_speed}%, Yön={steering_correction:.2f}, Pitch={current_pitch:.2f}°")

        elif self.current_state == STATE_ATIS_BOLGESI:
            set_motor_speed_and_direction(0, 0) 
            logger.log_info("Atış Bölgesi: Araç durduruldu. Hedef aranıyor.")

            for attempt in range(3): 
                aim_target_coords = detect_aim_target(frame_aim) 
                if aim_target_coords:
                    pan_angle, tilt_angle = self._calculate_laser_angles(aim_target_coords)
                    set_lazer_yonu(pan_angle, tilt_angle)
                    time.sleep(0.5) 
                    
                    activate_laser() 
                    logger.log_info(f"Atış: Lazer aktif. {attempt+1}. deneme.")
                    time.sleep(1.0) 
                    deactivate_laser() 
                    
                    logger.log_info(f"Atış başarılı! {attempt+1}. deneme. Puanlama için hedefi kaydet.")
                    break 
                else:
                    logger.log_warning(f"Hedef bulunamadı, {attempt+1}. deneme başarısız.")
            
            logger.log_info("Atış denemeleri tamamlandı. İniş rampasına geçiliyor.")
            self.current_state = STATE_DIK_EGIM_INIS_DURMA 

        elif self.current_state == STATE_DIK_EGIM_INIS_DURMA:
            target_speed = MAX_SPEED_PERCENTAGE * 0.2 
            current_pitch = imu_data.pitch
            
            if abs(current_pitch - RAMP_CATALOG_PITCH_ANGLE) < ANGLE_TOLERANCE_DEG and vehicle_speed > MIN_VEHICLE_SPEED_THRESHOLD:
                set_motor_speed_and_direction(0, 0)
                apply_brake()
                logger.log_info(f"Dik Eğim İnişinde duruldu (Pitch: {current_pitch:.2f}°). {RAMP_WAIT_DURATION_SEC} saniye bekleniyor.")
                time.sleep(RAMP_WAIT_DURATION_SEC)
                release_brake()
                logger.log_info("Rampada bekleme tamamlandı. Bitiş Çizgisine geçiliyor.")
                self.current_state = STATE_FINISH 
            else:
                steering_correction = self._calculate_steering_correction(path_center_offset)
                set_motor_speed_and_direction(target_speed, steering_correction)
                logger.log_debug(f"Dik Eğim İniş: Hız={target_speed}%, Yön={steering_correction:.2f}, Pitch={current_pitch:.2f}°")

        elif self.current_state == STATE_FINISH:
            set_motor_speed_and_direction(0, 0)
            apply_brake()
            logger.log_info("Parkur başarıyla tamamlandı. Araç durduruldu.")
            self.system_terminated = True
            
        elif self.current_state == STATE_EMERGENCY_STOP:
            set_motor_speed_and_direction(0, 0)
            apply_brake()
            logger.log_error("ACİL DURDURMA ETKİN! Sistem manuel olarak durduruldu veya kritik hata oluştu.")
            self.system_terminated = True 

    def _calculate_steering_correction(self, path_center_offset):
        Kp = 0.5 
        steering_output = Kp * path_center_offset
        return np.clip(steering_output, -1.0, 1.0)

    def _calculate_balance_correction(self, roll_angle):
        Kp_roll = 0.1 
        return Kp_roll * roll_angle

    def _calculate_cone_avoidance(self, path_center_offset, detected_cones):
        base_steering_correction = path_center_offset 
        avoidance_steering = 0.0 

        if detected_cones:
            avg_cone_x_center = sum([c['center_x'] for c in detected_cones]) / len(detected_cones)
            
            image_center_x = self.front_camera.get(cv2.CAP_PROP_FRAME_WIDTH) / 2
            
            offset_from_vehicle_center = avg_cone_x_center - image_center_x

            tallest_cone = max(detected_cones, key=lambda c: c['h'])
            cone_urgency_factor = min(1.0, tallest_cone['h'] / 300.0) 

            if offset_from_vehicle_center < 0: 
                avoidance_direction_sign = 1 
            else: 
                avoidance_direction_sign = -1 
            
            avoidance_magnitude = np.clip(abs(offset_from_vehicle_center) / image_center_x, 0.0, 1.0) 
            
            avoidance_steering = avoidance_direction_sign * avoidance_magnitude * 0.5 * cone_urgency_factor 
            logger.log_debug(f"Koni Kaçınma: Ofset={offset_from_vehicle_center:.2f}, Aciliyet={cone_urgency_factor:.2f}, Düzeltme={avoidance_steering:.2f}")

        final_steering_correction = base_steering_correction + avoidance_steering
        return np.clip(final_steering_correction, -1.0, 1.0) 

    def _calculate_laser_angles(self, aim_target_coords):
        img_width = self.aim_camera.get(cv2.CAP_PROP_FRAME_WIDTH)
        img_height = self.aim_camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
        
        pixel_x, pixel_y = aim_target_coords
        
        FOV_X_DEG = 60 
        FOV_Y_DEG = 45 
        
        offset_x = pixel_x - img_width / 2
        offset_y = pixel_y - img_height / 2
        
        pan_angle = (offset_x / (img_width / 2)) * (FOV_X_DEG / 2)
        tilt_angle = (offset_y / (img_height / 2)) * (FOV_Y_DEG / 2) * -1 
        
        logger.log_debug(f"Lazer hedefleniyor: Pan={pan_angle:.2f}°, Tilt={tilt_angle:.2f}°")
        return pan_angle, tilt_angle

    def run(self):
        logger.log_info("Robot Kontrol Döngüsü Başlatılıyor.")
        while not self.system_terminated:
            loop_start_time = time.time()

            imu_data, encoder_data, vehicle_speed = self.read_sensor_data()
            frame_front, frame_aim, detected_label, label_center, path_center_offset, aim_target_coords = self.process_camera_data()

            if frame_front is None or frame_aim is None:
                logger.log_error("Kamera görüntüsü alınamadı, bu döngü atlanıyor.")
                time.sleep(1.0 / LOOP_RATE_HZ) 
                continue 
            
            next_state = self.update_state(frame_front, detected_label, vehicle_speed, imu_data, encoder_data)

            self.execute_state_actions(frame_front, frame_aim, detected_label, path_center_offset, imu_data, vehicle_speed, aim_target_coords)

            self.current_state = next_state

            elapsed_time = time.time() - loop_start_time
            sleep_time = (1.0 / LOOP_RATE_HZ) - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                logger.log_warning(f"Kontrol döngüsü gecikti! İşlem süresi: {elapsed_time:.4f}s")
                
            if not self.system_terminated and not is_ready(): 
                pass 

        logger.log_info("Robot Kontrol Döngüsü Sonlandırıldı.")
        self.front_camera.release()
        self.aim_camera.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    controller = RobotController()
    controller.run()