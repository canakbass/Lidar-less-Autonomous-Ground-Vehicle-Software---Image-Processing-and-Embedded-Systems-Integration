// robot_control.c (Implementasyon dosyası)

#include "robot_control.h"
#include <stdio.h>  // sprintf için
#include <string.h> // strcmp, strstr, memset için
#include <stdlib.h> // atof, atoi için
#include <math.h>   // fabsf (float için abs), sin, cos, atan2f vb. için

// --- Global Veriler ---
// UART alım tamponu ve bayrağı
volatile char uart_rx_buffer[64]; 
volatile uint8_t uart_rx_flag = 0; // Yeni veri geldiğinde 1 olacak

// Sensör ve araç durumu verileri
IMU_Data_t current_imu_data = {0};
Encoder_Data_t current_encoder_data = {0};
float current_vehicle_speed = 0.0;

// Traktör motoru gaz kelebeği için servo PWM değişkenleri
// Bu değerler mikrosaniye cinsinden pulse genişliğini temsil eder (örn. 1000-2000 us)
uint32_t gas_servo_pulse = 1500; // Başlangıçta rölanti/orta konum

// Lazer pan/tilt servoları için PWM değişkenleri
uint32_t laser_pan_servo_pulse = 1500;
uint32_t laser_tilt_servo_pulse = 1500;

// Enkoder sayımını resetlemek için başlangıç değerleri
int32_t left_encoder_reset_count = 0;
int32_t right_encoder_reset_count = 0;

// --- Dahili Yardımcı Fonksiyonlar (İsteğe bağlı) ---
// Servo pulse değerini açıya çeviren fonksiyon (kalibrasyon gerektirir)
uint32_t map_angle_to_pulse(float angle);
// Hız yüzdesini gaz kelebeği servo pulse'ına çeviren (kalibrasyon gerektirir)
uint32_t map_speed_to_gas_pulse(int speed_percentage);

// --- Fonksiyon Implementasyonları ---

void robot_init(void) {
    // HAL kütüphaneleri ve çevre birimleri (UART, TIM, GPIO vb.) MX_Cube tarafından ayarlanmış olmalı.

    // Motor PWM'lerini başlat (varsa elektrik motoru sürücüleri için)
    // Eğer traktör motoru doğrudan paletleri sürmüyorsa bu kısım olmayabilir veya farklı kontrol mekanizması olur.
    // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Sol Motor
    // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // Sağ Motor

    // Gaz kelebeği servosu PWM'ini başlat
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // GAS_SERVO_PWM_PIN'e karşılık gelmeli

    // Lazer pan/tilt servoları PWM'ini başlat
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // LASER_PAN_SERVO_PWM_PIN
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // LASER_TILT_SERVO_PWM_PIN

    // Enkoder timer'larını başlat (Encoder Modunda)
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // Sol enkoder (TIM3_CH1 ve TIM3_CH2)
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // Sağ enkoder (TIM4_CH1 ve TIM4_CH2)

    // UART alımını kesme modunda başlat
    // Bir sonraki alım için HAL_UART_Receive_IT() çağrısı yapılmalı
    HAL_UART_Receive_IT(&huart2, (uint8_t*)uart_rx_buffer, sizeof(uart_rx_buffer) - 1);
    
    // Motorları güvenli konuma al (dur) ve freni bırak
    set_motor_speed_and_direction(0, 0.0);
    release_brake();
    deactivate_laser();

    // Enkoder sayımlarını sıfırla (başlangıç noktası için)
    reset_encoder_distance(); // Yeni bir fonksiyon ekledik

    // Mini PC'ye hazır sinyali gönder
    char ready_msg[] = "READY\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)ready_msg, strlen(ready_msg), HAL_MAX_DELAY);
}

void robot_loop(void) {
    // Her döngüde sensör verilerini oku
    read_imu_data(&current_imu_data);
    read_encoder_data(&current_encoder_data);
    current_vehicle_speed = calculate_vehicle_speed(&current_encoder_data);
    current_encoder_data.distance_since_last_label = calculate_distance_traveled_since_reset(); // Yeni eklenen fonksiyon

    // Mini PC'ye sensör verilerini gönder
    send_sensor_data_to_mini_pc();

    // Yeni komut varsa işle
    if (uart_rx_flag) {
        uart_rx_flag = 0; // Bayrağı sıfırla
        process_mini_pc_command((const char*)uart_rx_buffer);
        // Tamponu temizle ve yeni alımı başlat
        memset((void*)uart_rx_buffer, 0, sizeof(uart_rx_buffer));
        HAL_UART_Receive_IT(&huart2, (uint8_t*)uart_rx_buffer, sizeof(uart_rx_buffer) - 1);
    }
}

// --- UART Kesme Geri Çağırım Fonksiyonu ---
// main.c dosyasında HAL_UART_RxCpltCallback içinde çağrılacak.
// Bu fonksiyon, UART alımı tamamlandığında tetiklenir.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == huart2.Instance) {
        uart_rx_flag = 1; // Yeni veri geldi bayrağını set et
    }
}

void process_mini_pc_command(const char* command_buffer) {
    // Komutları ayrıştırma ve ilgili fonksiyonları çağırma
    // Örnek komut formatları: "MOTOR:100,0.5", "BRAKE:APPLY", "LASER_POS:10.0,-5.0", "LASER:ON"
    if (strstr(command_buffer, "MOTOR:") != NULL) {
        int speed_perc;
        float steering_ang;
        // sscanf ile stringi ayrıştır
        if (sscanf(command_buffer, "MOTOR:%d,%f", &speed_perc, &steering_ang) == 2) {
            set_motor_speed_and_direction(speed_perc, steering_ang);
        }
    } else if (strcmp(command_buffer, "BRAKE:APPLY\n") == 0) { // Komut sonunda '\n' olabileceğini unutma
        apply_brake();
    } else if (strcmp(command_buffer, "BRAKE:RELEASE\n") == 0) {
        release_brake();
    } else if (strstr(command_buffer, "LASER_POS:") != NULL) {
        float pan, tilt;
        if (sscanf(command_buffer, "LASER_POS:%f,%f", &pan, &tilt) == 2) {
            set_lazer_yonu(pan, tilt);
        }
    } else if (strcmp(command_buffer, "LASER:ON\n") == 0) {
        activate_laser();
    } else if (strcmp(command_buffer, "LASER:OFF\n") == 0) {
        deactivate_laser();
    }
    // Debug için gelen komutu seri porttan başka bir kanala basabiliriz (SWV, ST-Link)
}

void send_sensor_data_to_mini_pc(void) {
    char tx_buffer[100];
    // IMU verisini (pitch, roll, yaw), enkoder sayımlarını ve hızı gönder
    // Format: "IMU:PITCH,ROLL,YAW,ENC:LEFT_COUNT,RIGHT_COUNT,DISTANCE,SPD:SPEED\n"
    sprintf(tx_buffer, "IMU:%.2f,%.2f,%.2f,ENC:%ld,%ld,%.2f,SPD:%.2f\n",
            current_imu_data.pitch, current_imu_data.roll, current_imu_data.yaw,
            current_encoder_data.left_count, current_encoder_data.right_count,
            current_encoder_data.distance_since_last_label, 
            current_vehicle_speed);
    // UART üzerinden gönder
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);
}

void set_motor_speed_and_direction(int speed_percentage, float steering_angle) {
    // speed_percentage: 0-100 (Mini PC'den gelen hız yüzdesi)
    // steering_angle: -1.0 (tam sol) ile 1.0 (tam sağ) arası (Mini PC'den gelen direksiyon komutu)

    // Traktör motoru gaz kelebeği kontrolü (PWM sinyali ile servo sürme)
    // 1000us (en düşük gaz) - 2000us (tam gaz) aralığında bir pulse genişliği haritalaması
    // Bu değerler ve fonksiyonlar gerçek motor ve servo kalibrasyonuna bağlıdır.
    gas_servo_pulse = map_speed_to_gas_pulse(speed_percentage); 
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, gas_servo_pulse); 
    
    // Paletli sistemin direksiyonu için ayrı bir mekanizma varsa (örn. hidrolik kontrol valfleri veya
    // elektrikli direksiyon motorları) bu `steering_angle` değeri ile kontrol edilir.
    // Traktör motorunun gücünü paletlere aktaran mekanizma karmaşıksa:
    // Örneğin, hidrolik pompa devrini ayarlayan oransal valfler veya mekanik vites aktüatörleri.
    // Bu kısım araç tasarımınıza göre uyarlanmalıdır.
    // Şu anki senaryoda gaz kelebeği kontrolü temel alınmıştır.
}

void apply_brake(void) {
    // Fren aktüatörünü etkinleştir (röle veya MOSFET ile kontrol)
    HAL_GPIO_WritePin(BRAKE_RELAY_PORT, BRAKE_RELAY_PIN, GPIO_PIN_SET); // Röleyi çek, freni uygula
}

void release_brake(void) {
    // Fren aktüatörünü devre dışı bırak
    HAL_GPIO_WritePin(BRAKE_RELAY_PORT, BRAKE_RELAY_PIN, GPIO_PIN_RESET); // Röleyi bırak, freni sal
}

void set_lazer_yonu(float pan_angle, float tilt_angle) {
    // Lazer Pan ve Tilt servo motorlarının PWM sinyallerini ayarla
    // Derece cinsinden açıları PWM pulse genişliklerine çevir (kalibrasyon gerektirir)
    laser_pan_servo_pulse = map_angle_to_pulse(pan_angle); // Örnek: 1500us merkez, +/- 500us = +/- 90 derece
    laser_tilt_servo_pulse = map_angle_to_pulse(tilt_angle);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, laser_pan_servo_pulse);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, laser_tilt_servo_pulse);
}

void activate_laser(void) {
    // Lazer modülünü açmak için pini yüksek yap
    HAL_GPIO_WritePin(LASER_ENABLE_PORT, LASER_ENABLE_PIN, GPIO_PIN_SET);
}

void deactivate_laser(void) {
    // Lazer modülünü kapatmak için pini düşük yap
    HAL_GPIO_WritePin(LASER_ENABLE_PORT, LASER_ENABLE_PIN, GPIO_PIN_RESET);
}

void read_imu_data(IMU_Data_t* imu_data) {
    // IMU sensöründen (örn. MPU6050, BNO055, LSM6DSO) I2C veya SPI ile verileri oku.
    // Burası seçtiğin IMU'nun kütüphanesi veya doğrudan register okumalarıyla doldurulacak.
    // Örneğin: MPU6050 için read_mpu6050_angles(imu_data->pitch, imu_data->roll, imu_data->yaw);
    // PLACEHOLDERS (Gerçek sensör entegrasyonu ile değişecek)
    imu_data->pitch = 2.0 + (float)HAL_GetTick() / 10000.0; // Simüle edilmiş değişim
    imu_data->roll = 0.5 + (float)HAL_GetTick() / 20000.0;
    imu_data->yaw = 90.0 + (float)HAL_GetTick() / 5000.0;
}

void read_encoder_data(Encoder_Data_t* encoder_data) {
    // Enkoder timer'larından (TIM_ENCODER_MODE) pals sayılarını oku
    // __HAL_TIM_GET_COUNTER() ile timer sayacının mevcut değerini al.
    encoder_data->left_count = __HAL_TIM_GET_COUNTER(&htim3); 
    encoder_data->right_count = __HAL_TIM_GET_COUNTER(&htim4); 
    
    // Her enkoder palsinin kaç cm veya metreye tekabül ettiğini hesapla (kalibrasyonla bulunur).
    // Örneğin: const float CM_PER_PULSE = 0.01; // Her pals 0.01 cm
    // encoder_data->distance_since_last_label += (delta_left_count + delta_right_count) / 2.0 * CM_PER_PULSE;
    
    // distance_since_last_label bilgisi için, Mini PC'den gelen "reset mesafe" komutları da işlenebilir.
    // Şimdilik sadece placeholder olarak enkoder sayımlarından türetelim.
    // (Ortalama sayım - Reset Sayım) * CM_PER_PULSE
    float avg_current_count = (float)(encoder_data->left_count + encoder_data->right_count) / 2.0;
    float avg_reset_count = (float)(left_encoder_reset_count + right_encoder_reset_count) / 2.0;
    encoder_data->distance_since_last_label = (avg_current_count - avg_reset_count) * 0.001; // Örnek cm/pals (0.001 cm/pals)
}

void reset_encoder_distance(void) {
    // Mesafe sayacını sıfırlamak için enkoderlerin anlık değerlerini kaydet
    left_encoder_reset_count = __HAL_TIM_GET_COUNTER(&htim3);
    right_encoder_reset_count = __HAL_TIM_GET_COUNTER(&htim4);
}

float calculate_vehicle_speed(Encoder_Data_t* encoder_data) {
    // Enkoder sayımlarındaki değişime göre anlık araç hızını hesapla.
    // Bu PID kontrol döngüsünde kullanılacak kritik bir geri bildirimdir.
    // Örneğin, belirli bir periyotta (robot_loop döngü sıklığı) ortalama hız.
    // (Mevcut pals - Önceki pals) / (geçen zaman) * (tekerlek çevresi / pals per devir)
    // Delta zamanı için HAL_GetTick() kullanılabilir veya bir timer kesmesi.
    static int32_t prev_left_count = 0;
    static int32_t prev_right_count = 0;
    static uint32_t prev_tick = 0;

    uint32_t current_tick = HAL_GetTick();
    float delta_time_sec = (float)(current_tick - prev_tick) / 1000.0f;

    if (delta_time_sec == 0) return 0.0f; // Bölme hatasını önle

    int32_t delta_left = encoder_data->left_count - prev_left_count;
    int32_t delta_right = encoder_data->right_count - prev_right_count;

    float avg_delta_count = (float)(delta_left + delta_right) / 2.0f;
    
    // Örneğin: 1 enkoder palsi 0.01 metreye denk geliyorsa
    const float METER_PER_PULSE = 0.0001; 
    current_vehicle_speed = (avg_delta_count * METER_PER_PULSE) / delta_time_sec; // m/s cinsinden hız

    prev_left_count = encoder_data->left_count;
    prev_right_count = encoder_data->right_count;
    prev_tick = current_tick;

    return current_vehicle_speed;
}

// --- Dahili Yardımcı Fonksiyon Implementasyonları ---
uint32_t map_angle_to_pulse(float angle) {
    // Servo motorun açı aralığını (örn. -90'dan +90'a) PWM pulse genişliği aralığına (örn. 1000us'den 2000us'ye) dönüştürür.
    // Buradaki değerler, kullanılan servo motorun teknik özelliklerine göre ayarlanmalıdır.
    // Örnek: angle = -90 -> 1000us, angle = 0 -> 1500us, angle = 90 -> 2000us
    return (uint32_t)(1500 + (angle / 90.0) * 500); 
}

uint32_t map_speed_to_gas_pulse(int speed_percentage) {
    // Hız yüzdesini traktör motorunun gaz kelebeği servo pulse'ına dönüştürür.
    // 0% hız genellikle rölanti (örn. 1200us), 100% hız tam gaz (örn. 1800us).
    // Bu değerler traktör motorunun ve servo'nun kalibrasyonuna bağlıdır.
    uint32_t min_pulse = 1200; // Rölanti veya minimum gaz
    uint32_t max_pulse = 1800; // Tam gaz
    return (uint32_t)(min_pulse + (float)speed_percentage / 100.0 * (max_pulse - min_pulse));
}