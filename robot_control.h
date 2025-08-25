// robot_control.h (Header dosyası)

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "stm32f446_hal.h" // HAL kütüphaneleri için

// IMU Veri Yapısı: Mini PC'ye gönderilecek sensör verileri
typedef struct {
    float pitch;
    float roll;
    float yaw;
} IMU_Data_t;

// Enkoder Veri Yapısı: Mini PC'ye gönderilecek sensör verileri
typedef struct {
    int32_t left_count;  // Sol enkoder pals sayısı
    int32_t right_count; // Sağ enkoder pals sayısı
    float distance_since_last_label; // Son tabela algılamasından bu yana katedilen mesafe (metre)
} Encoder_Data_t;

// --- Global Çevre Birimi Değişkenleri (MX_Cube veya projenizde tanımlanmalı) ---
// Bu değişkenler genellikle main.c dosyasında veya başka bir başlatma dosyasında
// HAL kütüphaneleri tarafından otomatik olarak oluşturulur ve dışarıya aktarılır (extern).
extern UART_HandleTypeDef huart2; // UART bağlantısı için (Mini PC ile haberleşme)
extern TIM_HandleTypeDef htim1;   // Motor PWM üretimi için (Örnek: Motor sürücüleri)
extern TIM_HandleTypeDef htim3;   // Enkoder (Sol Motor) sayımı ve Lazer Pan servo PWM için
extern TIM_HandleTypeDef htim4;   // Enkoder (Sağ Motor) sayımı ve Gaz Kelebeği servo PWM için
extern ADC_HandleTypeDef hadc1;   // Motor akım/gerilim okumaları için (Opsiyonel: Kullanılıyorsa eklenmeli)

// --- Fonksiyon Prototipleri ---
// Robot sistemini başlatır (çevre birimi initleri, başlangıç ayarları)
void robot_init(void); 
// Robotun ana döngüsüdür, sürekli çalışır
void robot_loop(void); 

// Mini PC'den gelen komutları işler
void process_mini_pc_command(const char* command_buffer);
// Sensör verilerini Mini PC'ye gönderir
void send_sensor_data_to_mini_pc(void);

// Motor kontrol fonksiyonu
// speed_percentage: %0-100 arası motor hızı (gaz kelebeği için)
// steering_angle: -1.0 (tam sol) ile 1.0 (tam sağ) arası (diferansiyel sürüş veya direksiyon servo için)
void set_motor_speed_and_direction(int speed_percentage, float steering_angle);

// Fren kontrol fonksiyonları
void apply_brake(void);
void release_brake(void);

// Lazer kontrol fonksiyonları
// pan_angle, tilt_angle: Lazerin yönelim açıları (derece cinsinden)
void set_lazer_yonu(float pan_angle, float tilt_angle);
void activate_laser(void);
void deactivate_laser(void);

// Sensör okuma fonksiyonları
void read_imu_data(IMU_Data_t* imu_data);
void read_encoder_data(Encoder_Data_t* encoder_data);
float calculate_vehicle_speed(Encoder_Data_t* encoder_data);
float calculate_distance_traveled_since_reset(void); // Tabela algılaması sonrası mesafe için

// --- Donanım Pinleri ve Kontrolcü Tanımları ---
// Bu tanımlar, projenizin GPIO ayarlarıyla eşleşmelidir.
// MOTOR_LEFT/RIGHT_PWM_PIN tanımları, eğer traktör motoru yerine doğrudan elektrik motorları sürülecekse geçerlidir.
// Traktör motoru için ana PWM gaz kelebeği içindir.

// Elektrik Motoru Sürücüleri için (Eğer traktör motorunun yerine bunlar kullanılacaksa)
#define MOTOR_LEFT_PWM_PORT GPIOA
#define MOTOR_LEFT_PWM_PIN  GPIO_PIN_0 // Örnek: TIM1_CH1
#define MOTOR_RIGHT_PWM_PORT GPIOA
#define MOTOR_RIGHT_PWM_PIN GPIO_PIN_1 // Örnek: TIM1_CH2

// Gaz Kelebeği Servo Pini (Traktör motoru için)
#define GAS_SERVO_PWM_PORT GPIOB
#define GAS_SERVO_PWM_PIN  GPIO_PIN_6 // Örnek: TIM4_CH1

// Fren Aktüatörü Pini (Röle veya MOSFET ile kontrol)
#define BRAKE_RELAY_PORT GPIOC
#define BRAKE_RELAY_PIN  GPIO_PIN_13

// Lazer Pan/Tilt Servo Pinleri
#define LASER_PAN_SERVO_PWM_PORT GPIOC
#define LASER_PAN_SERVO_PWM_PIN  GPIO_PIN_8 // Örnek: TIM3_CH3
#define LASER_TILT_SERVO_PWM_PORT GPIOC
#define LASER_TILT_SERVO_PWM_PIN GPIO_PIN_9 // Örnek: TIM3_CH4

// Lazer Aktif Etme Pini (Lazer güç kontrolü)
#define LASER_ENABLE_PORT GPIOD
#define LASER_ENABLE_PIN GPIO_PIN_2

// Acil Durdurma Butonu Pini (Güvenlik alt sistemi için, kesmeli giriş olmalı)
#define EMERGENCY_STOP_BUTTON_PORT GPIOA
#define EMERGENCY_STOP_BUTTON_PIN  GPIO_PIN_15 // Örnek: Harici kesme (EXTI)

#endif // ROBOT_CONTROL_H