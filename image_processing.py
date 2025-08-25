# image_processing.py

import cv2
import numpy as np
# from pytesseract import pytesseract # Tesseract OCR kütüphanesi için
# Tesseract kurulumu: sudo apt-get install tesseract-ocr libtesseract-dev
# Python kütüphanesi: pip install pytesseract opencv-python

from logger import logger # Loglama modülünü içe aktarıyoruz

# Görüntü işleme için sabitler (Bunlar gerçek testlerle kalibre edilecek değerlerdir!)
# HSV renk uzayı eşikleri (örnek değerler, sahada ayarlanmalı)
LOWER_RED1 = np.array([0, 70, 50])
UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([170, 70, 50])
UPPER_RED2 = np.array([180, 255, 255])

LOWER_WHITE = np.array([0, 0, 200])
UPPER_WHITE = np.array([180, 30, 255])

LOWER_ORANGE = np.array([5, 100, 100])
UPPER_ORANGE = np.array([20, 255, 255])

# Tabela ve hedef boyutları (piksel cinsinden, kamera çözünürlüğüne ve mesafeye göre değişir)
MIN_LABEL_RADIUS_PX = 50
MAX_LABEL_RADIUS_PX = 150
AIM_TARGET_MIN_RADIUS_PX = 10
AIM_TARGET_MAX_RADIUS_PX = 60

def preprocess_image(image):
    """Görüntüyü gürültü azaltma ve gri tonlamaya çevirme gibi ön işlemlerden geçirir."""
    if image is None:
        logger.log_error("Ön işleme için boş görüntü alındı.")
        return None, None
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    return blurred, image # Orijinal renkli görüntüyü de döndürürüz

def detect_parkour_label(image):
    """
    Parkur tabelalarını (Şekil 3'teki gibi, 60cm çapında, Arial font) algılar.
    Bu fonksiyon, metin algılama (OCR) ve şekil tanıma kombinasyonu kullanır.
    """
    processed_image, original_color_image = preprocess_image(image)
    if processed_image is None:
        return None, None
    
    circles = cv2.HoughCircles(processed_image, cv2.HOUGH_GRADIENT, dp=1, minDist=100,
                               param1=100, param2=30, minRadius=MIN_LABEL_RADIUS_PX, maxRadius=MAX_LABEL_RADIUS_PX)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            x, y, r = i[0], i[1], i[2]
            
            roi = original_color_image[max(0, y-r):min(image.shape[0], y+r), 
                                       max(0, x-r):min(image.shape[1], x+r)]
            
            if roi.shape[0] == 0 or roi.shape[1] == 0:
                continue 
            
            try:
                # Gerçek kullanımda burayı aktif et:
                # text_from_label = pytesseract.image_to_string(roi, config=config).strip()
                # text_from_label = text_from_label.upper().replace(" ", "_").replace("\n", "")
                
                # Şimdilik örnek olarak, manuel bir eşleştirme veya basit bir placeholder döndürelim:
                # return "DIK_ENGEL_1", (image.shape[1]//2, image.shape[0]//2) # Test amaçlı
                
                pass 

            except Exception as e:
                logger.log_error(f"Tabela OCR veya işleme hatası: {e}")
    
    return None, None 

def find_path_center(image):
    """
    Yol bariyerlerini (kırmızı ve beyaz, 80cm yükseklikte) algılayarak yolun orta noktasından sapmayı hesaplar.
    """
    if image is None:
        return 0
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    mask_red1 = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
    mask_red2 = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    mask_white = cv2.inRange(hsv, LOWER_WHITE, UPPER_WHITE)

    combined_mask = cv2.bitwise_or(mask_red, mask_white)
    
    kernel = np.ones((5,5),np.uint8)
    combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
    combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            image_center_x = image.shape[1] / 2
            offset_from_center = cx - image_center_x
            logger.log_debug(f"Yol ortası sapması: {offset_from_center:.2f}")
            return offset_from_center
    logger.log_debug("Yol bariyeri bulunamadı, sapma 0.")
    return 0 

def detect_aim_target(image_aim):
    """
    Atış hedefi olan iç içe daireleri nişan kamerasından algılar.
    """
    processed_image, original_color_image = preprocess_image(image_aim)
    if processed_image is None:
        return None
    
    circles = cv2.HoughCircles(processed_image, cv2.HOUGH_GRADIENT, dp=1.2, minDist=20,
                               param1=100, param2=50, minRadius=AIM_TARGET_MIN_RADIUS_PX, maxRadius=AIM_TARGET_MAX_RADIUS_PX)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        target_circle = None
        min_radius = float('inf')
        for i in circles[0, :]:
            x, y, r = i[0], i[1], i[2]
            # Şartnamede 6, 12, 18 cm çaplar var, piksel karşılıklarını tahmin etmeliyiz.
            if r < min_radius and r > AIM_TARGET_MIN_RADIUS_PX: 
                min_radius = r
                target_circle = (x, y, r)
        
        if target_circle:
            logger.log_info(f"Atış hedefi algılandı: Merkez ({target_circle[0]}, {target_circle[1]})")
            return (target_circle[0], target_circle[1]) 
    logger.log_debug("Atış hedefi bulunamadı.")
    return None

def detect_cones(image):
    """
    Trafik konilerini (kırmızı/turuncu beyaz, 75cm yükseklikte) algılar.
    """
    if image is None:
        logger.log_error("Konileri algılamak için boş görüntü alındı.")
        return []

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    mask_orange = cv2.inRange(hsv, LOWER_ORANGE, UPPER_ORANGE)
    mask_red = cv2.bitwise_or(cv2.inRange(hsv, LOWER_RED1, UPPER_RED1),
                              cv2.inRange(hsv, LOWER_RED2, UPPER_RED2))
    
    mask_white = cv2.inRange(hsv, LOWER_WHITE, UPPER_WHITE)
    
    mask_cone_colors = cv2.bitwise_or(cv2.bitwise_or(mask_orange, mask_red), mask_white)
    
    kernel = np.ones((7,7),np.uint8)
    mask_cone_colors = cv2.morphologyEx(mask_cone_colors, cv2.MORPH_OPEN, kernel) 
    mask_cone_colors = cv2.morphologyEx(mask_cone_colors, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask_cone_colors, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    cones = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 200: 
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w)/h
            if 0.3 < aspect_ratio < 0.7 and h > 50: 
                cones.append({'x': x, 'y': y, 'w': w, 'h': h, 'center_x': x + w/2, 'center_y': y + h/2}) 
                logger.log_debug(f"Koni algılandı: Konum=({x},{y}), Boyut=({w}x{h}), Alan={area}")
    return cones

def detect_bumps(image):
    """
    Engebeli arazideki tümsekleri (5cm yükseklikte) algılar.
    Bu daha çok derinlik algılama veya gölge analiziyle yapılabilir.
    Tek kamera ile zor olabilir, ancak yolun dokusundaki ani değişiklikler kullanılabilir.
    """
    if image is None:
        return []
    processed_image, _ = preprocess_image(image)
    edges = cv2.Canny(processed_image, 50, 150)
    
    logger.log_debug("Tümsek algılama placeholder'ı çalışıyor.")
    return [] 

def detect_finish_line(image):
    """
    Bitiş çizgisini algılar. Şartnamede detay verilmemiş, muhtemelen beyaz bir çizgi.
    """
    if image is None:
        return False
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 30, 255])
    mask_white = cv2.inRange(hsv, lower_white, upper_white)
    
    edges = cv2.Canny(mask_white, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=100, minLineLength=int(image.shape[1]*0.8), maxLineGap=10)

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if abs(y1 - y2) < 10 and y1 > image.shape[0] / 2:
                logger.log_info("Bitiş çizgisi algılandı.")
                return True
    return False

def check_for_crossed_line_over_4(roi_image_of_4):
    """
    '4' rakamının üzerinde çapraz bir çizgi olup olmadığını kontrol eder.
    Bu, 'üzeri çizili 4' tabelasını ayırt etmek için kritik olacaktır.
    """
    if roi_image_of_4 is None:
        return False
    
    logger.log_debug("check_for_crossed_line_over_4 fonksiyonu placeholder'ı çalışıyor.")
    return False