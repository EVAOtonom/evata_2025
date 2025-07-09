import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import Int8 
import os
import numpy as np
import time
import torch
import sys
from ament_index_python.packages import get_package_share_directory

# utils modülünden gerekli fonksiyonları import et
try:
    # DOĞRU İMPORT: utils klasörü içindeki utils.py modülünden import et
    from evata_sim.utils.utils import \
        time_synchronized, select_device, increment_path, \
        scale_coords, xyxy2xywh, non_max_suppression, split_for_trace_model, \
        driving_area_mask, lane_line_mask, plot_one_box, show_seg_result
    # Opsiyonel olarak letterbox'ı da buradan import edebiliriz, kodda ayrıca tanımlı ama:
    # from evata_sim.utils.utils import letterbox
    print("utils fonksiyonları 'evata_sim.utils.utils' üzerinden import edildi.")
    UTILS_AVAILABLE = True
except ImportError as e:
    print(f"HATA: 'evata_sim.utils.utils' import edilemedi: {e}")
    # ... (hata mesajları) ...
    UTILS_AVAILABLE = False
# ... (except blokları) ...

# letterbox fonksiyonu (utils.py içinde de var, ama orijinal kodda da vardı, burada kalsın)
# Alternatif olarak utils'den import edilebilir: from evata_sim.utils import letterbox
def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    shape = img.shape[:2]
    if isinstance(new_shape, int): new_shape = (new_shape, new_shape)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup: r = min(r, 1.0)
    ratio = r, r
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    if auto: dw, dh = np.mod(dw, stride), np.mod(dh, stride)
    elif scaleFill: dw, dh = 0.0, 0.0; new_unpad = (new_shape[1], new_shape[0]); ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]
    dw /= 2; dh /= 2
    if shape[::-1] != new_unpad: img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return img, ratio, (dw, dh)


class ZedLaneFollower(Node):
    def __init__(self):
        # Düğüm adını daha açıklayıcı yapalım
        super().__init__('zed_lane_follower_node')
        self.bridge = CvBridge()
        self.model = None
        self.device = None
        self.half = False
        self.imgsz = 640
        self.stride = 32
        self.package_name = 'evata_sim'

        # Takip değişkenleri (orijinal koddan)
        self.image_center_x = 640  # 1280 genişlik varsayımıyla
        self.current_steering = 0.0
        self.current_lane = "sag" # Başlangıç değeri veya None olabilir
        self.last_valid_lane = "sag" # Son geçerli şeridi tutmak için

        # --- Model Yükleme ---
        if not UTILS_AVAILABLE:
            self.get_logger().error("Gerekli utils fonksiyonları yüklenemediği için model yüklenemiyor.")
            return

        try:
            package_share_directory = get_package_share_directory(self.package_name)
            weights_filename = 'yolopv2.pt'
            weights_path = os.path.join(package_share_directory, 'utils', weights_filename)

            self.get_logger().info(f"Model ağırlık dosyası aranıyor: {weights_path}")

            if not os.path.isfile(weights_path):
                self.get_logger().error(f"Model ağırlık dosyası bulunamadı: {weights_path}")
                self.log_setup_error()
                return

            self.get_logger().info("YOLOPv2 modeli yükleniyor...")
            self.device = select_device('0' if torch.cuda.is_available() else 'cpu')
            self.half = self.device.type != 'cpu'

            self.model = torch.jit.load(weights_path, map_location=self.device)
            if self.half:
                self.model.half()
            self.model.eval()

            if self.device.type != 'cpu':
                self.warmup_model()
            self.get_logger().info(f"Model başarıyla yüklendi ve {self.device} cihazına atandı.")

        except Exception as e:
            self.get_logger().error(f"Model yüklenirken HATA oluştu: {e}")
            self.model = None
            return

        # --- ROS Abonelik ve Yayıncılar ---
        # ZED M kamerası için doğru topic adı
        self.zed_image_topic = '/zed/zed_node/right/image_rect_color'
        self.subscription = self.create_subscription(
            Image,
            self.zed_image_topic,
            self.image_callback,
            10) # QoS
        self.get_logger().info(f"'{self.zed_image_topic}' topic'ine abone olundu.")

        # Kontrol komutları için publisher (orijinal koddan)
        #self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        #self.get_logger().info("'/cmd_vel' topic'ine yayıncı oluşturuldu.")
        self.steering_publisher = self.create_publisher(Int8, "/stm/steering_angle", 10)
        self.get_logger().info("'/stm/steering_angle' (Int8) topic'ine yayıncı oluşturuldu.")

        self.get_logger().info("Zed Lane Follower düğümü başlatıldı.")

    def log_setup_error(self):
         """ Kurulum hatası durumunda yardımcı log mesajları yazdırır. """
         self.get_logger().error("--- KURULUM HATASI KONTROLÜ ---")
         self.get_logger().error("1. 'setup.py' dosyasında 'data_files' listesinde şu satır var mı?")
         self.get_logger().error("   (os.path.join('share', package_name, 'utils'), glob('evata_sim/utils/*.pt')),")
         self.get_logger().error("2. Kaynak kodda 'evata_sim/utils/yolopv2.pt' dosyası mevcut mu?")
         self.get_logger().error("3. 'setup.py' ve bu Python dosyasını kaydettikten sonra 'colcon build --packages-select evata_sim' komutunu çalıştırdınız mı?")
         self.get_logger().error("---------------------------------")

    def warmup_model(self):
        """ Modeli GPU üzerinde ısıtır. """
        self.get_logger().info("Model warmup yapılıyor...")
        try:
            dummy_input = torch.zeros(1, 3, self.imgsz, self.imgsz).to(self.device)
            dummy_input = dummy_input.half() if self.half else dummy_input
            # Çalıştığından emin olmak için modeli çağır
            with torch.no_grad():
                _ = self.model(dummy_input)
            self.get_logger().info("Warmup tamamlandı.")
        except Exception as e:
             self.get_logger().error(f"Warmup sırasında hata: {e}")

    def calculate_steering_angle(self, mid_points):
        """ Orta noktaların x koordinatlarının ortalamasına göre dönme açısını hesapla. (Orijinal koddan) """
        if not mid_points:
            self.current_steering = 0.0 # Orta nokta yoksa direksiyonu sıfırla
            print("not mid point")
            return 0.0

        x_coords = [point[0] for point in mid_points]
        if not x_coords : return 0.0 # Eğer x_coords boşsa

        avg_x = sum(x_coords) / len(x_coords)
        deviation = avg_x - self.image_center_x
        max_deviation = self.image_center_x
        # Kırpmayı önlemek için oranı -1 ve 1 arasında sınırla
        steering_ratio = np.clip(deviation / max_deviation, -1.0, 1.0)
        self.current_steering = steering_ratio # Direksiyon durumunu güncelle
        # Kontrol için tersini döndür (orijinal mantık)
        return -steering_ratio 

    def get_dynamic_roi_bounds(self, y, img_width=1280):
        """ Steering açısına göre dinamik ROI sınırlarını hesaplar. (Orijinal koddan) """
        roi_y_start = 520
        roi_y_end = 720
        # Statik ROI sınırları
        static_x1 = int(240 + (y - roi_y_start) * (0 - 240) / (roi_y_end - roi_y_start))
        static_x2 = int(1100 + (y - roi_y_start) * (1280 - 1100) / (roi_y_end - roi_y_start))

        # Dinamik kayma (katsayı ayarlanabilir)
        shift = int(self.current_steering * 150)
        dynamic_x1 = static_x1 + shift
        dynamic_x2 = static_x2 + shift

        # Sınırları görüntü genişliği içinde tut
        return (max(0, dynamic_x1), min(img_width, dynamic_x2))

    def publish_steering_angle(self, steering_ratio):
        """ Hesaplanan direksiyon oranını Int8 olarak yayınlar. """
        # steering_ratio: -1.0 (tam sol) ile 1.0 (tam sağ) arasında bir değer olmalı
        
        # Oranı 40 ile çarp ve tam sayıya çevir
        steering_value_raw = steering_ratio * -120.0
        
        
        # Int8 aralığı: -128 ile 127 arası. Değeri bu aralığa kırp.
        # Önemli: STM32'nin tam olarak hangi değer aralığını beklediğini bilmek iyi olur.
        # Şimdilik Int8'in tam aralığını kullanıyoruz.
        steering_value_int = int(round(steering_value_raw))
        steering_value_clamped = np.clip(steering_value_int, -128, 127)
        
        msg = Int8()
        msg.data = int(steering_value_clamped) # int() ile tekrar Python int'e çevir

        self.steering_publisher.publish(msg)
        self.get_logger().info(f"Publishing to /stm/steering_angle: {msg.data} (raw: {steering_value_raw:.2f})", throttle_duration_sec=0.5)
            # Loglamayı azaltmak için her zaman yazdırma, belki periyodik olarak?
            # self.get_logger().info(f"Publishing cmd_vel: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}", throttle_duration_sec=1.0)


    def image_callback(self, msg):
        """ Gelen görüntüyü alır ve şerit takip mantığını çalıştırır. """
        if self.model is None or not UTILS_AVAILABLE:
            return

        try:
            # Görüntüyü OpenCV formatına çevir
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Ana işleme fonksiyonunu çağır
            self.process_frame(cv_image)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge hatası: {e}")
        except Exception as e:
            self.get_logger().error(f'Görüntü işlenirken beklenmeyen hata: {e}', exc_info=True) # Traceback'i görmek için


    def process_frame(self, source_img, imgsz=640, conf_thres=0.3, iou_thres=0.45):
        """ Görüntüyü işler, şeritleri bulur, direksiyonu hesaplar ve görselleştirir. (Orijinal 'detect' fonksiyonu temel alınmıştır) """
        t_start_process = time.time() # İşleme süresini ölçmek için başlangıç

        # --- Görüntü Ön İşleme ---
        target_width, target_height = 1280, 720
        try:
            # Görüntüyü modele vermeden önce hedef boyuta getir
            img_resized_for_vis = cv2.resize(source_img, (target_width, target_height), interpolation=cv2.INTER_LINEAR)
        except Exception as e:
            self.get_logger().warn(f"Giriş görüntüsü yeniden boyutlandırılamadı: {e}. Orijinal boyut kullanılıyor.")
            img_resized_for_vis = source_img
            target_width, target_height = source_img.shape[1], source_img.shape[0]
            self.image_center_x = target_width // 2 # Görüntü merkezini güncelle

        # Modle girmeden önce letterbox uygula
        img_letterboxed, _, _ = letterbox(img_resized_for_vis, imgsz, stride=self.stride)
        # BGR->RGB, HWC->CHW
        img_tensor_input = img_letterboxed[:, :, ::-1].transpose(2, 0, 1)
        img_tensor_input = np.ascontiguousarray(img_tensor_input)

        # --- PyTorch Tensor Hazırlığı ---
        img_tensor = torch.from_numpy(img_tensor_input).to(self.device)
        img_tensor = img_tensor.half() if self.half else img_tensor.float()
        img_tensor /= 255.0
        if img_tensor.ndimension() == 3:
            img_tensor = img_tensor.unsqueeze(0)

        # --- Model Çıkarımı ---
        ll_seg_mask = None
        da_seg_mask = None
        det_pred = None # Nesne tespiti sonucu
        with torch.no_grad():
            try:
                t1 = time_synchronized()
                # Modelin tam çıktısını al (nesne, sürüş alanı, şerit)
                [pred_raw, anchor_grid], seg_raw, ll_raw = self.model(img_tensor)
                t2 = time_synchronized()
                # self.get_logger().debug(f"Inference time: {(t2 - t1)*1000:.1f} ms")

                # Çıktıları işle
                det_pred = split_for_trace_model(pred_raw, anchor_grid) # Nesne tespiti için ön işleme
                det_pred = non_max_suppression(det_pred, conf_thres, iou_thres) # NMS uygula

                # Maskeleri oluştur (utils fonksiyonları ile)
                # Not: Bu fonksiyonların çıktısının (1280, 720) boyutunda olduğunu varsayıyoruz
                da_seg_mask = driving_area_mask(seg_raw)
                ll_seg_mask = lane_line_mask(ll_raw)

            except Exception as e:
                self.get_logger().error(f"Model çıkarımı veya çıktı işleme hatası: {e}", throttle_duration_sec=5)
                # Hata durumunda görselleştirmeyi atla veya sadece orijinal görüntüyü göster
                cv2.imshow("Lane Following (ZED)", img_resized_for_vis)
                cv2.waitKey(1)
                return # Fonksiyondan çık

        # --- Şerit Takip Mantığı ---
        # Görselleştirme için görüntünün kopyasını al (1280x720)
        im0s = img_resized_for_vis.copy()

        if ll_seg_mask is None:
            self.get_logger().warn("Şerit maskesi alınamadı, takip yapılamıyor.", throttle_duration_sec=5)
            # Maske yoksa direksiyonu sıfırla
            self.current_steering = 0.0
            self.publish_steering_angle(0.0) # <<<--- DEĞİŞTİ
            # Sadece orijinal görüntüyü göster
            cv2.imshow("Lane Following (ZED)", im0s) # im0s, yeniden boyutlandırılmış görüntü
            cv2.waitKey(1)
            return

        # Şerit piksellerinin koordinatlarını al
        y_coords, x_coords = np.where(ll_seg_mask == 1)

        # --- ROI ve Orta Nokta Hesaplamaları (Orijinal koddan) ---
        roi_y_start = 520
        roi_y_end = 720
        def get_static_roi_x_bounds(y): # İç fonksiyon olarak tanımla
            if y < roi_y_start or y > roi_y_end: return None, None
            x1 = int(240 + (y - roi_y_start) * (0- 240) / (roi_y_end - roi_y_start))
            x2 = int(1100 + (y - roi_y_start) * (1280 - 1100) / (roi_y_end - roi_y_start))
            return max(0, x1), min(target_width, x2) # Görüntü sınırları içinde tut

        # Ana ROI içindeki pikselleri filtrele
        roi_mask_filter = (y_coords >= roi_y_start) & (y_coords <= roi_y_end)
        roi_y_coords = y_coords[roi_mask_filter]
        roi_x_coords = x_coords[roi_mask_filter]

        mid_points = []
        fallback_points = []

        for y in range(roi_y_start, roi_y_end + 1): # Her y seviyesi için
            x1_static, x2_static = get_static_roi_x_bounds(y)
            if x1_static is None: continue

            x1_dynamic, x2_dynamic = self.get_dynamic_roi_bounds(y, target_width)
            x1_combined = min(x1_static, x1_dynamic)
            x2_combined = max(x2_static, x2_dynamic)

            y_mask = roi_y_coords == y
            x_values_in_y = roi_x_coords[y_mask & (roi_x_coords >= x1_combined) & (roi_x_coords <= x2_combined)]

            if len(x_values_in_y) >= 2:
                x_values_sorted = np.sort(x_values_in_y)
                x_diff = np.diff(x_values_sorted)
                valid_pairs_idx = np.where(x_diff >= 500)[0] # Minimum fark
                if len(valid_pairs_idx) > 0:
                    idx = valid_pairs_idx[0]
                    x_left = x_values_sorted[idx]
                    x_right = x_values_sorted[idx + 1]
                    mid_x = (x_left + x_right) // 2
                    mid_points.append((mid_x, y))
                    # print(f"MidPoint: y={y}, x1={x_left}, x2={x_right}, mid={mid_x}") # Debug
                elif len(x_values_in_y) > 0: # Fallback
                    potential_right_x = np.max(x_values_in_y)
                    adjusted_x = potential_right_x - 335 # Sabit offset
                    adjusted_x = max(x1_combined, min(x2_combined, adjusted_x))
                    if adjusted_x > x1_combined + 10:
                        fallback_points.append((adjusted_x, y))

        if not mid_points and fallback_points:
            # self.get_logger().info("Fallback noktaları kullanılıyor.", throttle_duration_sec=2.0)
            mid_points = fallback_points

        # --- Şerit Tespiti (Genişletilmiş ROI - Orijinal koddan) ---
        ext_roi_y1, ext_roi_y2 = 518, 520
        ext_roi_x1, ext_roi_x2 = 100, target_width - 100

        roi_mask_ext = (y_coords >= ext_roi_y1) & (y_coords <= ext_roi_y2) & \
                       (x_coords >= ext_roi_x1) & (x_coords <= ext_roi_x2)
        x_vals_ext = x_coords[roi_mask_ext]
        current_detected_lane = "None"

        if len(x_vals_ext) > 10:
            x_vals_ext_sorted = np.sort(np.unique(x_vals_ext))
            x_diffs_ext = np.diff(x_vals_ext_sorted)
            threshold = 800
            lines_idx = np.where(x_diffs_ext > threshold)[0]
            line_positions = []
            if len(lines_idx) >= 2:  # En az 2 çizgi aralığı (3 çizgi)
            # Çizgi pozisyonlarını al ve sırala
                lines = []
                for i in range(min(3, len(lines_idx)+1)):  # Maksimum 3 çizgi
                    lines.append(x_vals_ext_sorted[lines_idx[i]] if i < len(lines_idx) else x_vals_ext_sorted[-1])
                line_positions = sorted(lines)

            if len(line_positions) >= 2:
                car_x = int(np.mean([pt[0] for pt in mid_points])) if mid_points else self.image_center_x
                if len(line_positions) >= 3:
                     l1, l2, l3 = sorted(line_positions[:3])
                     # Orijinal mantıkta cok_sol yoktu, ekleyelim
                     if car_x < l1: current_detected_lane = "cok_sol"
                     elif car_x < (l1 + l2) / 2: current_detected_lane = "sol"
                     elif car_x < (l2 + l3) / 2: current_detected_lane = "sag"
                     else: current_detected_lane = "cok_sag"
                else: # 2 çizgi
                    l1, l2 = sorted(line_positions[:2])
                    lane_center = (l1 + l2) / 2
                    if car_x < lane_center: current_detected_lane = "sol"
                    else: current_detected_lane = "sag"

        # Şerit state'ini güncelle
        if current_detected_lane != "None":
            self.last_valid_lane = current_detected_lane
        # Her zaman son geçerli olanı kullan (veya başlangıç değerini)
        self.current_lane = self.last_valid_lane if self.last_valid_lane else "sag"


        # --- Direksiyon Hesaplama ve Kontrol ---
        steering_ratio_control = 0.0 # Bu, -1.0 ile 1.0 arası bir orandır
        if len(mid_points) >= 2: # En az iki orta nokta varsa takip yap
            steering_ratio_control = self.calculate_steering_angle(mid_points)
            # Şimdi yeni fonksiyonumuzu çağırıyoruz (bu oran ile)
            self.publish_steering_angle(steering_ratio_control) # <<<--- DEĞİŞTİ 1
        else:
            # Orta nokta yoksa direksiyonu sıfırla
            self.get_logger().warn("Takip için yeterli orta nokta bulunamadı.", throttle_duration_sec=2.0)
            self.current_steering = 0.0 # Direksiyon durumunu sıfırla
            # Yeni fonksiyonumuzu çağırıyoruz (0.0 oranı ile)
            self.publish_steering_angle(0.0) # <<<--- DEĞİŞTİ 2


        # --- Görselleştirme ---
        # Statik ROI (Mavi)
        # Not: get_static_roi_x_bounds'un None dönme ihtimalini ele al
        sr1, sr2 = get_static_roi_x_bounds(roi_y_start), get_static_roi_x_bounds(roi_y_end)
        if sr1 and sr2 and sr1[0] is not None and sr2[0] is not None:
             roi_pts_s = np.array([(sr1[0],roi_y_start), (sr1[1],roi_y_start), (sr2[1],roi_y_end), (sr2[0],roi_y_end)], np.int32)
             cv2.polylines(im0s, [roi_pts_s.reshape((-1, 1, 2))], isClosed=True, color=(255, 0, 0), thickness=2)

        # Dinamik ROI (Kırmızı)
        dr1, dr2 = self.get_dynamic_roi_bounds(roi_y_start, target_width), self.get_dynamic_roi_bounds(roi_y_end, target_width)
        roi_pts_d = np.array([(dr1[0],roi_y_start), (dr1[1],roi_y_start), (dr2[1],roi_y_end), (dr2[0],roi_y_end)], np.int32)
        cv2.polylines(im0s, [roi_pts_d.reshape((-1, 1, 2))], isClosed=True, color=(0, 0, 255), thickness=2)

        # Genişletilmiş ROI (Turuncu)
        cv2.rectangle(im0s, (ext_roi_x1, ext_roi_y1), (ext_roi_x2, ext_roi_y2), (0, 165, 255), 2)

        # Tespit Edilen Şerit Yazısı
        cv2.putText(im0s, f"Serit: {self.current_lane}", (30, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.8, (255, 0, 131), 4, cv2.LINE_AA)


        if len(mid_points) >= 2:
            pts = np.array(mid_points, np.int32).reshape((-1, 1, 2))
            cv2.polylines(im0s, [pts], isClosed=False, color=(0, 255, 0), thickness=3)
        # for point in mid_points: # Noktaları da çizmek isterseniz
        #     cv2.circle(im0s, point, radius=3, color=(0, 0, 255), thickness=-1)

        # Nesne Tespiti Sonuçları (Eğer varsa ve utils.plot_one_box varsa)
        if det_pred is not None and UTILS_AVAILABLE:
            for i, det in enumerate(det_pred): # Her görüntü için (bizde tek görüntü var)
                if len(det):
                    # Kutuları model boyutundan orijinal görüntü boyutuna ölçekle
                    det[:, :4] = scale_coords(img_tensor.shape[2:], det[:, :4], img_resized_for_vis.shape).round()
                    # Sonuçları yazdır
                    for *xyxy, conf, cls in reversed(det):
                         plot_one_box(xyxy, im0s, line_thickness=2) # Kalınlığı ayarlayabilirsiniz

        # Segmentasyon Sonuçlarını Göster (Sürüş Alanı + Şerit)
        if da_seg_mask is not None and ll_seg_mask is not None and UTILS_AVAILABLE:
            try:
                 # show_seg_result fonksiyonunun (1280, 720) boyutunda maske beklediğini varsayalım
                 if da_seg_mask.shape[0] != target_height or da_seg_mask.shape[1] != target_width:
                      da_seg_mask = cv2.resize(da_seg_mask.astype(np.uint8), (target_width, target_height), interpolation=cv2.INTER_NEAREST)
                 if ll_seg_mask.shape[0] != target_height or ll_seg_mask.shape[1] != target_width:
                      ll_seg_mask = cv2.resize(ll_seg_mask.astype(np.uint8), (target_width, target_height), interpolation=cv2.INTER_NEAREST)
                 # im0s üzerine çizim yapar
                 show_seg_result(im0s, (da_seg_mask, ll_seg_mask), is_demo=True)
                 print("hasan")
            except Exception as e:
                 self.get_logger().error(f"'show_seg_result' hatası: {e}", throttle_duration_sec=5)
                 # Alternatif: Sadece şeritleri overlay yap
                 overlay = np.zeros_like(im0s, dtype=np.uint8)
                 overlay[ll_seg_mask == 1] = (255, 0, 0) # Mavi yapalım bu sefer
                 im0s = cv2.addWeighted(im0s, 1, overlay, 0.5, 0)

        # FPS Ekle
        t_end_process = time.time()
        fps = 1.0 / (t_end_process - t_start_process) if (t_end_process - t_start_process) > 0 else 0
        cv2.putText(im0s, f"FPS: {fps:.1f}", (im0s.shape[1] - 150, 40),
                         cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Sonucu Göster
        cv2.imshow("Lane Following (ZED)", im0s)
        cv2.waitKey(1)


def main(args=None):
    print("Zed Lane Follower başlatılıyor...")
    rclpy.init(args=args)
    # Sınıf adını değiştirdiğimiz için burada da değiştirelim
    zed_lane_follower_node = ZedLaneFollower()

    if zed_lane_follower_node.model is not None and UTILS_AVAILABLE:
        print("Düğüm spin'e giriyor. Kapatmak için Ctrl+C.")
        try:
            rclpy.spin(zed_lane_follower_node)
        except KeyboardInterrupt:
            print("KeyboardInterrupt algılandı.")
        except Exception as e:
             zed_lane_follower_node.get_logger().fatal(f"Spin sırasında beklenmeyen HATA: {e}", exc_info=True)
        finally:
            print("Temizlik yapılıyor...")
            cv2.destroyAllWindows()
            if rclpy.ok():
                 zed_lane_follower_node.destroy_node()
                 rclpy.shutdown()
            print("Düğüm kapatıldı.")
    else:
        print("HATA: Model veya utils yüklenemediği için düğüm spin'e giremiyor.")
        if rclpy.ok():
             zed_lane_follower_node.destroy_node()
             rclpy.shutdown()


if __name__ == '__main__':
    main()