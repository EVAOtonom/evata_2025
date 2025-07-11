import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Int8 
import os
import numpy as np
import time
import torch
from evata_sim.utils.utils import \
    time_synchronized, select_device, increment_path, \
    scale_coords, xyxy2xywh, non_max_suppression, split_for_trace_model, \
    driving_area_mask, lane_line_mask, plot_one_box, show_seg_result, \
    AverageMeter, LoadImages


def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    # Resize and pad image while meeting stride-multiple constraints
    shape = img.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

    if not scaleup:  # only scale down, do not scale up (for better test mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
     
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))

    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    
    return img, ratio, (dw, dh)

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.image_callback,
            10)
        
        self.bridge = CvBridge()
        dir_path = os.path.dirname(os.path.realpath(__file__))
        src_dir = dir_path.split('/install')[0]  # install kısmını çıkar
        weights = os.path.join(src_dir, 'src', 'evata_sim', 'evata_sim', 'utils', 'yolopv2.pt')
        device = "0"
        model = torch.jit.load(weights)
        device = select_device(device)
        half = device.type != 'cpu'  # half precision only supported on CUDA
        model = model.to(device)

        if half:
            model.half()  # to FP16  
        model.eval()
        self.model = model
        self.device = device


        self.cmd_vel_publisher = self.create_publisher(Int8, "/stm/steering_angle", 10)
        self.image_center_x = 640  # Görselin orta noktası (1280x720 için)
        self.current_steering = 0.0  # Dinamik ROI için eklendi
        self.serit=None
        self.current_lane = None

    def calculate_steering_angle(self, mid_points):
        """Orta noktaların x koordinatlarının ortalamasına göre dönme açısını hesapla."""
        if not mid_points:
            return 0.0  # Orta nokta yoksa düz git

        # Tüm orta noktaların x koordinatlarını al
        x_coords = [point[0] for point in mid_points]

        # X koordinatlarının ortalamasını hesapla
        avg_x = sum(x_coords) / len(x_coords)

        # Görselin orta noktasına göre sapma miktarını hesapla
        deviation = avg_x - self.image_center_x

        # Sapma miktarını normalize et (örneğin, -1.0 ile 1.0 arasında)
        max_deviation = self.image_center_x  # Maksimum sapma (640 piksel)
        steering_angle = deviation / max_deviation  # -1.0 (sola) ile 1.0 (sağa) arasında
        self.current_steering = steering_angle  # Dinamik ROI için kaydet
        return steering_angle *120

    def get_dynamic_roi_bounds(self, y):
        """Steering açısına göre dinamik ROI sınırlarını hesaplar"""
        # Statik ROI hesaplaması (orijinal kodunuzdaki gibi)
        roi_y_start = 520
        roi_y_end = 720
        static_x1 = int(240 + (y - roi_y_start) * (0 - 240) / (roi_y_end - roi_y_start))
        static_x2 = int(1100 + (y - roi_y_start) * (1280 - 1100) / (roi_y_end - roi_y_start))
        
        # Dinamik kayma miktarı (300 katsayısı)
        shift = int(self.current_steering * 500)
        return (max(0, static_x1 + shift), min(1280, static_x2 + shift))

    def publish_cmd_vel(self, steering_angle):
        """Tekerlek açısını Int8 olarak yayınlar."""
        msg = Int8()

        # steering_angle zaten calculate_steering_angle içinde
        # (oran * 120) olarak hesaplanmıştı.
        # Şimdi bunu Int8 sınırlarına (-128 ile 127) kırpalım.
        steering_value_raw = steering_angle # Bu zaten çarpılmış değer
        steering_value_int = int(round(steering_value_raw))
        
        
        msg.data = steering_value_int# Python int'e çevir

        self.cmd_vel_publisher.publish(msg) # Publisher adı orijinaldeki gibi cmd_vel_publisher
        self.get_logger().info(f"Publishing to /stm/steering_angle: {msg.data} (Calculated: {steering_value_raw:.2f})", throttle_duration_sec=0.5)

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.detect(self.latest_image)
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')


    def detect(self, source, imgsz=640, conf_thres=0.3, iou_thres=0.45, 
               device='0', classes=None, agnostic_nms=False,):
        stride = 32
        model = self.model
        device = self.device
        half = device.type != 'cpu'
        #height,width = source.shape[:2]
        #source = source[:,:width//2]
        img0 = cv2.resize(source, (1280,720), interpolation=cv2.INTER_LINEAR)
        img = letterbox(img0, imgsz, stride=stride)[0]
        
        # Convert
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)

        # Run inference
        if device.type != 'cpu':
            model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once
        t0 = time.time()
        im0s = img0
        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        # Inference
        with torch.no_grad():
            t1 = time_synchronized()
            [pred, anchor_grid], seg, ll = model(img)
            t2 = time_synchronized()

        tw1 = time_synchronized()
        pred = split_for_trace_model(pred, anchor_grid)
        tw2 = time_synchronized()

        # Apply NMS
        t3 = time_synchronized()
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes=classes, agnostic=agnostic_nms)
        t4 = time_synchronized()

        da_seg_mask = driving_area_mask(seg)
        ll_seg_mask = lane_line_mask(ll)
        # Kameranın orta noktasını belirleme
        mid_point = im0s.shape[1] // 2

        # Şerit çizgilerinin koordinatlarını bulma
        y_coords, x_coords = np.where(ll_seg_mask == 1)

       

        roi_y_start = 520
        roi_y_end = 720
        def get_roi_x_bounds(y):
            if y < roi_y_start or y > roi_y_end:
                return None, None
            # Lineer interpolasyon ile x1 ve x2 değerlerini hesapla
            x1 = int(240 + (y - roi_y_start) * (0 - 240) / (roi_y_end - roi_y_start))
            x2 = int(1100 + (y - roi_y_start) * (1280-1100) / (roi_y_end - roi_y_start))
            return x1, x2

        roi_mask = (y_coords >= roi_y_start) & (y_coords <= roi_y_end)
        roi_y_coords = y_coords[roi_mask]
        roi_x_coords = x_coords[roi_mask]

        mid_points = []
        fallback_points = []  # Yedek noktalar için liste

        # Her 10 y değeri için işlem yapma
        for y in range(roi_y_start, roi_y_end + 1):
            x1_static, x2_static = get_roi_x_bounds(y)
            if x1_static is None or x2_static is None:
                continue

            # Dinamik ROI sınırlarını hesapla
            x1_dynamic, x2_dynamic = self.get_dynamic_roi_bounds(y)
            
            # İki ROI'nin birleşimini al
            x1 = min(x1_static, x1_dynamic)
            x2 = max(x2_static, x2_dynamic)

            y_mask = roi_y_coords == y
            x_values = roi_x_coords[y_mask & (roi_x_coords >= x1) & (roi_x_coords <= x2)]
            
            if len(x_values) >= 2:
                # Birbirinden en az 500 piksel uzak olan iki x değeri bulma
                x_values_sorted = np.sort(x_values)
                x_diff = np.diff(x_values_sorted)
                valid_pairs = np.where(x_diff >= 500)[0]
                if len(valid_pairs) > 0:
                    x1_pair = x_values_sorted[valid_pairs[0]]
                    x2_pair = x_values_sorted[valid_pairs[0] + 1]
                    mid_x = (x1_pair + x2_pair) // 2
                    mid_points.append((mid_x, y))  # Orta noktayı listeye ekle
                    #print(f"y = {y}, x1 = {x1_pair}, x2 = {x2_pair}, Orta Nokta = ({mid_x}, {y})")
                else:
                    for x in x_values:
                        if x < self.image_center_x:
                            # Sol şerit ise: 335 pixel ekle
                            adjusted_x = x + 335
                        else:
                            # Sağ şerit ise: 335 pixel çıkar
                            adjusted_x = x - 335

                        # ROI sınırları içinde kalacak şekilde ayarla
                        adjusted_x = max(x1, min(x2, adjusted_x))
                        if adjusted_x > x1:
                            fallback_points.append((adjusted_x, y))
                            #print(f"Fallback: y = {y}, min_x = {adjusted_x}, adjusted_x = {adjusted_x}")

        # Eğer mid_points boşsa, fallback_points'i kullan
        if not mid_points and fallback_points:
            mid_points = fallback_points
            #print("Using fallback points")

        # Statik ROI çizimi (mavi)
        roi_top_left = (get_roi_x_bounds(roi_y_start)[0], roi_y_start)
        roi_top_right = (get_roi_x_bounds(roi_y_start)[1], roi_y_start)
        roi_bottom_left = (get_roi_x_bounds(roi_y_end)[0], roi_y_end)
        roi_bottom_right = (get_roi_x_bounds(roi_y_end)[1], roi_y_end)
        roi_pts = np.array([roi_top_left, roi_top_right, roi_bottom_right, roi_bottom_left], np.int32)
        roi_pts = roi_pts.reshape((-1, 1, 2))
        cv2.polylines(im0s, [roi_pts], isClosed=True, color=(255, 0, 0), thickness=2)

        # Dinamik ROI çizimi (kırmızı)
        dyn_top_left = (self.get_dynamic_roi_bounds(roi_y_start)[0], roi_y_start)
        dyn_top_right = (self.get_dynamic_roi_bounds(roi_y_start)[1], roi_y_start)
        dyn_bottom_left = (self.get_dynamic_roi_bounds(roi_y_end)[0], roi_y_end)
        dyn_bottom_right = (self.get_dynamic_roi_bounds(roi_y_end)[1], roi_y_end)
        dyn_pts = np.array([dyn_top_left, dyn_top_right, dyn_bottom_right, dyn_bottom_left], np.int32)
        dyn_pts = dyn_pts.reshape((-1, 1, 2))
        cv2.polylines(im0s, [dyn_pts], isClosed=True, color=(0, 0, 255), thickness=2)

            ### GENİŞLETİLMİŞ ROI TANIMI ###
        # Bu ROI, mevcut ROI'lerin biraz üstünde yer alır
        # Hem sola hem sağa genişlemiş şekilde tanımlanır
        ext_roi_y1 = 518  # üst
        ext_roi_y2 = 520  # alt
        ext_roi_x1 = 100  # en sol
        ext_roi_x2 = 1160  # en sağ

        # ROI kutusunu çiz
        cv2.rectangle(im0s, (ext_roi_x1, ext_roi_y1), (ext_roi_x2, ext_roi_y2), (0, 128, 255), 3)

        # ROI içindeki ll_seg_mask verilerini filtrele
        roi_mask_extended = (y_coords >= ext_roi_y1) & (y_coords <= ext_roi_y2) & \
                            (x_coords >= ext_roi_x1) & (x_coords <= ext_roi_x2)

        x_vals_ext = x_coords[roi_mask_extended]
        x_vals_ext_sorted = np.sort(x_vals_ext)
        x_diffs_ext = np.diff(x_vals_ext_sorted)
        threshold = 50  # çizgi arası minimum boşluk

        # Çizgi indekslerini bul (aralarındaki fark eşikten büyükse)
        lines_idx = np.where(x_diffs_ext > threshold)[0]

        #print("ROI içindeki x koordinatları:", x_vals_ext)
        #print("Çizgi indeksleri:", lines_idx)

        # Eski şerit değerini koruma mekanizması
        if not hasattr(self, 'last_valid_lane'):
            self.last_valid_lane = "None"

        # Genişletilmiş ROI'deki çizgilere göre şerit tespiti
        if len(lines_idx) >= 2:  # En az 2 çizgi aralığı (3 çizgi)
            # Çizgi pozisyonlarını al ve sırala
            lines = []
            for i in range(min(3, len(lines_idx)+1)):  # Maksimum 3 çizgi
                lines.append(x_vals_ext_sorted[lines_idx[i]] if i < len(lines_idx) else x_vals_ext_sorted[-1])
            lines = sorted(lines)
            
            # Arabanın x konumunu hesapla
            car_x = int(np.mean([pt[0] for pt in mid_points])) if mid_points else self.image_center_x
            
            # Şerit durumunu belirle
            if len(lines) >= 3:  # 3 çizgi varsa
                if car_x < lines[0]:
                    self.current_lane = "cok_sol"
                elif lines[0] < car_x < lines[1]:
                    self.current_lane = "sol"
                elif lines[1] < car_x < lines[2]:
                    self.current_lane = "sag"
                else:
                    self.current_lane = "cok_sag"
            else:  # 2 çizgi varsa
                lane_center = (lines[0] + lines[1]) / 2
                if car_x < lane_center:
                    self.current_lane = "sol"
                else:
                    self.current_lane = "sag"
            
            # Geçerli değeri kaydet
            self.last_valid_lane = self.current_lane
        else:
            # Yeterli çizgi yoksa son geçerli değeri koru
            self.current_lane = self.last_valid_lane if self.last_valid_lane else "sag"

        # Görselleştirme (HER DURUMDA göster)
        cv2.putText(im0s, f"Serit: {self.current_lane}", (20, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 131), 3, cv2.LINE_AA)


        if len(mid_points) >= 2:
            for i in range(1, len(mid_points)):
                cv2.line(im0s, mid_points[i - 1], mid_points[i], (0, 255, 0), thickness=2)
            # Orta noktaları daire olarak çiz
            for point in mid_points:
                cv2.circle(im0s, point, radius=5, color=(0, 0, 255), thickness=-1)

            # Tekerlek açısını hesapla ve gönder
            steering_angle = self.calculate_steering_angle(mid_points)
            self.publish_cmd_vel(steering_angle)

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0s.shape).round()
                # Write results
                for *xyxy, conf, cls in reversed(det):
                    plot_one_box(xyxy, im0s, line_thickness=3)
            # Show result
            show_seg_result(im0s, (da_seg_mask, ll_seg_mask), is_demo=True)
          
            cv2.imshow("Static (Mavi) + Dynamic (Kirmizi) ROI", im0s)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    