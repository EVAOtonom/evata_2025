import open3d as o3d
import numpy as np
import cv2

def pcd_to_pgm(pcd_path, pgm_path, resolution=0.05):
    # PCD dosyasını yükle
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)

    # X ve Y koordinatlarını al
    x = points[:, 0]
    y = points[:, 1]

    # Minimum ve maksimum değerleri bul
    x_min, x_max = x.min(), x.max()
    y_min, y_max = y.min(), y.max()

    # Görüntü boyutunu çözünürlük ve koordinat aralığına göre hesapla
    width = int(np.ceil((x_max - x_min) / resolution)) + 1
    height = int(np.ceil((y_max - y_min) / resolution)) + 1

    # Piksel koordinatlarına dönüştür
    x_pix = ((x - x_min) / resolution).astype(int)
    y_pix = ((y - y_min) / resolution).astype(int)

    # Beyaz arka planlı boş görüntü oluştur
    img = np.ones((height, width), dtype=np.uint8) * 255

    # Noktaları siyah olarak işaretle
    for xi, yi in zip(x_pix, y_pix):
        # OpenCV görüntüde y ekseni ters olduğu için yüksekliği kullanıyoruz
        img[height - 1 - yi, xi] = 0

    # PGM dosyasını kaydet
    cv2.imwrite(pgm_path, img)
    print(f"PGM dosyası kaydedildi: {pgm_path}")

# Örnek kullanım
pcd_to_pgm("map.pcd", "harita.pgm")

