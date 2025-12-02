import cv2
import numpy as np
import json
from picamera2 import Picamera2
import time
import RPi.GPIO as GPIO

# === Calibration manuelle (résultats précédents) ===
K = np.array([[2.02642615e+03, 0.00000000e+00 ,7.35001211e+02],
 [0.00000000e+00 ,2.03266897e+03, 3.94862508e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])  # K est la matrice intrinsèque, elle contient la longueur focale en X et en Y et les coordonnées X et Y du centre optique 

dist = np.array([-1.05330068e-01 , 1.94991428e+00 ,-9.42214661e-04,  3.43771505e-03,
 -5.59925686e+00])

#K = np.array([[1.39642048e+03,0.00000000e+00, 6.51229604e+02],
 #[0.00000000e+00, 1.39874493e+03, 4.00142560e+02],
 #[0.00000000e+00,0.00000000e+00 ,1.00000000e+00]])
#dist = np.array( [-1.04439621e-01, 1.24333944e+00, -6.08488889e-04, 2.13086365e-03,
# -2.69943666e+00])

# === GPIO : LED sur GPIO 18 ===
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
GPIO.output(18, GPIO.HIGH)
print("[INFO] LED allumée (GPIO 18)")

# === Initialisation caméra ===
picam2 = Picamera2()
sensor_width, sensor_height = picam2.sensor_resolution
coeff_zoom = 1.0

crop_width = int(sensor_width / coeff_zoom)
crop_height = int(sensor_height / coeff_zoom)
crop_x = (sensor_width - crop_width) // 2
crop_y = (sensor_height - crop_height) // 2

config= picam2.create_video_configuration(
    controls={"ScalerCrop": (crop_x, crop_y, crop_width, crop_height)},
    main={"size": (1280, 720), "format":"RGB888"}
)
picam2.configure(config)
picam2.start()
time.sleep(2)

# === Capture et correction de distorsion ===
print("[INFO] Capture en cours...")
image_distorted = picam2.capture_array()
h, w = image_distorted.shape[:2]
new_K, _ = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))
mapx, mapy = cv2.initUndistortRectifyMap(K, dist, None, new_K, (w, h), cv2.CV_32FC1)
image = cv2.remap(image_distorted, mapx, mapy, cv2.INTER_LINEAR)
cv2.imwrite("image_redressee.png", image)

# === Prétraitement : CLAHE + flou ===
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
enhanced = clahe.apply(gray)
blurred = cv2.medianBlur(enhanced, 5)

# === HoughCircles ===
circles = cv2.HoughCircles(
    blurred,
    method=cv2.HOUGH_GRADIENT,
    dp=1.0,
    minDist=120,
    param1=80,
    param2=20,
    minRadius=65,
    maxRadius=80
)

centres_puits = []

if circles is not None:
    circles = np.uint16(np.around(circles))
    for i, (x, y, r) in enumerate(circles[0, :]):
        cv2.circle(image, (x, y), r, (0, 255, 0), 2)
        cv2.circle(image, (x, y), 3, (0, 0, 255), -1)
        centres_puits.append((int(x), int(y), int(r)))
        print(f"[DEBUG] Puits #{i+1} → (x={x}, y={y}), rayon={r}")
else:
    print("[INFO] Aucun puits détecté.")

# === Sauvegarde ===
with open("centres_puits.json", "w") as f:
    json.dump(centres_puits, f, indent=2)
cv2.imwrite("puits_detectes.png", image)

# === Nettoyage ===
picam2.stop()
GPIO.output(18, GPIO.LOW)
GPIO.cleanup()
print("[INFO] LED éteinte (GPIO 18) et GPIO nettoyé")
print(f"[INFO] {len(centres_puits)} puits détectés.")
