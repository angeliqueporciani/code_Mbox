import cv2
import numpy as np
import glob
import os

# === PARAMÈTRES UTILISATEUR ===
pattern_size = (9, 6)          # (cols, rows) : coins internes
square_size = 25.0             # en millimètres
images_folder = "images_calibration"
output_file = "calibration_data.npz"

# === PRÉPARATION DES POINTS 3D (monde réel) ===
objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)  # On crée un tableu de zéros de taille 54 vu qu'on a 54 points chacun avec 3 coordonnées
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) # permet d obtenir automatiquement les coordonnées X et Y des points 3D 
objp *= square_size  # applique la taille réelle

objpoints = []  # Points 3D
imgpoints = []  # Points 2D

# === CHARGEMENT DES IMAGES ===
images = glob.glob(os.path.join(images_folder, "*.jpg")) + \
         glob.glob(os.path.join(images_folder, "*.png"))

if not images:
    raise ValueError(f"Aucune image trouvée dans le dossier '{images_folder}'.")

ok = 0
fail = 0

# Recherche du damier dans chaque image 
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)  # Recherche automatique des coins internes du damiers 

    if ret:
        print(f" Damier détecté dans : {os.path.basename(fname)}")
        objpoints.append(objp)
        # amélioration précision des coins
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        # Affichage (optionnel, désactivé si SSH)
        # cv2.drawChessboardCorners(img, pattern_size, corners2, ret)
        # cv2.imshow('Calibration - Coins détectés', img)
        # cv2.waitKey(200)
        ok += 1
    else:
        print(f" ÉCHEC détection dans : {os.path.basename(fname)}")
        fail += 1

# cv2.destroyAllWindows()  # si l'affichage était activé

print("\n--- Bilan détection ---")
print(f"  {ok} images valides")
print(f"  {fail} images non valides")

# === CALIBRATION ===
if ok < 3:
    print("\n Trop peu d’images valides pour une calibration fiable (minimum recommandé : 5 à 10).")
else:
    print("\n Calibration en cours...")
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)  # Ici on calcule les parametres de calibration a partir des correspondances 3D et 2D ainsi que la taille des images

    print("\n Calibration terminée.")
    print(" Matrice intrinsèque (K):\n", K)
    print(" Coefficients de distorsion:\n", dist.ravel())

    np.savez(output_file, K=K, dist=dist)
    print(f"\n Paramètres sauvegardés dans : {output_file}")
