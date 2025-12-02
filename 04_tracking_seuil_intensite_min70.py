import time, json, csv, datetime, os
import cv2
import numpy as np
import pigpio
from picamera2 import Picamera2

# === Calibration caméra ===
K = np.array([[5282.32084, 0.0, 849.301187],  # K est la matrice intrinsèque, elle contient la longueur focale en X et en Y et les coordonnées X et Y du centre optique 
              [0.0, 5279.52624, 295.988566],
              [0.0, 0.0, 1.0]])
dist = np.array([[0.729973676, 8.28433969, -0.0403485395,
                  -0.00650735379, -295.103845]]) # Contient les coefficients de distorsion 

RAYON_PUIT_MM = 18.0
TEMPS_ATTENTE = 120
LED_GPIO = 18

# === Gestion profil LED ===
def charger_profil_led(fichier="profil_led.json"):  # Contient les valeurs ecrites par l'operateur grace a l'inteface web pour le changement de l intensité des leds
    with open(fichier, "r") as f:
        profil = json.load(f)
    montee = {int(k): v for k, v in profil["montee"].items()}  # On extrait le profil de montée du fichier json
    descente = {int(k): v for k, v in profil["descente"].items()} # On extrait le profil de descente du fichier json
    return profil["temps_total"], montee, descente  # La fonction retourne un tuple qui contient le temps total, le profil de montée et le profil de descente 

# Cette fonction permet de determiner l intensité des leds selon le temps ecoulés apres la phase de stabilisation au tout depart 
def calculer_intensite(t, temps_total, montee, descente):  # A savoir: t= temps ecoulé apres la phase de stabilisation definit plus haut TEMPS_ATTENTE
    if t <= max(montee.values()):   # Si le temps ecoules est inferieur a la la valeur max de montée ca signifie que t est encore dans la phase de montée progressive
        intensite = 0          
        for p in sorted(montee.items(), key = lambda x: x[1]): # p represente le couple (intensité : temps associé) donc ici on parcout chaque couple du dictionnaire montée dans l'ordre chronologique
            if t >= p[1]:       # p[0] = intensite et p[1] = temps donc ici par exemple dans mon cas: (80:0, 90:30) si t= 3  alors 3 >= 0 donc intensite = 80
                intensite = p[0]
        return intensite
        
    if t <= min(descente.values()):
        return 100
    
    intensite= 100                                          # Meme logique
    for p in sorted(descente.items(), key=lambda x: x[1]):
        if t >= p[1]:
            intensite = p[0]
    return intensite

# === Classe Larve qui permet de representer une larve et suivre sa trajectoire repris du code de MR ROY: LR69 github===
class Larve:
    def __init__(self, num, cx_px, cy_px, mm_par_px, seuil_mm=0.3):  # Initialise une larve suivi dans un puits
        self.num = num   # id de la larve
        self.cx_px, self.cy_px = cx_px, cy_px  # coordonnées du centre du puit en pixels
        self.mm_par_px = mm_par_px   # coefficient de conversion pixels2mm
        self.seuil_mm = seuil_mm   # seuil minimal pour dire qu'une larve a bouge 
        self.positions = []   # Liste des positions enregistrés
        self.derniere_pos_px = None  # Derniere position connu en pixels

    def _immobile_flag(self, pos_px):
        if self.derniere_pos_px is None:   
            return 0     # On considere que la larve n a pas bougé si il n y a pas de position precedente 
        dx_mm = (pos_px[0] - self.derniere_pos_px[0]) * self.mm_par_px
        dy_mm = (pos_px[1] - self.derniere_pos_px[1]) * self.mm_par_px         # On compare la position actuelle à la précedante en x dans la variable dx_mm et en y dans dy_mm pour determiner l'immobilité
        return int(np.hypot(dx_mm, dy_mm) < self.seuil_mm)                     # Retourne 1 si le deplacement est inferieur au seuil donc immobile, sinon 0 donc mobile 

    def _enregistrer(self, pos_px, t, immobile, fichier_numero):  # Enregistre une nouvelle position avec les données utiles 
        dx_px = pos_px[0] - self.cx_px   # Permet d obtenir les coordonnées avec comme point de reference le centre du puits en pixels
        dy_px = pos_px[1] - self.cy_px
        x_mm = round(dx_px * self.mm_par_px, 3) # Coordonnées avec comme point de referece le centre du puit mais les coordonnées sont récupérées en mm
        y_mm = round(dy_px * self.mm_par_px, 3)
        horloge = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.positions.append((round(t, 2), x_mm, y_mm, immobile, horloge, fichier_numero))
        self.derniere_pos_px = pos_px

    def update(self, pos_px, t, fichier_numero):  # Fonction qui alimente le fichier CSV
        immobile = self._immobile_flag(pos_px)   # Verifie si la larve est immobile ou non 
        self._enregistrer(pos_px, t, immobile, fichier_numero)  # Enregistre la position et met a jour la memoire sel.derniere_pos_px

    def repeter_derniere_position(self, t, fichier_numero):
        if self.derniere_pos_px is not None:
            self._enregistrer(self.derniere_pos_px, t, 1, fichier_numero)

    def reset_positions(self):
        self.positions = []

# === LED PWM ===
def configurer_led(pi, LED_GPIO):
    pi.set_PWM_frequency(LED_GPIO, 10000)
    pi.set_PWM_dutycycle(LED_GPIO, 255)

def eteindre_led(pi, LED_GPIO):
    pi.set_PWM_dutycycle(LED_GPIO, 0)
    pi.stop()

# === Initialisation caméra noIR module 3===
def initialiser_camera():
    picam2 = Picamera2()
    sensor_width, sensor_height = picam2.sensor_resolution
    coeff_zoom = 1.0
    crop_width = int(sensor_width / coeff_zoom)
    crop_height = int(sensor_height / coeff_zoom)
    crop_x = (sensor_width - crop_width) // 2
    crop_y = (sensor_height - crop_height) // 2

    config = picam2.create_video_configuration(
        controls={
            "ScalerCrop": (crop_x, crop_y, crop_width, crop_height),
            "FrameRate": 5,
            "AwbEnable": False,
            "AeEnable": False,
            "AnalogueGain": 2.0,
            "ExposureTime": 480
        },
        main={"size": (1280, 720), "format": "RGB888"}
    )

    picam2.configure(config)
    picam2.start()
    time.sleep(2)

    test_img = picam2.capture_array()
    h, w = test_img.shape[:2]
    new_K, _ = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))  # nouvelle matrice camera optimise 
    mapx, mapy = cv2.initUndistortRectifyMap(K, dist, None, new_K, (w, h), cv2.CV_32FC1)   #matrice de correction de  de distorsion calculé lors de la calibration de la camera, 
                                                                                           #elle contient pour chaque pixel de l image final, les coordonnées d'ou venir chercher le pixel de l image d origine

    return picam2, mapx, mapy

# === SOLUTION 1: Fonctions pour réinitialiser MOG2 ===
def initialiser_mog2_par_puits(nb_puits):
    """Crée une liste de MOG2 pour chaque puits"""
    return [cv2.createBackgroundSubtractorMOG2(history=125, varThreshold=60, detectShadows=False)
            for _ in range(nb_puits)]

def reinitialiser_mog2_par_puits(nb_puits):
    """Réinitialise complètement tous les MOG2"""
    print("[REINIT] Réinitialisation de tous les MOG2 suite au changement d'intensité")
    return [cv2.createBackgroundSubtractorMOG2(history=125, varThreshold=60, detectShadows=False)
            for _ in range(nb_puits)]

def detecter_larve_mog2(frame, cx, cy, rayon, mog2):
    x1, x2 = cx - rayon, cx + rayon  # Définit les bornes du carré autour du puits 
    y1, y2 = cy - rayon, cy + rayon
    roi = frame[y1:y2, x1:x2].copy()   # Region d'interet centrée sur le puits
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)) # Amelioration du contraste local
    gray = clahe.apply(gray)

    # On applique un masque circulaire contré sur le puits 
    mask = np.zeros_like(gray)
    cv2.circle(mask, (rayon, rayon), rayon, 255, -1)
    gray = cv2.bitwise_and(gray, gray, mask=mask)
    
    gray = cv2.GaussianBlur(gray, (5, 5), 0)  # Réduction du bruit

    fgmask = mog2.apply(gray)  # Soustraction de fond mog2 appliqué sur l'image (gray) 
    thresh = cv2.threshold(fgmask, 30, 255, cv2.THRESH_BINARY)[1]   # seuillage binaire pour séparer les zones mobiles 
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)   # Trouver les contours des zones detectés

    # Ici on vient cibler les contours en choisissant dans la ROI le plus petit contour, ca permet d'eviter les gros artefacts
    min_area = float('inf')
    best_cnt = None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if 2 < area < 400 and area < min_area:
            min_area = area
            best_cnt = cnt

    if best_cnt is not None:
        M = cv2.moments(best_cnt)
        if M["m00"] != 0:
            cx_rel = int(M["m10"] / M["m00"])  # Coordonnées X du centre
            cy_rel = int(M["m01"] / M["m00"])  # Coordonnées Y du centre
            abs_pos = (cx_rel + x1, cy_rel + y1)
            cv2.circle(frame, abs_pos, 4, (0, 255, 0), -1)
            return frame, abs_pos

    return frame, None
    
def charger_puits(fichier="centres_puits.json"):
    with open(fichier, "r") as f:
        data = json.load(f)
    return [(int(x), int(y), int(r)) for x, y, r in data]  # Retourne un tuple avec le centre (x, y) et le rayon en pixels de chaque puits 

def exporter_csv(larves, fichier_numero):
    nom_fichier = f"trajectoires_larves_heure{fichier_numero}.csv"
    with open(nom_fichier, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["id", "timestamp", "x_mm", "y_mm", "immobile", "id_raspberry", "date_horloge", "fichier_numero"])
        for larve in larves:
            for ligne in larve.positions:
                timestamp, x_mm, y_mm, immobile, horloge, fichier_numero_pos = ligne
                writer.writerow([larve.num, timestamp, x_mm, y_mm, immobile, 2, horloge, fichier_numero_pos])
            larve.reset_positions()
    print(f"[EXPORT] Fichier {nom_fichier} enregistré.")

def verifier_calibration_puits(picam2, mapx, mapy, path="puits_detectes.png"):
    if not os.path.exists(path):
        print("Image non trouvee") 
        return
        
    img_reelle = picam2.capture_array()
    img_reelle = cv2.remap(img_reelle, mapx, mapy, cv2.INTER_LINEAR)
    img_reelle = cv2.cvtColor(img_reelle, cv2.COLOR_RGB2BGR) 
    
    img_calib = cv2.imread(path)
    if img_calib is None: 
        print("Impossible de lire l'image de calibration") 
        return
    superpose = cv2.addWeighted(img_reelle, 0.5, img_calib, 0.5, 0)
    cv2.imshow("Verification calibration des puits", superpose)
    print("[INFO] Appuyez sur une touche pour continuer apres la verification")
    
    cv2.waitKey(0)
    cv2.destroyWindow("Verification calibration des puits") 
    
# === Programme principal===
def main():
    puits = charger_puits()
    mm_par_px = [RAYON_PUIT_MM / r for _, _, r in puits]
    larves = [Larve(i+1, cx, cy, mm_par_px[i]) for i, (cx, cy, _) in enumerate(puits)]  # Création d'un objet Larve par puits

    # Charger profil LED
    temps_total, profil_montee, profil_descente = charger_profil_led()

    # Initialiser la Led via GPIO
    pi = pigpio.pi()
    configurer_led(pi, LED_GPIO)

    # Initialiser la camera et les cartes de correction de distorsion 
    picam2, mapx, mapy = initialiser_camera()
    mog2_par_puits = initialiser_mog2_par_puits(len(puits))  # Initialisation de la soustraction de fond mog2

    verifier_calibration_puits(picam2, mapx, mapy)
    
    print(f"[INFO] Stabilisation du fond pendant {TEMPS_ATTENTE} secondes...")
    t0 = time.time()   # On demarre le chronometre principal
    fichier_numero = 1
    prochaine_sauvegarde = t0 + 3600  # permet de sauvegarder un fichier toutes les heures
    dernier_pourcent = 0  # Derniere intensite de led apliquée
    en_pause = False  # Flag de pause passe à True si on est en pause suite a un changement d intensité
    pause_start = 0    # Horodatage du debut de la pause 
    DUREE_PAUSE = 15.0
    # Nouvelle variable pour le délai post-stabilisation ( juste après la pause apres un changement d'intensité cela permet de stabiliser la soustraction de fond)
    en_delai_post_stabilisation = False
    delai_post_stabilisation_start = 0
    DUREE_DELAI_POST_STABILISATION = 5.0
    SEUIL_CHANGEMENT = 10  # Seuil pour détecter un changement d'intensité significatif
    
    while True:
        t = time.time() - t0  # Temps écoulé depuis le debut 
        
        # Si la durée total du suivi est depassé, on arrete (temps_total correspond a la durée de l experience principale apres la phase de stabilisation TEMPS_ATTENTE) 
        if t >= TEMPS_ATTENTE + temps_total:   
            print("[INFO] Temps total ecoulé")
            break 
        
        # Capture une image et applique la correction de distorsion
        distorted = picam2.capture_array()
        frame = cv2.remap(distorted, mapx, mapy, cv2.INTER_LINEAR)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        # Mettre à jour LED
        if t < TEMPS_ATTENTE:
            pourcent = 80  # Pendant la phase de stabilisation, les leds ont une intensité fixe a placer dans la variable pourcent
        else:
            t_relative = t - TEMPS_ATTENTE
            pourcent = calculer_intensite(t_relative, temps_total, profil_montee, profil_descente)  # Calcul de l'intensité après la phase de stabilisation 
       
        # Détecter changement d'intensité et réinitialiser MOG2
        changement_intensite = abs(pourcent - dernier_pourcent)
        if changement_intensite >= SEUIL_CHANGEMENT and not en_pause and not en_delai_post_stabilisation:
            print(f"[CHANGEMENT] Intensité: {dernier_pourcent}% → {pourcent}% (Δ={changement_intensite}%)")
            # Réinitialiser tous les MOG2
            mog2_par_puits = reinitialiser_mog2_par_puits(len(puits))
            # Reset de la pos apres stabilisation suite a un changement 
            for larve in larves:
                larve.derniere_pos_px =  None
            en_pause = True  
            pause_start = time.time()
            print("[INFO] Stabilisation suite au changement d'intensité et réinitialisation MOG2")
        
        dernier_pourcent = pourcent # On met a jour la valeur de reference
        
        # On applique la nouvelle intensité led via PWM
        duty = int(pourcent * 255 / 100) 
        pi.set_PWM_dutycycle(LED_GPIO, duty)
        
        # Vérifier si la durée de pause est finie
        if en_pause:
            if time.time() - pause_start < DUREE_PAUSE: 
                # Pendant la pause, continuer à alimenter les MOG2 pour l'apprentissage du fond
                for i, (cx, cy, r) in enumerate(puits):
                    _ = detecter_larve_mog2(frame, cx, cy, r, mog2_par_puits[i])
                    
                temps_restant = int(DUREE_PAUSE - (time.time() - pause_start))
                cv2.putText(frame, f"Stabilisation changement intensite... {temps_restant}s", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
                cv2.putText(frame, f"Intensite LED: {pourcent}%", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 165, 0), 2)
                cv2.imshow("Tracking multi-larves", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue
            else:
                en_pause = False
                # Démarrer le délai post-stabilisation
                en_delai_post_stabilisation = True
                delai_post_stabilisation_start = time.time()
                print("[INFO] Stabilisation terminée, début du délai post-stabilisation (5s)")
                
        # Vérifier si le délai post-stabilisation est fini
        if en_delai_post_stabilisation:
            if time.time() - delai_post_stabilisation_start < DUREE_DELAI_POST_STABILISATION:
                # Pendant le délai, continuer le tracking mais ne pas exporter les données
                for i, (cx, cy, r) in enumerate(puits):
                    frame, pos = detecter_larve_mog2(frame, cx, cy, r, mog2_par_puits[i])
                    # On ne fait pas larves[i].update() ici pour éviter l'export
                    if pos is None and larves[i].derniere_pos_px:
                        cv2.circle(frame, larves[i].derniere_pos_px, 4, (0, 0, 255), -1)
                
                temps_restant = int(DUREE_DELAI_POST_STABILISATION - (time.time() - delai_post_stabilisation_start))
                cv2.putText(frame, f"Delai post-stabilisation... {temps_restant}s", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                cv2.putText(frame, f"Intensite LED: {pourcent}%", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 165, 0), 2)
                cv2.imshow("Tracking multi-larves", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue
            else:
                en_delai_post_stabilisation = False
                print("[INFO] Délai post-stabilisation terminé, reprise complète du tracking avec export")
                
        # Tracking normal avec export des données
        for i, (cx, cy, r) in enumerate(puits):
            frame, pos = detecter_larve_mog2(frame, cx, cy, r, mog2_par_puits[i])
            if t >= TEMPS_ATTENTE:
                if pos:
                    larves[i].update(pos, t, fichier_numero)
                else:
                    larves[i].repeter_derniere_position(t, fichier_numero)
                    if larves[i].derniere_pos_px:
                        cv2.circle(frame, larves[i].derniere_pos_px, 4, (0, 0, 255), -1)

        # Affichage des informations
        if t < TEMPS_ATTENTE:
            cv2.putText(frame, f"Stabilisation du fond... ({int(TEMPS_ATTENTE - t)}s)",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        else:
            # Afficher l'intensité LED actuelle
            cv2.putText(frame, f"Intensite LED: {pourcent}%", 
                       (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        if time.time() >= prochaine_sauvegarde:
            exporter_csv(larves, fichier_numero)
            fichier_numero += 1
            prochaine_sauvegarde += 3600

        cv2.imshow("Tracking multi-larves", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    picam2.stop()
    cv2.destroyAllWindows()
    eteindre_led(pi, LED_GPIO)

    # Export final
    nom_final = "trajectoires_larves_complet.csv"
    with open(nom_final, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["id", "timestamp", "x_mm", "y_mm", "immobile", "id_raspberry", "date_horloge", "fichier_numero"])
        for larve in larves:
            for ligne in larve.positions:
                timestamp, x_mm, y_mm, immobile, horloge, fichier_numero_pos = ligne
                writer.writerow([larve.num, timestamp, x_mm, y_mm, immobile, 2, horloge, fichier_numero_pos])
    print(f"[EXPORT FINAL] Données finales enregistrées dans {nom_final}")

if __name__ == "__main__":
    main()
