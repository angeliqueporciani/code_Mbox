import os, glob, subprocess, zipfile, time, json
from flask import Flask, render_template, redirect, url_for, send_file, request
import pandas as pd
import matplotlib.pyplot as plt

app = Flask(__name__)

# ---------- Utilitaires ----------
STATIC_DIR = "static"
os.makedirs(STATIC_DIR, exist_ok=True)

def tracer_trajectoires(dossier=".", sortie="static/trajectoires.png"):
    """
    Lit tous les CSV 'trajectoires_larves_*.csv' (+ complet) et trace toutes les trajectoires.
    Aucun échantillonnage.
    """
    fichiers = sorted(glob.glob(os.path.join(dossier,"trajectoires_larves_*.csv")))
    if os.path.exists("trajectoires_larves_complet.csv"):
        fichiers.append("trajectoires_larves_complet.csv")

    if not fichiers:
        raise FileNotFoundError("Aucun CSV de trajectoires trouvé.")

    df = pd.concat([pd.read_csv(f) for f in fichiers], ignore_index=True)
    
    # Supprimer les doublons potentiels (même timestamp, même id, même position)
    df = df.drop_duplicates(subset=['id', 'timestamp', 'x_mm', 'y_mm'], keep='first')
    
    # Trier par id et timestamp pour des trajectoires cohérentes
    df = df.sort_values(['id', 'timestamp'])

    plt.figure(figsize=(12, 8))
    
    # Obtenir le nombre unique de larves
    larves_uniques = sorted(df['id'].unique())
    nb_larves = len(larves_uniques)
    
    # Utiliser une palette de couleurs avec suffisamment de couleurs
    colors = plt.cm.get_cmap("tab10" if nb_larves <= 10 else "tab20", nb_larves)
    
    for idx, larve_id in enumerate(larves_uniques):
        sub = df[df['id'] == larve_id]
        plt.plot(sub['x_mm'], sub['y_mm'],
                 label=f"Larve {larve_id}",
                 alpha=0.9,
                 linewidth=1.5,
                 color=colors(idx))

    plt.xlabel("X (mm)")
    plt.ylabel("Y (mm)")
    plt.title(f"Trajectoires complètes des larves (N={nb_larves})")
    plt.grid(True)
    plt.axis("equal")
    
    # Ajuster la légende selon le nombre de larves
    if nb_larves <= 12:
        plt.legend(loc="center left", bbox_to_anchor=(1.02, 0.5), fontsize=8, frameon=False)
    else:
        plt.legend(loc="upper right", fontsize=6, frameon=False, ncol=2)
    
    plt.tight_layout()
    os.makedirs(os.path.dirname(sortie), exist_ok=True)
    plt.savefig(sortie, dpi=150, bbox_inches="tight")
    plt.close()
    
    print(f"[INFO] Graphique généré avec {nb_larves} larves uniques")
    return sortie

def nettoyer_anciens_csv():
    """
    Supprime les anciens fichiers CSV de trajectoires pour éviter les doublons
    """
    fichiers_a_supprimer = glob.glob("trajectoires_larves_heure*.csv")
    for fichier in fichiers_a_supprimer:
        try:
            os.remove(fichier)
            print(f"Supprimé: {fichier}")
        except OSError:
            pass

def creer_archive_csv(dest=os.path.join(STATIC_DIR, "trajectoires_csv.zip")):
    # Chercher uniquement les fichiers de la session actuelle
    fichiers_heure = glob.glob("trajectoires_larves_heure*.csv")
    fichiers_complet = ["trajectoires_larves_complet.csv"] if os.path.exists("trajectoires_larves_complet.csv") else []
    
    fichiers = fichiers_heure + fichiers_complet
    fichiers = [f for f in fichiers if os.path.exists(f)]
    
    if not fichiers:
        raise FileNotFoundError("Aucun fichier CSV de la session actuelle trouvé.")

    with zipfile.ZipFile(dest, "w", zipfile.ZIP_DEFLATED) as zf:
        for f in fichiers:
            zf.write(f)
    
    print(f"Archive créée avec {len(fichiers)} fichiers: {[os.path.basename(f) for f in fichiers]}")
    return dest

# ---------- Routes ----------
@app.route("/")
def index():
    return render_template("index.html", config={"timestamp":int(time.time())})

# Chemin vers index html pour la calibration des puits
@app.route("/run_calibration", methods=["POST"])
def run_calibration():
    subprocess.Popen(["python3", "detect_puits_circle.py"])
    time.sleep(2)  # Attendre que l'image soit générée
    return redirect(url_for("index"))

# Chemin pour tracking des larves
@app.route("/run_tracking", methods=["POST"])
def run_tracking():
    # Nettoyer les anciens fichiers avant de commencer un nouveau tracking
    nettoyer_anciens_csv()
    subprocess.Popen(["python3", "track_larves.py"])
    return redirect(url_for("index"))

# On vérifie si l'image existe
@app.route("/check_calibration_image")
def check_calibration_image():
    image_path = os.path.join(STATIC_DIR, "puits_detectes.png")
    if os.path.exists(image_path):
        return {"exists": True, "timestamp": int(os.path.getmtime(image_path))}
    else:
        return {"exists": False}

# Ajouter cette nouvelle route pour servir l'image de calibration
@app.route("/calibration_image")
def calibration_image():
    image_path = os.path.join(STATIC_DIR, "puits_detectes.png")
    if os.path.exists(image_path):
        return redirect(url_for("static", filename="puits_detectes.png") + f"?t={int(time.time())}")
    else:
        return "Image de calibration non trouvée", 404
def plot_trajectoires():
    try:
        img_path = tracer_trajectoires()
    except Exception as e:
        return f"Erreur : {e}", 500
    return redirect(url_for("static", filename=os.path.basename(img_path)) + f"?t={int(time.time())}")

# Chemin pour le CSV 
@app.route("/download_csv", methods=["GET"])
def download_csv():
    zip_path = os.path.join(STATIC_DIR, "trajectoires_csv.zip")
    if os.path.exists(zip_path):
        try:
            os.remove(zip_path)
        except PermissionError:
            zip_path = zip_path.replace(".zip", f"_{int(time.time())}.zip")
    try:
        zip_path = creer_archive_csv(zip_path)
    except FileNotFoundError:
        return "Aucun fichier CSV à archiver.", 404

    return send_file(zip_path, as_attachment=True, download_name="trajectoires_csv.zip")
 
@app.route("/set_led_profile", methods=["POST"])
def set_led_profile():
    try:
        # Temps total de la manipulation
        temps_total = int(request.form["temps_total"])

        # Montée : récupérer les seuils de 0 à 100%
        montee = {}
        for i in range(0, 101, 10):
            val = request.form.get(f"montee_{i}")
            if val is not None and val.strip() != "":
                montee[str(i)] = int(val)

        # Descente : de 100% à 0%
        descente = {}
        for i in range(100, -1, -10):
            val = request.form.get(f"descente_{i}")
            if val is not None and val.strip() != "":
                descente[str(i)] = int(val)

        profil = {
            "temps_total": temps_total,
            "montee": montee,
            "descente": descente
        }

        with open("profil_led.json", "w") as f:
            json.dump(profil, f, indent=2)

        print("[INFO] Profil LED enregistré :", profil)
        return redirect(url_for("index"))

    except Exception as e:
        return f"Erreur lors de l'enregistrement du profil LED : {e}", 500

# ---------- Lancement ----------
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, threaded=True)
