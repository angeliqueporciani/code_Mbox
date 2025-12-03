# Code Mbox

Ce repertoire contiens les codes nécessaires à l'initialisation des Mbox. 
 
 01_capture_calib.py permet de prendre plusieurs photos avec le damier afin de calculer les valeurs de calibration et de correction de la distorsion de l'image. 
 
02_valeur_calib.py permet de calculer les valeurs de la matrice de distorsion et de correction de l'image. Ces valeurs doivent être recuperé pour être mises dans le script 03_detect_puits_circle.py et 04_tracking_seuil_intensite_min70. 

03_detect_puits_circle.py permet de lancer la detection des puits automatiquement et d'enregistrement de l'image avec puits dans le dossier. 

04_tracking_seuil_intensite_min70.py permet de lancer le tracking automatique des larves
 
 app.py contient le code necessaire au lancement de l'application en local. 
 
 Le dossier static contiens les images qui vont être proposé dans l'interface graphique de l'application. 
 
 