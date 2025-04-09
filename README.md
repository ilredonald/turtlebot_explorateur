## TURTLEBOT EXPLORATEUR :

Ce dépôt décrit le système d’un TurtleBot capable d’explorer de façon autonome un environnement inconnu et de détecter les motifs présents dans l’arène. La navigation et la détection s’appuient sur ROS, un LiDAR et une caméra embarquée.

## Objectif du robot :

•	Explorer automatiquement une zone inconnue.

•	Éviter les obstacles.

•	Cartographier l’environnement.

•	Détecter des motifs spécifiques (formes, couleurs, QR codes, etc.).

•	afficher les positions de ces motifs sur la carte.

## Matériel nécessaire :

•	TurtleBot (Burger ou Waffle)

•	LIDAR

•	Caméra RGB

•	Ordinateur avec ROS (Noetic ou autre)

•	Outils de visualisation (RViz, rqt, etc.)


## 🚀 Lancement de la détection :

✅ **Lancer le launch `projet.launch` du package `detection`:**
```bash
roslaunch detection projet.launch
```
## Configurations :

✅ **Dans rqt, cliquer sur Plugins :**

choisir `Visualization` et cliquer sur `Image View`et enfin choisir le topic : `/camera/image_annotated` : pour afficher le flux vidéo de la camera avec les différentes informations comme la distance à l’objet détecté etc.

✅ **Dans `rviz`, cliquer sur `add` et choisir les topics :** 

•	`/marker` : pour visualiser les marques des motifs détectés sur la carte.

•	`/path` : pour visualiser le chemin parcouru par le robot.

## 🗺️ Lancement de l’exploration et de la cartographie

✅ **Lancer le script `connect_and_launch_frontier.sh` :**
```bash
./connect_and_launch_frontier.sh
```
Nb : On vous demandera de renseigner l’adresse IP d’un turtlebot pour s’y connecter.
