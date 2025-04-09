## 🚀 Lancement de la détection

✅ **Lancer le launch `projet.launch` du package `detection`:**
```bash
roslaunch detection projet.launch
```
## Configurations :

✅ **Dans rqt, cliquer sur Plugins :**
choisir `Visualization` et cliquer sur `Image View`et enfin choisir le topic : `/camera/image_annotated` : pour afficher le flux vidéo de la camera avec les différentes informations comme la distance à l’objet détecté etc.

✅ **Dans `rviz`, cliquer sur `add` et choisir les topics :** `/marker` : pour visualiser les marques des motifs détectés sur la carte et `/path` : pour visualiser le chemin parcouru par le robot.

## 🗺️ Lancement de l’exploration et de la cartographie

✅ **Lancer le script connect_and_launch_frontier :**
```bash
./connect_and_launch_frontier.sh
```
Nb : On vous demandera de renseigner l’adresse IP d’un turtlebot pour s’y connecter.
