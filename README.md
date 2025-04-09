## 🚀 Lancement de la détection

✅ **Lancer le launch `projet.launch` du package `detection`:**
```bash
roslaunch detection projet.launch
```
**Dans rqt, cliquer sur Plugins :**
choisir `Visualization` et cliquer sur `Image` View et enfin choisir le topic : `/camera/image_annotated` : pour afficher le flux vidéo de la camera avec les différentes informations comme la distance à l’objet détecté etc.
**Dans `rviz`, cliquer sur `add` et choisir les topics :**
•	`/marker` : pour visualiser les marques des motifs détectés sur la carte.
•	`/path` : pour visualiser le chemin parcouru par le robot.

