## les packages :

**Ce depôt a deux packages** :

•`detection` qui gère la detection et le marquages des motifs détectés sur la carte.

•`frontier_exploration` qui gére la navigation autonome du robot et de la cartographie de l'arène.

## les noeuds dans le package `detection` :

dans ce package nous avons 

`objectrecognition` :

Rôle : Reconnaît et détecte des objets à partir des images capturées par le robot.

Entrées :

Image du robot (type : sensor_msgs/Image).

Sorties :

Boîtes englobantes des objets détectés (type : sensor_msgs/PointCloud2 ou autre format adapté).

2. trace1
Rôle : Trace la trajectoire du robot et marque la position d’un objet détecté.

Entrées :

Position de l'objet détecté (type : geometry_msgs/PointStamped via /point_rouge).

Trajectoire du robot (type : nav_msgs/Odometry ou similaire).

Sorties :

Trajectoire du robot sur la carte.

Position de l'objet marqué sur la carte.

3. turtlebot_distance_estimator
Rôle : Estime la distance à un objet détecté et publie sa position.

Entrées :

Image de profondeur (type : sensor_msgs/Image).

Coordonnées de l'objet dans l’image (type : sensor_msgs/PointCloud2 ou similaire).

Sorties :

Position de l’objet dans l’espace réel (type : geometry_msgs/PointStamped via /point_rouge).

4. ra2
Rôle : Non précisé dans les informations précédentes. Besoin de détails pour ce nœud.

Entrées :

Pas d'informations spécifiques fournies.

Sorties :

Pas d'informations spécifiques fournies.

5. qrcode
Rôle : Détecte et traite les QR codes dans l’environnement du robot.

Entrées :

Image du robot ou capture vidéo (type : sensor_msgs/Image).

## les noeuds dans le package `frontier_exploration` :

Pour la navigation, nous avons choisi d’utiliser l’algorithme frontier_exploration. Dans cette algorithme, nous rencontrons les noeuds suivant :

➔	`explore`: qui permet au robot d’explorer un environnement inconnu en detectant et en atteignant les forntières à l’aide du costmap_2d et du planificateur move_base.
Les services utilisés : ( services intégré à l’environnement ROS)

●	`/start` → démarre l’exploration (réactive le timer).
●	`/abort` → arrête l’exploration (annule les objectifs et stoppe le timer).

➔	`frontier_search` : qui permet de detecter des forntières dans la carte locale du robot. Les frontières représentent les limites entre les zones connues et inconnues par le robot. Aucun E/S.
Il complete une liste de frontier qui sera directement appelé dans le noeud explore.

➔	`costmap_2d` :  qui gere un costmap_2d dans le système robotique. Le costmap represente une carte de l’environnement du robot où différents valeurs indiquent les coùts de deplacement de chaque cellule de la carte.
Il s’abonne au topic costmap pour recevoir les mises à jour de la carte.

➔	`node` : il initialise le noeud explore.

