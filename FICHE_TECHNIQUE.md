## les packages :

**Ce depôt a deux packages** :

•`detection` qui gère la detection et le marquages des motifs détectés sur la carte.

•`frontier_exploration` qui gére la navigation autonome du robot et de la cartographie de l'arène.

## les noeuds dans le package `detection` :

dans ce package nous avons 
➔ objectrecognition
Rôle : Ce nœud est conçu pour effectuer la détection d'objets dans les images en utilisant un modèle pré-entraîné YOLO (You Only Look Once). Lorsqu'un objet est détecté avec une confiance supérieure à un seuil défini, il dessine un rectangle autour de l'objet et republie l'image annotée.

Entrées :

Image (de type sensor_msgs/Image) via le topic /camera/image (provient d'une caméra ou d'un nœud de traitement d'image).

Sorties :

Image annotée avec des rectangles autour des objets détectés (de type sensor_msgs/Image), publiée sur le topic /camera/image_squared.

➔ turtlebot_distance_estimator
Rôle : Ce nœud calcule la distance d'un objet (carré vert) à partir de l'image capturée par la caméra, en utilisant des informations de la caméra et de l'odométrie du robot. Il publie la position estimée de l'objet dans le repère global du robot.

Entrées :

Image de la caméra (sensor_msgs/Image) via le topic /camera/image_squared.

Odométrie du robot (nav_msgs/Odometry) via le topic /odom.

Sorties :

Image annotée (avec les informations de distance et d'angle) publiée sur le topic /camera/image_annotated.

Position estimée de l'objet (geometry_msgs/PoseStamped) publiée sur le topic /marker.

➔ trace1
Rôle : Ce nœud publie la trajectoire du robot (odométrie) et ajoute des marqueurs visuels (croix rouges) aux positions parcourues par le robot, évitant de marquer les mêmes positions plusieurs fois.

Entrées :

Odométrie (nav_msgs/Odometry) via le topic /odom.

Position du marqueur (geometry_msgs/PoseStamped) via le topic /marker.

Sorties :

Trajectoire mise à jour (nav_msgs/Path) publiée sur le topic /path.

Marqueur visuel (visualization_msgs/Marker) publié sur le topic /visualization_marker.

➔ `traitement_image` : Ce nœud traite les images reçues depuis la caméra en les convertissant en niveaux de gris, puis en appliquant une égalisation d'histogramme adaptative (CLAHE) pour améliorer le contraste. Il republie ensuite l'image traitée.

Entrées : Image de la caméra (`sensor_msgs/Image`) via le topic `/camera/image`.

Sorties : Image traitée (`sensor_msgs/Image`) publiée sur le topic `/camera/image_processed`..

➔ `qrcode` : Ce nœud est conçu pour détecter les QR codes dans les images traitées par le nœud traitement_image. Lorsqu'un QR code est détecté, il envoie un message contenant le statut de la détection et affiche une image annotée du QR code détecté.

Entrées : Image traitée (`sensor_msgs/Image`) via le topic `/camera/image_processed` (provient du nœud traitement_image).

Sorties : Statut de la détection de QR code (`std_msgs/String`) publié sur le topic `/qr_status` (valeurs possibles : "QR code detected" ou "No QR code detected").

➔ `ra2` : Ce nœud intègre un objet 3D (par exemple, une boîte) dans un flux d'image en temps réel, projetant cet objet sur l'image capturée par la caméra du robot. Il utilise la calibration de la caméra pour effectuer cette projection.

Entrées : Image traitée provenant de la caméra du robot (type : `sensor_msgs/Image`) via le topic `/camera/image_processed`.

Sorties : Image avec l'objet 3D projeté dessus (type : `sensor_msgs/Image`) publiée sur le topic `/image_with_3d_object`.

## les noeuds dans le package `frontier_exploration` :

Pour la navigation, nous avons choisi d’utiliser l’algorithme frontier_exploration. Dans cette algorithme, nous rencontrons les noeuds suivant :

➔	`explore`: qui permet au robot d’explorer un environnement inconnu en detectant et en atteignant les forntières à l’aide du costmap_2d et du planificateur move_base.
Les services utilisés : ( services intégré à l’environnement ROS)

●	`/start` → démarre l’exploration (réactive le timer).

●	`/abort` → arrête l’exploration (annule les objectifs et stoppe le timer).

➔	`frontier_search` : qui permet de detecter des forntières dans la carte locale du robot. Les frontières représentent les limites entre les zones connues et inconnues par le robot. 

Aucun E/S. Il complete une liste de frontier qui sera directement appelé dans le noeud explore.

➔	`costmap_2d` :  qui gere un costmap_2d dans le système robotique. Le `costmap` represente une carte de l’environnement du robot où différents valeurs indiquent les coûts de deplacement de chaque cellule de la carte.
Il s’abonne au topic `/costmap` pour recevoir les mises à jour de la carte.

➔	`node` : il initialise le noeud `explore`.

