# Cyclops
Ca voit tout, mais avec un seul oeuil (camera)

# Installation
Utiliser Debian ou une de ses fork

Si tu veux installer OpenCV avec CUDA, voici le lien pour installer Cuda (J'utilise pas CUDA ni CUDNN pour l'instant, vu que les jetson sont underpowered compare aux Framework)

https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#introduction

Pour les neural networks, il faut aussi CuDNN.

Script une commande : `./DownloadRepos.sh` 

En fait ce script lance le script `./InstallRequirement.sh` qui installe toutes les libs nécessaires, 
puis télécharge les repos OpenCV, et enfin lance `./IntallOpenCV.sh`, script qui compile et installe OpenCV avec les bons parametres.

Si tu veux utiliser Cuda ou CuDNN, il faudra faire `./InstallOpenCV.sh 1` 

# Coder dessus
Je recommande VisualStudioCode

Avec l'extension CMake, faire un Clean Reconfigure All Projects à chaque fois que un .h et/ou un .cpp est ajouté.

Ci dessous je décris tous les modules de mon code.

## assets
L'endroit où se trouvent les assets utilisés par la visualisation.

Pour exporter un obj de Blender, choisir les paramètres suivants :

Y forward

Z up

UV, Vertex colors, triangulate faces


## calibration
Pour effectuer la calibration, lancer l'executable depuis /build `./cyclops -c`. 

Pour calibrer, il faut prendre en photo un echiquier. Les paremetres de l'echiquier sont regles dans le fichier config.json, qui est cree automatiquement.

Une fois la calibration faite, si t'en es content, déplacer le fichier de calibration de /build vers ce fichier. Il aura le nom de la caméra.

Les paramètres de la calibration sont dans config.cfg. La calibration essaie de garder un maximum d'images avec le minimum d'erreur de reprojection et peut prendre un poil de temps à s'executer.

## sim
Dossier où stocker les simulations software in the loop

## thirdparty
contient les submodules tels que imgui ou nlohmannjson

## include
les includes

## source
toutes les sources 

# Lancer le code

L'exécutable se trouvera dans le fichier build. Il peut se lancer depuis n'importe ou, il retrouvera son chemin.

`./cyclops -h` pour avoir l'aide

# Jetson (pas supporte)

Surtout si c'est dans le cas où les caméras sont connectées en MIPI-CSI, des fois le daemon plante : éxecuter
`systemctl restart nvargus-daemon.service`

`jtop` : comme htop (un gestionnaire de taches) mais pour jetson, c'est un peu mieux

# Commandes utiles

`v4l2-ctl --list-devices` : liste les caméras et leurs interfaces. V4L2 est le module linux aui s'occupe des caméras et autres sources vidéo.

`v4l2-ctl -d<n> --list-formats-ext` : liste les formats supportés par la caméra à l'interface `/dev/video<n>`

`htop` : utilisation du cpu

`nvtop` : utilisation de la carte graphique nvidia

`gst-inspect-1.0` : liste les modules gstreamer disponibles. GStreamer est un système d'ouverture de flux vidéo avec des modules

`gst-launch-1.0` : permet de lancer une pipeline gstreamer pour tester.