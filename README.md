# IMU Fusion

Package ROS2 qui fusionne les données d'accéléromètre et de gyroscope publiées par une caméra [RealSense D435i](https://www.intelrealsense.com/depth-camera-d435i/) pour produire un message IMU unique.

## Architecture Système

```
        ┌──────────────────────────────┐
        │       Caméra RealSense       │
        │                              │
        │ ┌─────────────────┐          │
        │ │  Accéléromètre  │ ──► (1)  │
        │ └─────────────────┘          │
        │ ┌─────────────────┐          │
        │ │    Gyroscope    │ ──► (2)  │
        │ └─────────────────┘          │
        └──────────────────────────────┘
                       │
                       ▼
        ┌──────────────────────────────┐
        │      Noeud IMU Fusion        │
        │                              │    ◄── Vous êtes ici
        │    Publie sur /camera/imu    │
        └──────────────────────────────┘
                       │
                       ▼
        ┌──────────────────────────────┐
        │         SLAM Toolbox         │
        │   (Mapping & Localisation)   │
        └──────────────────────────────┘
                       │
                       ▼
        ┌──────────────────────────────┐
        │             Nav2             │
        │  (Planification & Control)   │
        └──────────────────────────────┘
                       │
                       ▼
        ┌──────────────────────────────┐
        │     Contrôleur du robot      │
        └──────────────────────────────┘
```

## Fonctionnalités

- **Synchronisation :**  
  Les messages d'accéléromètre et de gyroscope sont synchronisés avec une tolérance de 0.1 seconde grâce à l'ApproximateTimeSynchronizer.

- **Fusion des données :**  
  Les mesures linéaires de l'accéléromètre et les vitesses angulaires du gyroscope sont fusionnées dans un seul message IMU.  
  - L'horodatage est calculé comme la moyenne des timestamps des deux capteurs.
  - Le `frame_id` de l'accéléromètre est utilisé comme référence.

- **QoS BEST_EFFORT :**  
  Utilisation d'un profil QoS BEST_EFFORT pour correspondre aux caractéristiques des données publiées par la caméra RealSense.

## Utilisation

Pour lancer le noeud de fusion IMU, la commande suivante doit être éxécutée :

```bash
ros2 launch imu_fusion imu_fusion.launch.py
```

Le noeud s'abonne aux topics :
- `/camera/camera/accel/sample` pour l'accéléromètre
- `/camera/camera/gyro/sample` pour le gyroscope

Il publie ensuite un message fusionné sur le topic :
- `/camera/imu`
