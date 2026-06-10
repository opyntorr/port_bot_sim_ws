# Entorno para la sim local de Isaac Sim 4.5 (port del JetAuto AGV).
# Uso:  source isaac/isaac_env.sh   (en CADA terminal: la de Isaac y la del cerebro/RViz)
#
# Isaac hereda este dominio del .bashrc; lo fijamos explícito para que el cerebro
# (RViz, teleop, SLAM, etc.) y la sim se VEAN en el mismo dominio (sim local, sin robot real).
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# CycloneDDS solo-LOCAL (loopback unicast): Isaac y el cerebro estan en la MISMA
# laptop. Con el default, cyclonedds elige interfaces distintas (WiFi/docker/...) y
# NO se descubren entre si; este config los fuerza a loopback. NO uses el peer del
# robot (10.42.0.1) del .bashrc aqui.
export CYCLONEDDS_URI="file:///home/opyntorr/agv_uav_project_jetauto/isaac/cyclonedds_local.xml"
export ISAACSIM=/home/opyntorr/isaacsim
echo "[isaac_env] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION  CYCLONEDDS_URI=loopback  (use_sim_time:=true en los nodos del cerebro)"
