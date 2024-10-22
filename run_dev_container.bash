export USER_ID=$(id -u)
export GROUP_ID=$(id -g)
export ROS_DOMAIN_ID=1
xhost +
docker compose up
