export USER_ID=$(id -u)
export GROUP_ID=1000
export ROS_DOMAIN_ID=1
xhost +
docker compose up
