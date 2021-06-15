while getopts n: option 
do 
 case "${option}" 
 in
 n) NOME=${OPTARG};; 
 esac 
done 

if [ "$NOME" = "" ] ; then
  echo "Missing -n argument with id of drone"
  exit 1
fi

drone_enc=$NOME"_enc"
drone_dec=$NOME"_dec"

export ROS_DOMAIN_ID=128
source /opt/ros/dashing/setup.bash
cd ../x264_ws
colcon build --cmake-args '-DHWACC=1'
source install/setup.bash
ros2 run h264_utils h264viewer -n $drone_dec -c $drone_enc
