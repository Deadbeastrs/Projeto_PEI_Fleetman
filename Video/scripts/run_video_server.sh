while getopts a:p: option 
do 
 case "${option}" 
 in 
 a) ADDR=${OPTARG};; 
 p) PORT=${OPTARG};; 
 esac 
done 

if [ "$ADDR" = "" ]  ||  [ "$PORT" = "" ] ; then
  echo "Missing -a argument for address or -p for port"
  exit 1
fi

export ROS_DOMAIN_ID=128
source /opt/ros/dashing/setup.bash
python3 ../server/main.py $ADDR $PORT


