#!/bin/bash

# Reload IPv6 networking blocks
sudo sysctl -p

sudo rfkill block wifi  
sudo rfkill block bluetooth

. /home/ubuntu/2018Offseason/zebROS_ws/ROSJetsonMaster.sh
#echo 1100-1200,443,80,554,1735 > /proc/sys/net/ipv4/ip_local_reserved_ports

#echo 5800 5810 > /proc/sys/net/ipv4/ip_local_port_range
#systemctl restart networking

sudo chmod a+rw /dev/ttyACM0
#sudo python /home/ubuntu/2017VisionCode/time_sync_server.py & 
sudo umount /mnt/900_2 --lazy

export CUDA_CACHE_MAXSIZE=104857600
export CUDA_CACHE_PATH=/home/ubuntu/.nv/ComputeCache

if sudo mount /dev/nvme0n1p1 /mnt/900_2; then
		date >> /home/ubuntu/mounted.txt
		echo worked >> /home/ubuntu/mounted.txt
		sudo chmod a+rw /mnt/900_2/
		roslaunch controller_node 2018_jetson_teleop.launch output:=screen record:=true
else
		date >> /home/ubuntu/mounted.txt
		echo did not mount >> /home/ubuntu/mounted.txt
		roslaunch controller_node jetson_teleop.launch output:=screen
fi

top -b > /mnt/900_2/$(date +%Y%m%d%H%M%S)_top_log.txt

nvpmodel -m 0
/home/ubuntu/jetson_clocks.sh
/home/ubuntu/2018Offseason/jetson_setup/clocks.sh &

