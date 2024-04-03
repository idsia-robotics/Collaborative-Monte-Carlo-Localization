#!/bin/bash


#find . -name "*.csv" -type f -delete


declare -a DIST
DIST[0]="-1"
DIST[1]="1"
DIST[2]="3"
DIST[3]="2"
DIST[4]="4"
DIST[5]="0"
DIST[6]="0"

declare -a DETECT
DETECT[0]="false"
DETECT[1]="true"
DETECT[2]="true"
DETECT[3]="true"
DETECT[4]="true"
DETECT[5]="true"
DETECT[6]="true"

declare -a ALGO
ALGO[0]="MCL"
ALGO[1]="FOX_CMCL"
ALGO[2]="PROROK_CMCL"
ALGO[3]="KMEANS_CMCL"
ALGO[4]="GOODPOINTS_CMCL"
ALGO[5]="THIN_CMCL"
ALGO[6]="NAIVE_CMCL"

declare -a MAP
MAP[0]="Env1"
MAP[1]="Env2"
MAP[2]="Env3"


particleNum=10000
path1="/ros_ws/dump/"
rosbagDir="/ros_ws/rosbags/"
seq="demo"
m=2
mapName=${MAP[ $m ]}

source /ros_ws/install/setup.bash

a=4
run=0
detect=${DETECT[ $a ]}
dist=${DIST[ $a ]}
algo=${ALGO[ $a ]}
csv="$path1/$algo/$mapName/S$s/" 
echo $csv


ros2 launch cmcl_ros cmcl_real.launch mapName:="$mapName" detection:="$detect" distType:="$dist" csvPath:="$csv" run:="$run" &
sleep 5
ros2 lifecycle set /map_server activate
rosbagPath="$rosbagDir/$seq/"
echo $rosbagPath
ros2 bag play $rosbagPath

kill $(pgrep -f rviz2)
kill $(pgrep -f DetectionNoiseNode)
kill $(pgrep -f MCLNode)
kill $(pgrep -f LidarScan2PointCloudNode)
kill $(pgrep -f EvalNode)
kill $(pgrep -f map_server)
kill $(pgrep -f UVDetection2WorldNode)
kill $(pgrep -f RelativePosedNode)
kill $(pgrep -f cmcl.launch)
kill $(pgrep -f OprtitrackPublisherNode)
pkill -9 ros2
ros2 daemon stop

