#!/bin/bash
roscore &
sleep 5 # wait for roscore to start up
/home/dylan/Desktop/noeticws/build/vision/yolo-trt &
python3 /home/dylan/Desktop/noeticws/src/navigation/src/navigation.py &
python3 /home/dylan/Desktop/noeticws/src/reacher/src/reacher.py

