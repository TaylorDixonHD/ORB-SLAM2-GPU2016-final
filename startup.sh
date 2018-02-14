#!/bin/bash


case "$1" in
  start)
    /home/ubuntu/ORB-SLAM2-GPU2016-final/build/gpu/tx1
    ;;
  stop)
    echo "stopping slam service"
    ;;
  *)
    echo "Usage /startup.sh {start|stop}"
    exit 1
    ;;
esac

exit 0
