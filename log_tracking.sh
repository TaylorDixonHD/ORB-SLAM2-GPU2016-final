#!/bin/bash


touch ./tracking_log.txt
count=1
while [ $count -lt 10 ]; do 
	date +%H:%M:%S >> ./tracking_log.txt
	curl -X POST localhost:18070/tracker/state >> ./tracking_log.txt
	sleep 5
done
