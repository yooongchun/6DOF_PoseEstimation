#!/bin/bash

    for file in ` ls ./pointcloud_txt `    
    do    
        echo ${file}
	./build/GeneratePointCloudFromPositionData ./pointcloud_txt/${file} ./pointcloud/${file##*_}.pcd
    done    

