#!/bin/bash

rclone copy . google_drive:/rob450-data --progress

if [ $? -eq 0 ]; then
    rm -f *.rrd *.log
else
    echo "Upload failed, keeping files"
    exit 1
fi
