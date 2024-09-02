#!/bin/bash

TARGET_DIR="/home/cvl/ros_whill_ws/path_estimation/csv_toward"

for file in "$TARGET_DIR"/*.csv; do
    if [ -f "$file" ]; then
        > "$file"
        echo "Emptied $file"
    fi
done

echo "ALL CSV FILES IN $TARGET_DIR HAVE BEEN EMPTIED."
