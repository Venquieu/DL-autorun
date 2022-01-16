#!/bin/bash

gpu_ids=$1
cmd=$2
nvidia-smi > info.txt
status=`python detector.py info.txt`

while ["$status" == "F"]
do
    sleep 60
    echo "busy now, keep waiting..."
    status=`python detector.py info.txt`
done
echo "finally! start running..."
rm info.txt

# run the command
$cmd