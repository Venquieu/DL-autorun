#!/bin/bash

gpu_ids=$1
gpu_num=$2
memory_needs=$3
cmd=$4

nvidia-smi > info.txt
status=`python detector.py info.txt --gpu_ids $1 --gpu_nums $2 --memory_needs $3`

while ["$status" == "F"]
do
    sleep 60
    echo "busy now, keep waiting..."
    status=`python detector.py info.txt`
done


echo "finally! start running..."
rm info.txt

# run the command
if [ "$cmd" == "" ] 
then
    bash cmd.sh $status
else
    $cmd
fi