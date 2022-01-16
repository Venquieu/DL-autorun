#!/bin/bash
# set -x

files=`python utils/data_parser.py $1`
const_advance_time=$2
const_duration_time=$3

week=$(echo "$1" | awk -F"." '{print $1}')
week=$(echo "$week" | awk -F"_" '{print $2}')
bagfiles="bagfiles_$week"
imgfiles="imgfiles_$week"

mkdir $bagfiles
mkdir $imgfiles
# in case there is a `rosbag record` running
kill -SIGINT `ps -ef | grep "lib/rosbag/record" | grep -v "grep" | awk '{print $2}'`

# ------topics for `rosrun aw_bag play` ------
com_topic="/aw_cam00/image_raw/compressed \
            /aw_cam03/image_raw/compressed \
            /aw/localized_pose \
            /aw/unexpected_objects_segmentation \
            /aw/planning_info \
            /aw/static_planning_info"
# ------   topics for `rosbag record`   ------
save_topic="$com_topic /aw/uosegmentation_visualization/compressed"
rosrun aw_bag change_nn

for row in ${files[@]}; do
    bag_name=$(echo "$row" | awk -F"-" '{print $1}')
    takeover_time=$(echo "$row" | awk -F"-" '{print $2}')
    row_index=$(echo "$row" | awk -F"-" '{print $3}')

    export robot_type=`echo $bag_name | awk -F"_" '{print $7}'`
    export robot_id=`echo "$bag_name" | awk -F"_" '{print $8}'`
    advance_time=$const_advance_time
    duration_time=$const_duration_time
    
    
    start_time=`expr $takeover_time - $advance_time`
    if [ $start_time -lt 0 ]; then
        duration_time=`expr $duration_time + $start_time`
        start_time=0
    fi

    # Locate useful time period
    python utils/stationary_detector.py > time_period.txt&
    rosrun aw_bag play $bag_name --topics /aw/localized_pose /aw_cam03/image_raw/compressed -s $start_time -u $duration_time
    kill -SIGINT `ps -ef | grep "stationary_detector.py" | grep -v "grep" | awk '{print $2}'`
    sleep 2
    time_period=`python utils/time_parser.py time_period.txt $takeover_time $start_time`
    parsed_start_time=$(echo "$time_period" | awk -F" " '{print $1}')
    save_file_name="$row_index"_"$bag_name"_"$parsed_start_time"
    rm time_period.txt

    # run necessary process
    roslaunch aw_uosegmentation aw_uosegmentation_visualization.launch --screen& # T2
    cd ./$bagfiles
    rosbag record -O $save_file_name.bag $save_topic&  # T3
    cd ..

    cd ./$imgfiles
    mkdir $save_file_name
    cd $save_file_name
    python ../../utils/image_saver.py $bag_name&
    cd ../..

    # play bag
    row_flag="0"
    for time_elem in $time_period
    do
        if [ "$row_flag" == "0" ]; then
            single_start_time=$time_elem
            row_flag="1"
            continue
        fi
        row_flag="0"
        single_duration_time=$time_elem
        if [ $single_duration_time -lt 0 ]; then
            single_duration_time=`expr $duration_time - $single_start_time + $start_time`
        fi
        rosrun aw_bag play $bag_name --topics $com_topic -s $single_start_time -u $single_duration_time
        sleep 1
    sleep 1
    done
    sleep 2
    
    # shut down thread
    kill -SIGINT `ps -ef | grep "lib/rosbag/record" | grep -v "grep" | awk '{print $2}'`
    kill -SIGINT `ps -ef | grep "roslaunch" | grep -v "grep" | awk '{print $2}'`
    kill -SIGINT `ps -ef | grep "image_saver.py" | grep -v "grep" | awk '{print $2}'`
done
sleep 2


cd ./$bagfiles
for file in `ls`
do
    file_size=`du -b $file | awk '{print $1}'`
    if [ $file_size -lt 5000000 ]; then  # ~5M
            mv $file "NoSignal_$file"
            echo "Renaming bag $file"
    fi
done
cd ..


echo "=====Saving process finished====="
