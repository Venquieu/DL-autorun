#!/bin/bash

export CUDA_VISIBLE_DEVECES=$1
num_gpus=$2

# write your command here, e.g.,
cd ~/my_project
python train.py