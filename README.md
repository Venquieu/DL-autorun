# DL-Autorun
This is a simple tool for automaticly run your programs(generally they are deep learning programs) when the GPUs are available.
## usage
You can use the tool in following way.

1. specify the number of GPUs you want:
```
python runner.py --nums 2 -f cmd.sh
python runner.py --nums 2 -c "python -m train"
```
Note that the program will set environment variable `CUDA_VISIBLE_DEVECES` and pass available GPU ids and number to `cmd.sh` file. Then the file could accept them if you need it:
```
gpu_ids=$1
num_gpus=$2
```