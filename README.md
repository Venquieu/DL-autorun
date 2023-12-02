# DL-Autorun
This is a simple tool for automaticly run your programs(generally they are deep learning programs) when the GPUs are available.

## install
```bash
bash install.sh

# for bash shell
source ~/.bashrc
# for zsh shell
source ~/.zshrc
```

## usage
You can use the tool in following way.

1. specify the number of GPUs you want:
```
monitor --nums 2 -f cmd.sh
monitor --nums 2 -c "python -m train"
```
Note that the program will set environment variable `CUDA_VISIBLE_DEVECES` and pass available GPU ids and number to `cmd.sh` file. Then the file could accept them if you need it:
```
gpu_ids=$1
num_gpus=$2
```