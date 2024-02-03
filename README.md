# DL-Autorun
This is a simple tool for automaticly run your programs(generally they are deep learning programs) when the GPUs are available.

## install
```bash
bash install.sh
```
Then you can restart or source your shell to make it work.
Here is how to source it:
```
# for bash shell
source ~/.bashrc

# for zsh shell
source ~/.zshrc
```

## usage
You can use the tool in following ways:

1. specify the number of GPUs you want:
```
monitor --nums 2 -f cmd.sh
monitor --nums 2 -c "python -m train"
```

2. specify the id of GPUs you want to use:
```
monitor --ids 4 5 6 7 -f cmd.sh
```

Note that the program will set environment variable `CUDA_VISIBLE_DEVICES` and pass available GPU ids and number to `cmd.sh` file. Then the file could accept them if you need it:
```
gpu_ids=$1
num_gpus=$2
```