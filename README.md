# DL-Autorun
This is a simple tool for automaticly run your programs(generally they are deep learning programs) when the GPUs are available.
## usage
You can use the tool in following way.

1. specify the number of GPUs you want:
```
python runner.py --nums 2 -f cmd.sh
python runner.py --nums 2 -c "python -m train"
```