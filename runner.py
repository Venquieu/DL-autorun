import os
import time
import argparse

from detector import GpuProcesser
from utils import build_logger
logger = build_logger('runner')

class Monitor(object):
    def __init__(self, items):
        self.items = items

    def loop(self):
        while True:
            for arg, func in self.items:
                if func(arg): return True
            time.sleep(60)

    @staticmethod
    def do_nothing(arg):
        return False

    @staticmethod
    def enable_if(arg, func):
        if arg is not None:
            return arg, func
        return  arg, Monitor.do_nothing


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="auto-runner")
    parser.add_argument('--gpu_ids', type=int, nargs='+', help="the gpus you want to use")
    parser.add_argument('--gpu_nums', type=int, help="the number of gpus you want to use")
    parser.add_argument('--memory_needs', type=int, help="the memory you need")

    args = parser.parse_args()

    nvidiasmi_info = os.system('nvidia-smi') 
    processer = GpuProcesser(nvidiasmi_info)

    monitor = Monitor([
        Monitor.enable_if(args.gpu_ids, processer.is_gpus_available),
        Monitor.enable_if(args.gpu_nums, processer.is_n_gpu_available)
    ])
    monitor.loop()

    logger.info('start running...')
    os.system('bash cmd.sh')
