import os
import time
import argparse

from detector import GpuProcesser, list_to_str
from utils import build_logger

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="auto-runner")
    parser.add_argument('--ids', type=int, nargs='+', help="the gpus you want to use")
    parser.add_argument('--nums', type=int, help="the number of gpus you want to use")
    parser.add_argument('--memory_needs', type=int, help="the memory you need")

    args = parser.parse_args()

    processer = GpuProcesser()

    logger = build_logger('runner:')
    logger.info('processer initialized, start monitoring...')
    
    while processer.update():
        if args.ids is not None and \
            processer.is_gpus_available(args.ids):
            break

        if args.nums is not None and \
            processer.is_n_gpu_available(args.nums):
            break

        time.sleep(30)

    ids = processer.query()
    if len(ids) == 0:
        exit()

    ids = list_to_str(ids)
    logger.info('find suitable device [{}], start running...'.format(ids))
    os.system('bash cmd.sh {}'.format(ids))
