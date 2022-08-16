import os
import time
import argparse

from detector import GpuProcesser, list_to_str
from utils import build_logger

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="auto-runner")
    parser.add_argument('--ids', type=int, nargs='+', help="the gpus you want to use")
    parser.add_argument('--nums', type=int, help="the number of gpus you want to use")
    parser.add_argument('--mem', type=int, help="the memory you need")
    parser.add_argument('--thres', default=0.1, type=float, 
        help="memory threshold to judge if a card is free"
    )
    parser.add_argument('--sleep', default=30, type=int, 
        help="sleep time in loop"
    )

    args = parser.parse_args()

    processer = GpuProcesser(args.thres)

    logger = build_logger('runner:')
    logger.info('processer initialized, start monitoring...')
    
    while processer.update():
        if (
            args.ids is not None 
            and processer.gpus_available(args.ids)
        ):
            break

        if (
            args.nums is not None 
            and processer.n_gpus_available(args.nums)
        ):
            break

        if (
            args.mem is not None
            and processer.has_memory(args.mem)
        ):
            break

        time.sleep(args.sleep)

    ids = processer.query()
    if len(ids) == 0:
        exit()

    ids = list_to_str(ids)
    logger.info('find suitable device [{}], start running...'.format(ids))
    os.system('bash cmd.sh {}'.format(ids))
