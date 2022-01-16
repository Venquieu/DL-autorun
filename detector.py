import argparse

def info_parser(gpu_info:str)->list:
    info = gpu_info.split('\n')
    assert len(info) > 10, 'Something wrong in the info file, it\'s too short'
    smi_info, pid_info = [], [] 
    flag = 0
    for i in range(8, len(info)):
        if info[i].startswith(' '):
            flag = 1
        if flag == 0:
            smi_info.append(info[i])
        else:
            pid_info.append(info[i])

    mem_info = smi_info[1::4]
    for id in range(len(mem_info)):
        info = mem_info[id].split('|')[2].strip()
        info = info.split('/')
        used_memory, total_memory = int(info[0].strip()[:-3]), int(info[1].strip()[:-3])
        mem_info[id] = (used_memory, total_memory, used_memory/total_memory)
    return mem_info


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="detector")
    parser.add_argument(
        'file_path', help="path of the nvidia-smi output file"
    )
    parser.add_argument(
        '--gpu_ids', type=int, nargs='+', default=[0], help="the gpus you want to use"
    )
    parser.add_argument(
        '--gpu_nums', type=int, default=1, help="the number of gpus you want to use"
    )
    parser.add_argument(
        '--memory_needs', type=int, help="the memory you need"
    )
    args = parser.parse_args()

    with open(args.file_path, 'r') as f:
        file = f.read()

    mem_info = info_parser(file)

    for id in args.gpu_ids:
        if mem_info[id][0] > 20:
            print('F')
            exit()
    print('T')
