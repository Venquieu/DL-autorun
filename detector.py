import argparse

MEM_THRES = 20 # MB

def mem_info_parser(gpu_info:str)->list:
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

def list2str(lst:list)->str:
    for i in range(len(lst)):
        lst[i] = str(lst[i])
    return ' '.join(lst)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="detector")
    
    parser.add_argument('file_path', help="path of the nvidia-smi output file")
    parser.add_argument('--gpu_ids', type=int, nargs='+', default=[0], help="the gpus you want to use")
    parser.add_argument('--gpu_nums', type=int, default=1, help="the number of gpus you want to use")
    parser.add_argument('--memory_needs', type=int, help="the memory you need")

    args = parser.parse_args()

    with open(args.file_path, 'r') as f:
        file = f.read()

    mem_info = mem_info_parser(file)

    avaliable_gpus = []
    for id,mem in enumerate(mem_info):
        if mem[0] <= MEM_THRES:
            avaliable_gpus.append(id)

    if args.gpu_nums is not None:
        count = len(avaliable_gpus)
        if count >= args.gpu_nums:
            print(list2str(avaliable_gpus))
        else:
            print('F')
        exit()
    if args.gpu_ids is not None:
        for id in args.gpu_ids:
            if id not in avaliable_gpus:
                print('F')
                exit()
        print(list2str(avaliable_gpus))
    
    else:
        print('F')
