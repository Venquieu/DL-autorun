import subprocess

class GpuProcesser(object):
    def __init__(self, gpu_info=None):
        self.UTIL_THRES = 0.2 # 20%

        self.gpu_status = gpu_info
        if gpu_info is None:
            self.update()

        self.num_gpus = len(self.gpu_map)
        self.free_gpus = []
        
    def parse(self, info=None):
        if info is None:
            info = self.gpu_status

        info = info.split('\n')
        assert len(info) > 10, 'Something wrong in the info file, it is too short'
        
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
    
    def is_gpus_available(self, ids):
        gpus = list(ids)
        assert max(gpus)<len(self.gpu_map)

        for gpu_id in gpus:
            if self.gpu_map[gpu_id][2] > self.UTIL_THRES:
                return False
        
        self.free_gpus = gpu_id
        return True

    def is_n_gpu_available(self, num):
        # firstly clear it
        self.free_gpus = []

        for gpu_id in range(self.num_gpus):
            if self.gpu_map[gpu_id][2] < self.UTIL_THRES:
                self.free_gpus.append(gpu_id)

            if len(self.free_gpus) == num:
                return True
        
        return False

    def has_memory(self, memory_needs:int):
        raise NotImplementedError

    def update(self):
        try:
            output = subprocess.run(
                ['nvidia-smi'],
                stdout=subprocess.PIPE
            )
            self.gpu_status = output.stdout.decode('utf-8')
            self.gpu_map = self.parse(self.gpu_status)
            return True
        except:
            print('error occured when running `nvidia-smi`')
            return False

    def query(self):
        return self.free_gpus

def list_to_str(ids: list):
    string = ''
    for id in ids:
        string += str(id) + ','

    return string.strip(',')