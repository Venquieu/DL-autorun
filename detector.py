import subprocess
from copy import deepcopy

class GpuProcesser(object):
    def __init__(self, threshold):
        self.UTIL_THRES = threshold
        self.update()

        self.num_gpus = len(self.gpu_map)
        self.free_gpus = []
        
    def parse(self, info=None):
        if info is None:
            info = self.gpu_status

        info = info.split('\n')
        assert len(info) > 10, \
            'Something wrong in the info file, it is too short'
        
        smi_info = []
        for i in range(8, len(info)):
            if info[i].startswith(' '):
                break
            smi_info.append(info[i])

        info_map = []
        for id, smi_info_ in enumerate(smi_info[1::4]):
            mem_info = smi_info_.split('|')[2].strip()
            mem_info = mem_info.split('/')

            used_memory = int(mem_info[0].strip()[:-3])
            total_memory = int(mem_info[1].strip()[:-3])
            info_map.append(
                {
                    'id': id,
                    'left': total_memory - used_memory,
                    'total': total_memory,
                    'util': used_memory/total_memory
                }
            )
        return info_map
    
    def gpus_available(self, ids):
        gpus = list(ids)
        assert max(gpus) < self.num_gpus

        for gpu_id in gpus:
            if self.gpu_map[gpu_id]['util'] > self.UTIL_THRES:
                return False
        
        self.free_gpus = gpu_id
        return True

    def n_gpus_available(self, num):
        # firstly clear it
        self.free_gpus = []

        for gpu_id in range(self.num_gpus):
            if self.gpu_map[gpu_id]['util'] < self.UTIL_THRES:
                self.free_gpus.append(gpu_id)

            if len(self.free_gpus) == num:
                return True
        
        return False

    def has_memory(self, memory:int):
        print('will search no more than 2 cards')
        max_left = [
            {'id': -1, 'left': 0},
            {'id': -1, 'left': 0}
        ]
        for item in self.gpu_map:
            if item['left'] > max_left[0]['left']:
                max_left[1] = deepcopy(max_left[0])

                max_left[0]['id'] = item['id']
                max_left[0]['left'] = item['left']
                
            elif item['left'] > max_left[1]['left']:
                max_left[1]['id'] = item['id']
                max_left[1]['left'] = item['left']

        if max_left[0]['left'] > memory:
            self.free_gpus = max_left[0]['id']
            return True
        if max_left[0]['left'] + max_left[1]['left'] > memory:
            self.free_gpus = [max_left[0]['id'], max_left[1]['id']]
            return True

        self.free_gpus = []
        return False

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