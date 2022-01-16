import numpy as np
import defaults
key_words = defaults.excel_key_words

def find_low_obstacle(data:np.array):
    row_idx = np.array([range(1, data.shape[0]+1)]).T
    data = np.hstack((data, row_idx))
    
    filter_data = np.empty(shape=[0, data.shape[1]])

    for row in data:
        flag = False
        for elem in row:
            for word in key_words:
                if word in str(elem):
                    flag = True
                    break
            if flag:
                filter_data = np.concatenate((filter_data, row[np.newaxis, :]), axis=0)
                flag = False
                break

    return filter_data
