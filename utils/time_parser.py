import argparse
import defaults
import pdb 

advance_time_post = defaults.time_parser_advance_cut_time
time_redundancy = defaults.time_parser_redundancy_time
min_duration_time = defaults.stationary_min_duration_time

def remove_short_period(stamps:list):
    for i in range(len(stamps)-1):
        if stamps[i+1] - stamps[i] < min_duration_time:
            stamps[i] = -1
            stamps[i+1] = -1
    while stamps.count(-1):
        stamps.remove(-1)
    return stamps

def stamp_to_second(baseline:tuple, stamp):
    '''
    Args:
        stamp: [stamp0, stamp1, stamp2, ...]
    Rets:
        [second0, second1, second2, ...]
    '''
    base_stamp, base_second = baseline[0], baseline[1]
    if isinstance(stamp, int):
        return stamp - base_stamp + base_second
    elif isinstance(stamp, list):
        for i in range(len(stamp)):
            stamp[i] = stamp[i] - base_stamp + base_second
        return stamp
    else:
        return None

def special_case_processing(takeover_time:int, time_periods:list):
    '''
    When the takeover time is very short
    '''
    if len(time_periods) % 2 == 1:
        time_periods.append(10000)
    for i in range(0, len(time_periods), 2):
        if time_periods[i] <= takeover_time <= time_periods[i+1] and takeover_time - time_periods[i] >= advance_time_post:
            time_periods[i] = takeover_time - advance_time_post
    if time_periods[-1] == 10000:
        time_periods.pop()
    return time_periods

def locate_takeover_period(takeover_time:int, time_periods:list):
    '''
    time_periods: [second0, second1, second2, ...]
    '''
    # if len(time_periods) <= 2:
    #     return time_periods
    min_interval = 600
    interval_thres = -5
    takeover_start_time_idx = None
    for i in range(1, len(time_periods), 2):
        interval = takeover_time - time_periods[i]
        if interval_thres <= interval <= min_interval:
            min_interval = interval
            takeover_start_time_idx = i
        elif interval < interval_thres:
            break

    if takeover_start_time_idx is None: # not find the takeover period
        return special_case_processing(takeover_time, time_periods)

    new_time_periods = [max(time_periods[takeover_start_time_idx-1], time_periods[takeover_start_time_idx] - advance_time_post)]
    for i in range(takeover_start_time_idx, len(time_periods)):
        new_time_periods.append(time_periods[i]) 
    new_time_periods = special_case_processing(takeover_time, new_time_periods)
    return new_time_periods

def add_redundancy(time_redundancy:int, time_periods:list):
    for i in range(1, len(time_periods)):
        if i%2 == 0:
            time_periods[i] -= time_redundancy
        else:
            time_periods[i] += time_redundancy
    
    for i in range(0, len(time_periods)-1):
        if time_periods[i] >= time_periods[i+1]:
            time_periods[i] = -1
            time_periods[i+1] = -1
    while time_periods.count(-1):
        time_periods.remove(-1)
    return time_periods

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Stationary time and moving time parser.")
    parser.add_argument(
        'file_path', help="path of the time period file"
    )
    parser.add_argument(
        'takeover_time', type=int, help="takeover time of the bag"
    )
    parser.add_argument(
        'start_time', type=int, help="original start time"
    )
    args = parser.parse_args()
    file_path = args.file_path # e.g. 'note.txt'
    takeover_time = args.takeover_time
    start_time = args.start_time

    with open(file_path, 'r') as f:
        file = f.read()

    file = file.split('\n')
    filter_space_file = []
    point_info = []

    for line in file:
        if line != '':
            filter_space_file.append(line)

    if len(filter_space_file) <= 1:
        exit()

    start_timestamp = int(filter_space_file[0])
    baseline = (start_timestamp, start_time)

    time_points = filter_space_file[-1].split()
    for i in range(len(time_points)):
        time_points[i] = int(time_points[i])
    # pdb.set_trace()
    remove_short_period(time_points)
    time_points = stamp_to_second(baseline=baseline, stamp=time_points)
    time_points = locate_takeover_period(takeover_time, time_points)
    time_points = add_redundancy(time_redundancy, time_points)


    for i in range(len(time_points)):
        if i%2 == 0:
            point_info.append(time_points[i]) # -s
        else:
            point_info.append(time_points[i] - time_points[i-1]) # -u

    if len(time_points)%2 == 1:
        point_info.append(-1)  # last -u

    # Output: [-s, -u, -s, -u, ...]     e.g. [191, 20, 242, 75]
    for i in range(len(point_info)):
        point_info[i] = str(point_info[i])
    print(' '.join(point_info))
