# key words for filter the items in the takeover excel
excel_key_words = ('低矮', '不存在', '水管', '电线', '垃圾', '树枝', '石头', '纸盒', '扫把', '除草机', '石块', '树叶', '袋子', '坑', '绳子')

stationary_speed_thres = 0.1
mobile_speed_thres = 0.2
stationary_contiguous_length = 100
# the least duration time(second) of stationary and mobile status
stationary_min_duration_time = 6

image_save_interval = 0.5 #second

# If have located the period that trigger the takeover, 
# then only use `time_parser_advance_cut_time` ahead time of the stationary status.
time_parser_advance_cut_time = 30
# time redundancy
time_parser_redundancy_time = 3