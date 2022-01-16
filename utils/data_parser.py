import numpy as np
import pandas as pd
import argparse

from rules import find_low_obstacle

def DataReader(file_path = 'systemtakeover_week47.xlsx',sheet_idx = 0):
    table = pd.read_excel(file_path,sheet_name=sheet_idx, header=None)
    return table.values

def Parser(command:str):
    if command.find('rosrun') == -1:
        return None, None
    com_ = command.split()
    return com_[3].split('/')[-1], com_[-1]
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Takeover data reader.")
    parser.add_argument(
        'file_path', help="path of the takeover excel file"
    )
    args = parser.parse_args()
    file_path = args.file_path # e.g. 'systemtakeover_week47.xlsx'

    excel_info = DataReader(file_path)
    filtered_info = find_low_obstacle(excel_info)
    commands = filtered_info[:, -2]

    for idx,com in zip(filtered_info[:, -1], commands):
        bag_name, stamp = Parser(com)
        if not bag_name is None:
            print(bag_name + '-' + stamp + '-' + str(idx))