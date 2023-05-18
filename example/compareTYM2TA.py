#!/usr/bin/env python3
import yaml
import matplotlib
# matplotlib.use("Agg")
from matplotlib.patches import Circle, Rectangle, Arrow
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
import matplotlib.animation as manimation
import argparse
import math
import os
import glob
import json
from copy import deepcopy as dp
import subprocess
import uuid
from faker import Faker

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--TYMPath", help="tym c++ program path")
    parser.add_argument("--CBSTAPath", help="cbs-ta program path")
    parser.add_argument("--map_dir", help="input file containing map")
    parser.add_argument("--seed", help="input file containing map")
    parser.add_argument("--time", help="input file containing map")
    args = parser.parse_args()
    args.time = int(args.time)
    f1 = Faker()
    Faker.seed(args.seed)

    map_file_paths = []
    if os.path.isdir(args.map_dir):
        map_file_paths = [f for f in glob.glob(args.map_dir + "/*.yaml")]
    else:
        map_file_paths.append(args.map_dir)
    map_file_paths = sorted(map_file_paths)

    def check(x_path):
        ignore_files = ["agents100", "agents90", "agents80", "agents70", "agents60", "agents50"]
        for ignore_file in ignore_files:
            if ignore_file in x_path:
                return True
        return False

    for map_file_path in map_file_paths:
        if check(map_file_path): continue
        output_name = "tmp_file_first" + args.seed
        run_args = ["./" + args.TYMPath, "-i", map_file_path, "-o", output_name]
        time_out = False
        try:
            subprocess.call(run_args, timeout=args.time, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        except subprocess.TimeoutExpired:
            time_out = True
        if not time_out:
            with open(output_name) as output_file:
                result = yaml.safe_load(output_file)
            tym_time = result["statistics"]['runtime']
            try:
                tym_ta_runtime = result["statistics"]['TA_runtime']
                tym_lowlevel_search_time = result["statistics"]['lowlevel_search_time']
            except :
                tym_ta_runtime = 0
                tym_lowlevel_search_time = 0
            tym_cost = result["statistics"]['cost']
            tym_ta_times = result["statistics"]['numTaskAssignments']
        else :
            tym_cost = "tym_Time_out"
            tym_time = "tym_Time_out"
            tym_ta_runtime = "tym_Time_out"
            tym_lowlevel_search_time = "tym_Time_out"
            tym_ta_times = "tym_Time_out"

        output_name = "tmp_file_second" + args.seed
        run_args = ["./" + args.CBSTAPath, "-i", map_file_path, "-o", output_name]
        time_out = False
        try:
            subprocess.call(run_args, timeout=args.time, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        except subprocess.TimeoutExpired:
            time_out = True
        if not time_out:
            with open(output_name) as output_file:
                result = yaml.safe_load(output_file)
            try:
                cbs_ta_runtime = result["statistics"]['TA_runtime']
                cbsta_lowlevel_search_time = result["statistics"]['lowlevel_search_time']
            except :
                cbs_ta_runtime = 0
                cbsta_lowlevel_search_time = 0
            cbsta_time = result["statistics"]['runtime']
            cbsta_cost = result["statistics"]['cost']
            cbsta_ta_times = result["statistics"]['numTaskAssignments']
            lowLevelNode = result["statistics"]['total_lowlevel_node']
            test_name = os.path.basename(map_file_path)
            if tym_cost != "tym_Time_out" and cbsta_cost != tym_cost:
                print(test_name, tym_time, tym_cost, cbsta_time, cbsta_cost, "FALSE", flush=True)
                continue
            print(test_name,
                  tym_ta_runtime, tym_lowlevel_search_time, tym_time,
                  cbs_ta_runtime, cbsta_lowlevel_search_time, cbsta_time,
                  cbsta_cost, tym_ta_times, cbsta_ta_times, lowLevelNode, flush=True)
        else :
            test_name = os.path.basename(map_file_path)
            print(test_name,
                  tym_ta_runtime, tym_lowlevel_search_time, tym_time,
                  "cbsta_Time_out", "cbsta_Time_out", "cbsta_Time_out",
                  tym_cost, flush=True)
        #

