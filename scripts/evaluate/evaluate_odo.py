# -*- coding: utf-8 -*-

"""
@File        : evaluate_odo.py
@Author      : hailiang
@Contact     : thl@whu.edu.cn
@Description : evaluate the odometry
@Version     : v1.0
"""

import numpy as np
import evaluate
from pathlib import Path

if __name__ == '__main__':
    truth_file = '/run/timeshift/backup/Datasets/WHU_IC_GVINS/kaist/urban38/truth_traj.csv'
    odo_file = '/home/divenire/0_myWorkSpace/Divenire_ws/workingProgram/icgvins_ws/src/IC-GVINS/results/T20230223192711/trajectory.csv'

    truth_traj = np.loadtxt(truth_file)

    # change the time stamp to GNSS time
    # odo_traj = evaluate.load_odo_file(odo_file)
    odo_traj = np.loadtxt(odo_file)

    filepath = Path(odo_file)
    outdir = str(filepath.parent)

    evaluate.evaluate(truth_traj, odo_traj, outdir)
