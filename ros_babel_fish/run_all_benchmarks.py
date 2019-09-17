#!/usr/bin/env python

from __future__ import print_function, division
from benchmarks.benchmark_float_msg import Benchmark as FloatBenchmark
from benchmarks.benchmark_image_simple import Benchmark as ImageBenchmark
from benchmarks.benchmark_joint_state import Benchmark as JointStateBenchmark
import numpy as np
import os
import subprocess
import time

if __name__ == "__main__":
    benchmarks = [("Float Msg", FloatBenchmark), ("Image Header", ImageBenchmark),
                  ("Joint State", JointStateBenchmark)]
    b = []
    for bench in benchmarks:
        b.append(bench[1]())
        b[-1].start()
    time.sleep(1)
    bag_path = os.path.join(os.path.dirname(__file__), "benchmarks", "data", "2019-09-10-11-42-04.bag")
    subprocess.call(("rosbag", "play", bag_path, "-r 20", "-d 2"), env=os.environ.copy())
    results = []
    for bench in b:
        results.append(bench.wait())
    print("")
    print("=" * 6 + " Results " + "=" * 6)
    for i, bench in enumerate(b):
        result = results[i]
        print("Benchmark: ", benchmarks[i][0])
        # print(result)
        print("Name" + 18 * " " + "\t\t Count \t\t First \t\t Avg without first \t\t Result")
        for name in result:
            if len(result[name]) == 0:
                print(name, ": No results")
                continue
            first = result[name][0]
            subsequent = result[name][1:-1]
            rval = result[name][-1]
            avg = np.round(np.sum(subsequent) / len(subsequent))
            name_print = name + " " * (18 - len(name))
            print(name_print, ": \t\t", len(subsequent), "\t\t", "{0:.2f}".format(first / 1000), "us \t\t",
                  "{0:.2f}".format(avg / 1000), "us \t\t ", rval)
        print("")
