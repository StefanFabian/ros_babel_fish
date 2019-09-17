#!/usr/bin/env python

from __future__ import print_function, division

import os
import subprocess
import time
from threading import Thread
import numpy as np


class Benchmark:
    benchmarks = ["base", "rbf", "rbf_extractor", "rti"]
    threads = []
    results = []

    def __init__(self):
        pass

    @staticmethod
    def __run_benchmark(name, results, index):
        popen = subprocess.Popen(["rosrun", "ros_babel_fish", name], shell=False, stdout=subprocess.PIPE)
        times = []
        while popen.poll() is None:
            line = popen.stdout.readline()
            if line == "":
                continue
            try:
                v = float(line)
                try:
                    l = long(line)
                    if abs(l - v) < 0.001:
                        v = l
                except ValueError:
                    pass
                times.append(v)
            except ValueError:
                print("Error: ", line)
        print(name, " exited")
        results[index] = times

    def start(self):
        if len(self.threads) != 0:
            return
        self.results = [[]] * len(self.benchmarks)
        for i, benchmark in enumerate(self.benchmarks):
            t = Thread(target=self.__run_benchmark, args=("benchmark_float_msg_" + benchmark, self.results, i))
            self.threads.append(t)
            t.start()

    def wait(self):
        for thread in self.threads:
            thread.join()
        result = {}
        for i, benchmark in enumerate(self.benchmarks):
            result[benchmark] = self.results[i]
        return result


if __name__ == "__main__":
    b = Benchmark()
    b.start()
    time.sleep(2)
    bag_path = os.path.join(os.path.dirname(__file__), "data", "2019-09-10-11-42-04.bag")
    subprocess.call(("rosbag", "play", bag_path, "-r 20", "-d 2"), env=os.environ.copy())
    result = b.wait()
    # print(result)
    print("Name \t\t Count \t\t First \t\t Avg without first \t\t Result")
    for name in result:
        if len(result[name]) == 0:
            print(name, ": No results")
            continue
        first = result[name][0]
        subsequent = result[name][1:-1]
        rval = result[name][-1]
        avg = np.sum(subsequent) / len(subsequent)
        print(name, ": \t\t", len(subsequent), "\t\t", "{0:.2f}".format(first / 1000), "us \t\t",
              "{0:.2f}".format(avg / 1000), "us \t\t ", rval)
