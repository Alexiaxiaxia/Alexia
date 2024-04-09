# plotter.py
# Copyright (c) STMicroelectronics
# License: BSD-3-Clause
import argparse
import numpy as np
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument('filename')
args = parser.parse_args()

data = np.loadtxt(args.filename, delimiter=',')

plt.figure(figsize=(10, 3))
plt.plot(data[:, 0], label='x')
plt.plot(data[:, 1], label='y')
plt.plot(data[:, 2], label='z')
plt.legend()
plt.show()