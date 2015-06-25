#!/usr/bin/env python

from scipy import signal
import math
import matplotlib.pyplot as plt
import numpy as np
import random

frequency = 100.0 # Hz

w = 2
time = [ t/frequency for t in range(1000)]
input_signal = [math.sin(w*t)+random.gauss(0,0.1) for t in time]

fc=10.0
Wn = 0.5*fc/frequency
N=3
b, a = signal.butter(N, Wn, 'low')
output_signal = signal.filtfilt(b, a, input_signal)


plt.subplot(2, 1, 1)
plt.plot(time,input_signal)
plt.subplot(2, 1, 2)

b, a = signal.butter(N, Wn, 'low')
output_signal = signal.filtfilt(b, a, input_signal)
plt.plot(time,output_signal)
plt.subplots_adjust(hspace=0.35)

plt.show()