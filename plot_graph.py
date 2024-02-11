import matplotlib.pyplot as plt
import numpy as np
import math


plt.style.use('seaborn-poster')




d = np.loadtxt("./dataSet/circleFast3000.txt", delimiter="\t")

data_x = []
data_y = []

filtered_x = []
filtered_y = []


speed = []
speedavg = []
speedmed = []

angles = []


dt_value_x = 0
dt_value_y = 0
for j in range(len(d)):
    data_x.append(d[j][0])
    data_y.append(d[j][1])

    filtered_x.append(d[j][2])
    filtered_y.append(d[j][3])

    speed.append(d[j][4])

    speedavg.append(d[j][6])

    speedmed.append(d[j][7])

    angles.append(d[j][8])


real_scale = True
ylim_start = 9000
ylim_end = 6000
xlim_start = 50
xlim_end = 15000

first=3
second=3
iter = 1

plt.subplot(first, second, iter)
plt.title("Captured Stroke")
if real_scale:
    plt.ylim([9000, 0])
    plt.xlim([0, 16000])
else:
    plt.ylim([ylim_start, ylim_end])
    plt.xlim([xlim_start, xlim_end])
plt.plot(data_x, data_y)
iter += 1

plt.subplot(first, second, iter)
plt.title("Filtered Stroke")
if real_scale:
    plt.ylim([9000, 0])
    plt.xlim([0, 16000])
else:
    plt.ylim([ylim_start, ylim_end])
    plt.xlim([xlim_start, xlim_end])
plt.plot(filtered_x, filtered_y)
iter += 1


plt.subplot(first, second, iter)
plt.title("Captured vs Filtered Overlapped")
if real_scale:
    plt.ylim([9000, 0])
    plt.xlim([0, 16000])
else:
    plt.ylim([ylim_start, ylim_end])
    plt.xlim([xlim_start, xlim_end])
plt.plot(data_x, data_y, color= "blue", alpha=0.5)
plt.plot(filtered_x, filtered_y, color="orange", alpha=0.5)
iter += 1


plt.subplot(first, second, iter)
plt.title("Speed Pixels/msec")
plt.plot(speed)
iter += 1


plt.subplot(first, second, iter)
plt.title("Speed  Histogram - Pixels/msec")
plt.hist(speed, bins=50, color='brown')
iter += 1

plt.subplot(first, second, iter)
plt.title("Speed  AVG - Pixels/msec")
plt.plot(speedavg)
iter += 1

plt.subplot(first, second, iter)
plt.title("Speed  MEDIAN - Pixels/msec")
plt.plot(speedmed)
iter += 1

plt.subplot(first, second, iter)
plt.title("Vector angle (radian)")
plt.plot(angles)
iter += 1

plt.subplot(first, second, iter)
plt.title("Vector angle (degree)")
deg = [ math.degrees(i) for i in angles]
plt.plot(deg)
iter += 1


plt.tight_layout()
plt.show()