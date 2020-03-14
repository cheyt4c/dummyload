import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

path = "data/fourWatt.csv"
cleanpath = "data/cleandata.csv"

try:
    with open(cleanpath, 'w+') as cleancsvFile, open(path, 'r+') as f:
        lines = f.readlines()
        for line in lines:
            for ch in line:
                if ( ch != ' '):
                    cleancsvFile.write(ch)

except IOException:
    print("File not found")

# Extracting and plotting the dummy load data frame

dummyLoadData = pd.read_csv(cleanpath)
print(dummyLoadData.dtypes)

def printFrame(frame):
    for label, content in frame.items():
        print('label:', label, label == "Time")
        print('content:', content, sep='r')
    return

def extractCol(name, frame):
    for label, content in frame.items():
        if ( label == name ):
            return "".join(label), content
            #print('content:', content, sep='r')
    return "{}, notFound".format(name), "N/a"

voltLabel, voltContent = extractCol("Voltage", dummyLoadData)
currLabel, currContent = extractCol("Current", dummyLoadData)
powLabel, powContent = extractCol("Power", dummyLoadData)
timeLabel, timeContent = extractCol("Time", dummyLoadData)

timeInMins = list(map(lambda x: x/(60*1000), timeContent))
# print(voltLabel, "\n", voltContent)
# print(timeLabel, "\n", timeContent)
# printFrame(dummyLoadData)
fig, (ax0, ax1, ax2) = plt.subplots(nrows=3, constrained_layout=True)
# plt.axes(projection=None, polar=False, ylabel="Voltage (V)", xlabel="Time (mins)", title="Voltage vs Time")

ax0.plot(timeInMins, voltContent)
ax0.set_title("Voltage vs Time")
ax0.set_xlabel("Time (mins)")
ax0.set_ylabel("Voltage (V)")

ax1.plot(timeInMins, currContent)
ax1.set_title("Current vs Time")
ax1.set_xlabel("Time (mins)")
ax1.set_ylabel("Current (A)")

ax2.plot(timeInMins, powContent)
ax2.set_title("Power vs Time")
ax2.set_xlabel("Time (mins)")
ax2.set_ylabel("Power (W)")

plt.show()
