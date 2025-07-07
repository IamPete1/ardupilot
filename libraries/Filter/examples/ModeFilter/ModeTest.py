# Import data from ModeFilter example and compare it to python implementation
import csv
import statistics
from matplotlib import pyplot as plt

input = []
output = []

with open('libraries/Filter/examples/ModeFilter/test.csv', 'r') as csvfile:
    # creating a csv reader object
    csvreader = csv.reader(csvfile)

    # Skip header
    next(csvreader)
    next(csvreader)

    # Get data line by line
    for row in csvreader:
        input.append(float(row[0]))
        output.append(float(row[1]))

# Apply moving window median and mode
mode = []
median = []

for i in range(0, len(input)):
    windowEnd = i+1
    windowStart = max(0, i - 4)
    window = input[windowStart : windowEnd]
    mode.append(statistics.mode(window))
    median.append(statistics.median(window))

plt.plot(input)
plt.plot(output)
plt.plot(mode)
plt.plot(median)

plt.legend(['input','cpp','py mode','py median'])

plt.title("ModeFilter test")
plt.xlabel("Samples")
plt.ylabel("Value")

plt.show()
