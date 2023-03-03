import serial
import time
import matplotlib.pyplot as plt

# configure the serial port
ser = serial.Serial('/dev/ttyUSB0', 115200)

# initialize the plot
fig, ax = plt.subplots()
ax.set_xlabel('Time (s)')
ax.set_ylabel('Sensor Reading')
line_obj, = ax.plot([], [])  # create an empty line object to update

# set the maximum number of data points to display
max_points = 150
xdata, ydata = [], []

# read and plot the data in real-time
while True:
    raw_data = ser.readline().decode().strip()
    if raw_data:
        try:
            data = float(raw_data)
            # append the new data to the buffer
            xdata.append(time.time())
            ydata.append(data)
            # truncate the buffer if it exceeds the maximum size
            if len(xdata) > max_points:
                xdata = xdata[-max_points:]
                ydata = ydata[-max_points:]
            # update the plot with the buffer data
            line_obj.set_data(xdata, ydata)
            ax.relim()
            ax.autoscale_view()
            plt.pause(0.01)
        except ValueError:
            pass
