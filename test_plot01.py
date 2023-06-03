import matplotlib.pyplot as plt
import random
import csv
import serial
from time import sleep

ser = serial.Serial ("/dev/ttyS0", 9600) 

def live_plot():
    plt.ion()  # Turn on interactive mode

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

    x_vals = []
    y1_vals = []
    y2_vals = []

    line1, = ax1.plot(x_vals, y1_vals, 'b-', label='Voltage')  # Create a line for voltage
    line2, = ax2.plot(x_vals, y2_vals, 'r-', label='Frequency')  # Create a line for frequency

    ax1.set_xlim(0, 100)  # Set the x-axis limits
    ax1.set_ylim(0, 100)  # Set the y-axis limits
    ax2.set_xlim(0, 100)  # Set the x-axis limits
    ax2.set_ylim(0, 100)  # Set the y-axis limits

    ax1.set_ylabel('Voltage')
    ax2.set_ylabel('Frequency')
    plt.xlabel('Time')
    plt.suptitle('Voltage-Time Frequency-Time')

    with open('random_values.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Time', 'Voltage', 'Frequency'])  # Write the header row

        while True:
            x = ser.read().decode("utf-8").strip()              #read serial port
            sleep(0.03)
            x_left = ser.inWaiting()             #check for remaining byte
            x += ser.read(x_left).decode("utf-8").strip()
            values = x.split(",")

            x_vals.append(len(x_vals) + 1)  # Increment x value
            # y1 = random.randint(1, 100)  # Generate random value for voltage
            # y2 = random.randint(1, 100)  # Generate random value for frequency
            y1 = float(values[0])
            y2 = float(values[1])
            y1_vals.append(y1)
            y2_vals.append(y2)

            line1.set_xdata(x_vals)
            line1.set_ydata(y1_vals)
            line2.set_xdata(x_vals)
            line2.set_ydata(y2_vals)

            ax1.relim()  # Recalculate the axes limits for voltage
            ax1.autoscale_view(True, True, True)  # Autoscale the axes for voltage
            ax2.relim()  # Recalculate the axes limits for frequency
            ax2.autoscale_view(True, True, True)  # Autoscale the axes for frequency

            plt.draw()
            plt.pause(0.1)  # Pause to update the plot

            writer.writerow([len(x_vals), y1, y2])  # Write the data to the CSV file

            if len(x_vals) >= 100:
                break

    plt.ioff()  # Turn off interactive mode
    plt.show()

live_plot()
