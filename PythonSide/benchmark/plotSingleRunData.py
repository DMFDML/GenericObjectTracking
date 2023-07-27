import sys
sys.path.append('./src/')
import csv
import argparse
from Util.objectTrackingConstants import *
import matplotlib.pyplot as plt


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Benchmark Values.')
    parser.add_argument('--tracker', dest='tracker', default='GenericObjectTracker',
                    help='Type of tracker being used')
    parser.add_argument('--file', dest='path', default=IMAGE_FILE_LOCATIONS,
                    help='The path to the images being used')
    args = parser.parse_args()


    filename = args.path + f"{args.tracker}.csv"
    Frame = []
    Time = []
    Rotation = []
    Rotation_z = []
    Translation = []
    
    with open(filename, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in reader:
            try:
                fr, ti, ro, ro_z, tr = row
                Frame[-1].append(int(fr))
                Time[-1].append(float(ti))
                Rotation[-1].append(float(ro))
                Rotation_z[-1].append(float(ro_z))
                Translation[-1].append(float(tr))
            except:
                Frame.append([])
                Time.append([])
                Rotation.append([])
                Rotation_z.append([])
                Translation.append([])


    Frame = np.array(Frame)
    Time = np.array(Time)
    Rotation = np.array(Rotation)
    Rotation_z = np.array(Rotation_z)
    Translation = np.array(Translation)

    avg_time = Time.mean(axis=0)
    avg_rotation = Rotation.mean(axis=0)
    avg_rotation_z = Rotation_z.mean(axis=0)
    avg_translation = Translation.mean(axis=0)

    plt.plot(Frame[0], avg_time, color='orange',label='Timg')
    plt.plot(Frame[0], avg_rotation, color='red',label='Rotation')
    plt.plot(Frame[0], avg_rotation_z, color='blue',label='Rotation_z')
    plt.plot(Frame[0], avg_translation, color='green',label='Translation')


    plt.title("Frame vs Time, Rotation, Translation")
    plt.legend()
    plt.show()

    
    

    

    



