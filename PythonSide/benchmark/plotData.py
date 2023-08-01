import sys
sys.path.append('./src/')
import csv
import argparse
from Util.objectTrackingConstants import *
import matplotlib.pyplot as plt
import glob

plt.rcParams.update({'font.size': 18})

def getValues(data):
    with open(".\\benchmark\\results\\graphsAndValues\\values.csv", 'w') as csvfile:
        csvfile.write("Object,Tracker,Speed,Precision_Rotation,Accuracy_Rotation,Precision_Translation,Accuracy_Translation\n")
        for d in data:
            avg_speed = d['time'].mean()
            accuracy_rotation = d['rotation'].mean()
            accuracy_translation = d['translation'].mean()

            precision_rotation = d['rotation'].std()
            precision_translation = d['translation'].std()
            
            csvfile.write(f"{d['object']},{d['tracker']},{avg_speed},{accuracy_rotation},{precision_rotation},{accuracy_translation},{precision_translation}\n")     

def plotSpeed(data):
    fig, ax = plt.subplots(2,2, figsize=(20,10))
    fig.tight_layout(pad=3.0)
    objects = ['coin', 'cylinder', 'screw', 'vive']

    for d in data:
        for d in data:
            if d['object'] == objects[0]:  
                ax[0][0].plot(d['frame'], d['time'], label=d['tracker'])
            elif d['object'] == objects[1]:   
                ax[0][1].plot(d['frame'], d['time'], label=d['tracker'])
            elif d['object'] == objects[2]:  
                ax[1][0].plot(d['frame'], d['time'], label=d['tracker'])
            elif d['object'] == objects[3]:  
                ax[1][1].plot(d['frame'], d['time'], label=d['tracker'])

    ax[0][0].title.set_text('Coin')  
    ax[0][1].title.set_text('Cylinder') 
    ax[1][0].title.set_text('Drill')  
    ax[1][1].title.set_text('Vive')  
    

    ax[0][0].set(xlabel='Frame', ylabel='Time (s)', title='Coin')
    ax[0][1].set(xlabel='Frame', ylabel='Time (s)', title='Cylinder')
    ax[1][0].set(xlabel='Frame', ylabel='Time (s)', title='Drill')
    ax[1][1].set(xlabel='Frame', ylabel='Time (s)', title='Vive')


    Line, Label = ax[0][0].get_legend_handles_labels()

    fig.legend(Line[:4], Label[:4], bbox_to_anchor=(1.0, 0.6), title="Tracker")
    
    fig.savefig(f"benchmark\\results\\graphsAndValues\\frame_vs_time_all.png")

def plotRotation(data):
    fig, ax = plt.subplots(2,2, figsize=(20,10))
    fig.tight_layout(pad=5.0)
    objects = ['coin', 'cylinder', 'screw', 'vive']

    for d in data:
        for d in data:
            if d['object'] == objects[0]:  
                ax[0][0].plot(d['frame'], d['rotation'], label=d['tracker'])
            elif d['object'] == objects[1]:   
                ax[0][1].plot(d['frame'], d['rotation'], label=d['tracker'])
            elif d['object'] == objects[2]:  
                ax[1][0].plot(d['frame'], d['rotation'], label=d['tracker'])
            elif d['object'] == objects[3]:  
                ax[1][1].plot(d['frame'], d['rotation'], label=d['tracker'])

    ax[0][0].title.set_text('Coin')  
    ax[0][1].title.set_text('Cylinder') 
    ax[1][0].title.set_text('Drill')  
    ax[1][1].title.set_text('Vive')  
    

    ax[0][0].set(xlabel='Frame', ylabel='Rotation (rad)', title='Coin')
    ax[0][1].set(xlabel='Frame', ylabel='Rotation (rad)', title='Cylinder')
    ax[1][0].set(xlabel='Frame', ylabel='Rotation (rad)', title='Drill')
    ax[1][1].set(xlabel='Frame', ylabel='Rotation (rad)', title='Vive')


    Line, Label = ax[0][0].get_legend_handles_labels()

    fig.legend(Line[:4], Label[:4], bbox_to_anchor=(1.0, 0.6), title="Tracker")
    
    fig.savefig(f"benchmark\\results\\graphsAndValues\\frame_vs_rotation_all.png")

def plotTranslation(data):
    fig, ax = plt.subplots(2,2, figsize=(20,10))
    fig.tight_layout(pad=5.0)
    objects = ['coin', 'cylinder', 'screw', 'vive']
    for d in data:
        for d in data:
            if d['object'] == objects[0]:  
                ax[0][0].plot(d['frame'], d['translation'], label=d['tracker'])
            elif d['object'] == objects[1]:   
                ax[0][1].plot(d['frame'], d['translation'], label=d['tracker'])
            elif d['object'] == objects[2]:  
                ax[1][0].plot(d['frame'], d['translation'], label=d['tracker'])
            elif d['object'] == objects[3]:  
                ax[1][1].plot(d['frame'], d['translation'], label=d['tracker'])

    ax[0][0].title.set_text('Coin')  
    ax[0][1].title.set_text('Cylinder') 
    ax[1][0].title.set_text('Drill')  
    ax[1][1].title.set_text('Vive')  
    

    ax[0][0].set(xlabel='Frame', ylabel='Translation (m)', title='Coin')
    ax[0][1].set(xlabel='Frame', ylabel='Translation (m)', title='Cylinder')
    ax[1][0].set(xlabel='Frame', ylabel='Translation (m)', title='Drill')
    ax[1][1].set(xlabel='Frame', ylabel='Translation (m)', title='Vive')


    Line, Label = ax[0][0].get_legend_handles_labels()

    fig.legend(Line[:4], Label[:4], bbox_to_anchor=(1.0, 0.6), title="Tracker")
    
    fig.savefig(f"benchmark\\results\\graphsAndValues\\frame_vs_translation_all.png")

def plotSpeedAny(data, obj = 'all', tracker = 'all'):
    if obj == 'all' and tracker == 'all':
        fig, ax = plt.subplots()
        for d in data:
            ax.plot(d['frame'], d['time'], label=d['object']+"_"+d['tracker'])
    elif obj == 'all':
        fig, ax = plt.subplots()
        for d in data:
            if d['tracker'] == tracker:
                ax.plot(d['frame'], d['time'], label=d['object'])
    elif tracker == 'all':
        fig, ax = plt.subplots()
        for d in data:
            if d['object'] == obj:
                ax.plot(d['frame'], d['time'], label=d['tracker'])
    else:
        fig, ax = plt.subplots()
        for d in data:
            if d['object'] == obj and d['tracker'] == tracker:
                ax.plot(d['frame'], d['time'], label=d['object']+"_"+d['tracker'])
        
    ax.set(xlabel='Frame', ylabel='Time (s)', title='Frame vs Time')
    ax.legend()
    fig.savefig(f"benchmark\\results\\graphsAndValues\\frame_vs_time_{obj}_{tracker}.png")

def plotRotationAny(data, obj = 'all', tracker = 'all'):
    if obj == 'all' and tracker == 'all':
        fig, ax = plt.subplots()
        for d in data:
            ax.plot(d['frame'], d['rotation'], label=d['object']+"_"+d['tracker'])
    elif obj == 'all':
        fig, ax = plt.subplots()
        for d in data:
            if d['tracker'] == tracker:
                ax.plot(d['frame'], d['rotation'], label=d['object'])
    elif tracker == 'all':
        fig, ax = plt.subplots()
        for d in data:
            if d['object'] == obj:
                ax.plot(d['frame'], d['rotation'], label=d['tracker'])
    else:
        fig, ax = plt.subplots()
        for d in data:
            if d['object'] == obj and d['tracker'] == tracker:
                ax.plot(d['frame'], d['rotation'], label=d['object']+"_"+d['tracker'])
        
    ax.set(xlabel='Frame', ylabel='Rotation (rad)', title='Frame vs Rotation')
    ax.legend()
    fig.savefig(f"benchmark\\results\\graphsAndValues\\frame_vs_rotation_{obj}_{tracker}.png")

def plotRotationZAny(data, obj = 'all', tracker = 'all'):
    if obj == 'all' and tracker == 'all':
        fig, ax = plt.subplots()
        for d in data:
            ax.plot(d['frame'], d['rotation_z'], label=d['object']+"_"+d['tracker'])
    elif obj == 'all':
        fig, ax = plt.subplots()
        for d in data:
            if d['tracker'] == tracker:
                ax.plot(d['frame'], d['rotation_z'], label=d['object'])
    elif tracker == 'all':
        fig, ax = plt.subplots()
        for d in data:
            if d['object'] == obj:
                ax.plot(d['frame'], d['rotation_z'], label=d['tracker'])
    else:
        fig, ax = plt.subplots()
        for d in data:
            if d['object'] == obj and d['tracker'] == tracker:
                ax.plot(d['frame'], d['rotation_z'], label=d['object']+"_"+d['tracker'])
        
    ax.set(xlabel='Frame', ylabel='Rotation (rad)', title='Frame vs rotation_z')
    ax.legend()
    fig.savefig(f"benchmark\\results\\graphsAndValues\\frame_vs_rotation_z_{obj}_{tracker}.png")

def plotTranslationAny(data, obj = 'all', tracker = 'all'):
    if obj == 'all' and tracker == 'all':
        fig, ax = plt.subplots()
        for d in data:
            ax.plot(d['frame'], d['translation'], label=d['object']+"_"+d['tracker'])
    elif obj == 'all':
        fig, ax = plt.subplots()
        for d in data:
            if d['tracker'] == tracker:
                ax.plot(d['frame'], d['translation'], label=d['object'])
    elif tracker == 'all':
        fig, ax = plt.subplots()
        for d in data:
            if d['object'] == obj:
                ax.plot(d['frame'], d['translation'], label=d['tracker'])
    else:
        fig, ax = plt.subplots()
        for d in data:
            if d['object'] == obj and d['tracker'] == tracker:
                ax.plot(d['frame'], d['translation'], label=d['object']+"_"+d['tracker'])
    
    ax.set(xlabel='Frame', ylabel='Translation (m)', title='Frame vs Translation')
    ax.legend()
    fig.savefig(f"benchmark\\results\\graphsAndValues\\frame_vs_translation_{obj}_{tracker}.png")


if __name__ == "__main__":

    # parser = argparse.ArgumentParser(description='Benchmark Values.')
    # parser.add_argument('--tracker', dest='tracker', default='GenericObjectTracker',
    #                 help='Type of tracker being used')
    # parser.add_argument('--file', dest='path', default=IMAGE_FILE_LOCATIONS,
    #                 help='The path to the images being used')
    # args = parser.parse_args()


    files = glob.glob(".\\benchmark\\results\\*.csv")
    data = []

    for file in files:
        name = file.split("\\")[-1].split(".")[0]
        obj, track = name.split("_")
        
        with open(file) as csvfile:
            reader = csv.reader(csvfile, delimiter=',', quotechar='|')
            Frame = []
            Time = []
            Rotation = []
            Rotation_z = []
            Translation = []
            for row in reader:

                try:
                    fr, ti, ro, ro_z, tr = row
                    if int(fr) == 0:
                        Frame.append([])
                        Time.append([])
                        Rotation.append([])
                        Rotation_z.append([])
                        Translation.append([])
                    Frame[-1].append(int(fr))
                    Time[-1].append(float(ti))
                    Rotation[-1].append(float(ro))
                    Rotation_z[-1].append(float(ro_z))
                    Translation[-1].append(float(tr))
                except:
                    pass

            

            Frame = np.array(Frame)
            Time = np.array(Time)
            Rotation = np.array(Rotation)
            Rotation_z = np.array(Rotation_z)
            Translation = np.array(Translation)

            avg_time = Time.mean(axis=0)
            avg_rotation = Rotation.mean(axis=0)
            avg_rotation_z = Rotation_z.mean(axis=0)
            avg_translation = Translation.mean(axis=0)

            data.append({'object': obj, 'tracker': track, 'frame': Frame[0], 'time': avg_time, 'rotation': avg_rotation, 'rotation_z': avg_rotation_z, 'translation': avg_translation})


    # getValues(data)
    plotSpeed(data)
    plotRotation(data)
    plotTranslation(data)
    # plotSpeedAny(data, obj='all', tracker='all')
    # plotRotationAny(data, obj='vive', tracker='all')
    # plotRotationZAny(data, obj='vive', tracker='all')
    # plotTranslationAny(data, obj='vive', tracker='all')

    # plt.plot(Frame[0], avg_time, color='orange',label='Timg')
    # plt.plot(Frame[0], avg_rotation, color='red',label='Rotation')
    # plt.plot(Frame[0], avg_rotation_z, color='blue',label='Rotation_z')
    # plt.plot(Frame[0], avg_translation, color='green',label='Translation')


    # plt.title("Frame vs Time, Rotation, Translation")
    # plt.legend()
    # plt.show()

    
    

    

    



