import sys
sys.path.append('./src/')
import csv
import argparse
from Util.objectTrackingConstants import *
import matplotlib.pyplot as plt
import glob
import numpy as np

plt.rcParams.update({'font.size': 20})

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

def plotValues(data, graph_type='time', graph_lable = 'Time (s)', best_case_value = 0.33333333, best_case_text='real time', y_min=0, y_max=0.2):
    fig, ax = plt.subplots(2,2, figsize=(20,10))
    fig.tight_layout(pad=3.0)
    objects = ['coin', 'cylinder', 'screw', 'vive']

    for d in data:
        if d['object'] == objects[0]:  
            ax[0][0].plot(d['frame'], d[graph_type], label=d['tracker'])
        elif d['object'] == objects[1]:   
            ax[0][1].plot(d['frame'], d[graph_type], label=d['tracker'])
        elif d['object'] == objects[2]:  
            ax[1][0].plot(d['frame'], d[graph_type], label=d['tracker'])
        elif d['object'] == objects[3]:  
            ax[1][1].plot(d['frame'], d[graph_type], label=d['tracker'])

    ax[0][0].title.set_text('Coin')  
    ax[0][1].title.set_text('Cylinder') 
    ax[1][0].title.set_text('Drill')  
    ax[1][1].title.set_text('Vive')  

    best_case = np.ones(len(data[0]['frame'])) * best_case_value
    ymin = np.ones(len(data[0]['frame'])) * y_min
    ymax = np.ones(len(data[0]['frame'])) * y_max

    # ax[0][0].plot(data[0]['frame'], best_case, label=best_case_text)  
    # ax[0][1].plot(data[0]['frame'], best_case, label=best_case_text)
    # ax[1][0].plot(data[0]['frame'], best_case, label=best_case_text)
    # ax[1][1].plot(data[0]['frame'], best_case, label=best_case_text)

    ax[0][0].fill_between(data[0]['frame'], ymin, best_case, color='green', alpha=.1)  
    ax[0][1].fill_between(data[0]['frame'], ymin, best_case, color='green', alpha=.1)
    ax[1][0].fill_between(data[0]['frame'], ymin, best_case, color='green', alpha=.1)
    ax[1][1].fill_between(data[0]['frame'], ymin, best_case, color='green', alpha=.1)

    ax[0][0].fill_between(data[0]['frame'], best_case, ymax, color='red', alpha=.1)  
    ax[0][1].fill_between(data[0]['frame'], best_case, ymax, color='red', alpha=.1)
    ax[1][0].fill_between(data[0]['frame'], best_case, ymax, color='red', alpha=.1)
    ax[1][1].fill_between(data[0]['frame'], best_case, ymax, color='red', alpha=.1)

    
    
    ax[0][0].set(xlabel='Frame', ylabel=graph_lable, title='Coin')
    ax[0][1].set(xlabel='Frame', ylabel=graph_lable, title='Cylinder')
    ax[1][0].set(xlabel='Frame', ylabel=graph_lable, title='Drill')
    ax[1][1].set(xlabel='Frame', ylabel=graph_lable, title='Vive')


    Line, Label = ax[0][0].get_legend_handles_labels()

    fig.legend(Line[:4], Label[:4], loc='lower center', bbox_to_anchor=(0.5, -0.02), ncol=4)
    fig.savefig(f"benchmark\\results\\graphsAndValues\\frame_vs_{graph_type}_all.png")

def plotTranslationAndRotation(data, best_case_values = [0.377, 0.11], y_mins=[0, 0,0,0], y_maxs=[2.5, 0.2, 0.6]):
    fig, ax = plt.subplots(3,3, figsize=(20,10))
    fig.tight_layout()
    objects = ['coin', 'cylinder', 'screw', 'vive']

    for d in data:
        if d['object'] == objects[0]:
              
            ax[0][0].plot(d['frame'], d['rotation'], label=d['tracker'])
            ax[1][0].plot(d['frame'], d['translation'], label=d['tracker'])

        elif d['object'] == objects[1]:   
            
            ax[0][1].plot(d['frame'], d['rotation'], label=d['tracker'])
            ax[1][1].plot(d['frame'], d['translation'], label=d['tracker'])
            
        elif d['object'] == objects[2]:  

            ax[0][2].plot(d['frame'], d['rotation'], label=d['tracker'])
            ax[1][2].plot(d['frame'], d['translation'], label=d['tracker'])

        elif d['object'] == objects[3]:  
            ax[2][0].plot(d['frame'], d['rotation'], label=d['tracker'])
            ax[2][1].plot(d['frame'], d['translation'], label=d['tracker'])

    ax[0][0].title.set_text('Coin')  
    ax[0][1].title.set_text('Cylinder') 
    ax[0][2].title.set_text('Drill')  
    ax[2][0].title.set_text('Vive')  
    ax[2][1].title.set_text('Vive')  

    best_case_rot = np.ones(len(data[0]['frame'])) * best_case_values[0]
    best_case_tran = np.ones(len(data[0]['frame'])) * best_case_values[1]

    y_min_rot = np.ones(len(data[0]['frame'])) * y_mins[0]
    y_max_rot = np.ones(len(data[0]['frame'])) * y_maxs[0]

    y_min_tran = np.ones(len(data[0]['frame'])) * y_mins[1]
    y_max_tran = np.ones(len(data[0]['frame'])) * y_maxs[1]
    y_max_tran_vive = np.ones(len(data[0]['frame'])) * y_maxs[2]

    ax[0][0].fill_between(data[0]['frame'], y_min_rot, best_case_rot, color='green', alpha=.1)  
    ax[0][1].fill_between(data[0]['frame'], y_min_rot, best_case_rot, color='green', alpha=.1)
    ax[0][2].fill_between(data[0]['frame'], y_min_rot, best_case_rot, color='green', alpha=.1)
    ax[0][0].fill_between(data[0]['frame'], best_case_rot, y_max_rot, color='red', alpha=.1)  
    ax[0][1].fill_between(data[0]['frame'], best_case_rot, y_max_rot, color='red', alpha=.1)
    ax[0][2].fill_between(data[0]['frame'], best_case_rot, y_max_rot, color='red', alpha=.1)

    ax[1][0].fill_between(data[0]['frame'], y_min_tran, best_case_tran, color='green', alpha=.1)  
    ax[1][1].fill_between(data[0]['frame'], y_min_tran, best_case_tran, color='green', alpha=.1)
    ax[1][2].fill_between(data[0]['frame'], y_min_tran, best_case_tran, color='green', alpha=.1)
    ax[1][0].fill_between(data[0]['frame'], best_case_tran, y_max_tran, color='red', alpha=.1)  
    ax[1][1].fill_between(data[0]['frame'], best_case_tran, y_max_tran, color='red', alpha=.1)
    ax[1][2].fill_between(data[0]['frame'], best_case_tran, y_max_tran, color='red', alpha=.1)

    ax[2][0].fill_between(data[0]['frame'], y_min_rot, best_case_rot, color='green', alpha=.1)
    ax[2][0].fill_between(data[0]['frame'], best_case_rot, y_max_rot, color='red', alpha=.1)
    ax[2][1].fill_between(data[0]['frame'], y_min_tran, best_case_tran, color='green', alpha=.1)
    ax[2][1].fill_between(data[0]['frame'], best_case_tran, y_max_tran_vive, color='red', alpha=.1)
    
    ax[0][0].set(ylabel='Rotation(rad)')
    ax[1][0].set(ylabel='Translation(m)')

    ax[2][0].set(ylabel='Rotation(rad)')
    ax[2][1].set(ylabel='Translation(m)', xlabel='Frame', title='Vive')

    ax[0][0].set(title='Coin')
    ax[0][1].set(title='Cylinder')
    ax[0][2].set(title='Drill')

    Line, Label = ax[0][0].get_legend_handles_labels()
    fig.delaxes(ax[2,2])

    fig.legend(Line[:4], Label[:4], loc='center', bbox_to_anchor=(0.75,0.2), title="Tracker")
    
    fig.savefig(f"benchmark\\results\\graphsAndValues\\rotation_and_translation.png")

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


            avg_time = Time.mean(axis=0)[:170]
            avg_rotation = Rotation.mean(axis=0)[:170]
            avg_rotation_z = Rotation_z.mean(axis=0)[:170]
            avg_translation = Translation.mean(axis=0)[:170]
            

            data.append({'object': obj, 'tracker': track, 'frame': Frame[0][:170], 'time': avg_time, 'rotation': avg_rotation, 'rotation_z': avg_rotation_z, 'translation': avg_translation})


    # getValues(data)
    plotValues(data, graph_type='time', graph_lable="Time (s)", best_case_value=0.058, best_case_text='real time', y_min=0, y_max=0.2)
    # plotValues(data, graph_type='rotation', graph_lable="Rotation (rad)", best_case_value=0.0080, best_case_text='Vive tracker', y_min=0, y_max=3.151)
    # plotValues(data, graph_type='rotation_z', graph_lable="Rotation_z (rad)", best_case_value=0.0080, best_case_text='Vive tracker', y_min=0, y_max=3.151)
    # plotValues(data, graph_type='translation', graph_lable="Translation (m)", best_case_value=0.0004, best_case_text='Vive tracker', y_min=0, y_max=0.5)
    plotTranslationAndRotation(data)
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

    
    

    

    



