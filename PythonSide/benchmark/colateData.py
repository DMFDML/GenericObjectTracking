import sys
sys.path.append('./src/')
import csv
import glob
import numpy as np

def saveValues(data):
    trackers = ['aruco', 'reflectorNoICP', 'reflector', 'general']
    file = open(f"./benchmark/results/graphsAndValues/averages.csv", 'w', newline='')
    file.write("object,tracker,Average Time,Average Rotation,Rotation SD,Average Position,Position SD\n")

    # Save all individual averages
    for d in data:
        file.write(f"{d['object']},{d['tracker']},{d['time'].mean()},{d['rotation'].mean()},{d['rotation'].std()},{d['translation'].mean()},{d['translation'].std()}\n")

    file.write("\ntracker,Average Time,Average Rotation,Rotation SD,Average Position,Position SD\n")
    values = [[],[],[],[]]

    for d in data:
        if d['tracker'] == trackers[0]:  
            values[0].append(d)
        elif d['tracker'] == trackers[1]:   
            values[1].append(d)
        elif d['tracker'] == trackers[2]:  
            values[2].append(d)
        elif d['tracker'] == trackers[3]:  
            values[3].append(d)
    
    for v in values:
        mean_t = np.array([d['time'] for d in v]).mean()
        mean_r = np.array([d['rotation'] for d in v]).mean()
        mean_p = np.array([d['translation'] for d in v]).mean()
        std_t = np.array([d['time'] for d in v]).std()
        std_r = np.array([d['rotation'] for d in v]).std()
        std_p = np.array([d['translation'] for d in v]).std()

        file.write(f"{v[0]['tracker']},{mean_t},{mean_r},{std_r},{mean_p},{std_p}\n")
            



if __name__ == "__main__":

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
    saveValues(data)


    
    

    

    



