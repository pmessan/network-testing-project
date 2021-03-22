#!/venv/bin/python3

# import yaml

# with open("rostopic_bw_1_tabless.txt", 'r') as stream:
#     try:
#         print(yaml.safe_load(stream))
#     except yaml.YAMLError as exc:
#         print(exc)


"""
didn't work cuz of: 

while scanning for the next token
found character '\t' that cannot start any token
  in "rostopic_bw_1.txt", line 2, column 1

  fixed by sed 's/TAB//g' rostopic_bw_1.txt > rostopic_bw_1_tabless.txt [TAB = ctrl+v followed by the tab key]
"""

import sys
import os
import pandas as pd 

bw_hold = []
bw_values= []
mean_hold = []
mean_values = []
mini_hold = []
mini_values = []
maxi_hold = []
maxi_values = []
windows = []

rostopic_file = str(sys.argv[1])


with open(rostopic_file, 'r') as stream:
    next(stream)    # skip topic, since we don't need it
    while stream:
        # read first line & manipulate
        i1 = stream.readline()
        # print(i1)
        if i1 == '':
            break
        bw_hold.append(i1.rstrip().split(' ')[1])
        # print(bw_hold)
        # read second line & manipulate 
        i2 = stream.readline().rstrip().split(' ')
        mean_hold.append(i2[1])
        # print(mean)
        mini_hold.append(i2[3])
        maxi_hold.append(i2[5])
        windows.append(int(i2[7]))
    

## sanitize bw data
for i in bw_hold:
    if i.endswith("MB/s"):
        i_value = i.replace("MB/s", "")
        bw_values.append(float(i_value)*1000)
    else:   # KB/s
        i_value = i.replace("KB/s", "")
        bw_values.append(float(i_value))

## sanitize mini
for i in mini_hold:
    if i.endswith("MB"):
        i_value = i.replace("MB", "")
        mini_values.append(float(i_value)*1000)
    else:   # KB/s
        i_value = i.replace("KB", "")
        mini_values.append(float(i_value))

## sanitize maxi
for i in maxi_hold:
    if i.endswith("MB"):
        i_value = i.replace("MB", "")
        maxi_values.append(float(i_value)*1000)
    else:   # KB/s
        i_value = i.replace("KB", "")
        maxi_values.append(float(i_value))


## sanitize mean
for i in mean_hold:
    if i.endswith("MB"):
        i_value = i.replace("MB", "")
        mean_values.append(float(i_value)*1000)
    else:   # KB/s
        i_value = i.replace("KB", "")
        mean_values.append(float(i_value))
    

     
# dictionary of lists  
output_dict = {'bandwidth': bw_values, 'mean': mean_values, 'minimum': mini_values, 'maximum' : maxi_values, 'window' : windows}  
       
df = pd.DataFrame(output_dict) 
    
# saving the dataframe to csv
out_rostopic_file = rostopic_file.replace(".txt", ".csv") 
df.to_csv(rostopic_file.replace(".txt", ".csv"), index=False) 

# optional, can be commented out
# os.remove(rostopic_file)


