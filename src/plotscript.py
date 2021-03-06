#!/usr/bin/python3
import matplotlib.pyplot as plt
import pandas as pd
import os
import pathlib
import argparse

###########
# Setup
############

print("Setting up...") 

parser = argparse.ArgumentParser(
    prog="./plotscript.py", description='Plot the results of the tests performed in network-test-script.sh')
parser.add_argument(
    "-p", "--path", help="Path relative to folder with input CSV files for the graphs.")
args = parser.parse_args()
path = args.path

folderList = os.listdir(path)

test_name = path.split("/")[-1]

# create array for the methods to map to numbers

methods = ["Yggdrasil", "CJDNS", "Husarnet",
            "Proxy Server"]

x = list(range(4))  # 6 methods being tested

plt.figure(figsize=(8.8, 5.5))

# to make axes and writing white
params = {"ytick.color" : "w",
          "xtick.color" : "w",
          "axes.labelcolor" : "w",
          "axes.edgecolor" : "w",
          "axes.titlecolor" : "w",
          "text.color" : "w",
          "axes.facecolor": "black"}
plt.rcParams.update(params)


def autolabel(rects):
    """Attach a text label above each bar in *rects*, displaying its height."""
    for rect in rects:
        height = rect.get_height()
        ax.annotate('{:.1f}'.format(height),
                    xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=(0, 3),  # 3 points vertical offset
                    textcoords="offset points",
                    ha='center', va='bottom')

# init empty arrays
iperf_servers = [0] * 4
iperf_clients = [0] * 4
min_ping = [0] * 4
max_ping = [0] * 4
avg_ping = [0] * 4
ssh_upload = [0] * 4
ssh_download = [0] * 4
rostopic_bw = [0] * 4

print("Setup complete.\nGenerating iperf TCP chart...")


###################
# plot iperf tcp data
###################

rel_path = path + "/iperf_tcp_test/"
filelist = sorted(os.listdir(rel_path))

for a in filelist:
    if a.endswith(".csv"):
        data = pd.read_csv(rel_path + a)
        # compute averages and plot those
        A = list(data['bits_per_second'].astype('float'))
        Ae = A[0::2]
        Ao = A[1::2]
        # print(Ae)
        av1 = sum(Ae)/(len(Ae)*1000000)
        av2 = sum(Ao)/(len(Ao)*1000000)

        # append to to clients and servers lists
        iperf_servers[filelist.index(a)] = av1
        iperf_clients[filelist.index(a)] = av2

# setup graph
ax = plt.subplot(111, label="iperf_tcp")
l1 = ax.bar(x, iperf_clients, width=0.3, color='orangered', align='edge')
l2 = ax.bar(x, iperf_servers, width=-0.3, color='limegreen', align='edge')
ax.legend((l1, l2), ("Local->Remote", "Remote->Local"))
autolabel(l1)
autolabel(l2)
ax.set_title('TCP Transmission Speed by Connection Type - '+ test_name)
ax.set_xticks(x)
ax.set_ylim([0, 50])
ax.set_xticklabels(methods)
plt.ylabel('Speed in Mbps')
plt.xlabel('Connection Type')

# save output
plt.savefig(path + "/charts/iperf_tcp_chart.png", dpi=200, transparent=True)

# CLEANUP
# drop the arrays with the iperf data
iperf_servers = [0] * 4
iperf_clients = [0] * 4

# clear legend for axes
ax.get_legend().remove()
plt.cla()

print("iPerf TCP chart generated successfully!\nGenerating iPerf UDP chart...")


###################
# plot iperf udp data
###################

rel_path = path + "/iperf_udp_test/"
filelist = sorted(os.listdir(rel_path))

for a in filelist:
    if a.endswith(".csv"):
        # compute averages and plot those
        data = pd.read_csv(rel_path + a)
        A = list(data['bits_per_second'].astype('float'))
        Ae = A[0::2]
        Ao = A[1::2]
        av1 = sum(Ae)/(len(Ae)*1000000)
        av2 = sum(Ao)/(len(Ao)*1000000)

        # append to to clients and servers lists
        iperf_servers[filelist.index(a)] = av1
        iperf_clients[filelist.index(a)] = av2

ax = plt.subplot(111, label="iperf_udp")
l1 = ax.bar(x, iperf_servers, width=0.3, color='turquoise', align='edge')
l2 = ax.bar(x, iperf_clients, width=-0.3, color='slateblue', align='edge')
ax.legend((l1, l2), ("Local->Remote", "Remote->Local"))
autolabel(l1)
autolabel(l2)
ax.set_title('UDP Transmission Speed by Connection Type - '+ test_name )
ax.set_xticks(x)
ax.set_ylim([0, 1.5])
ax.set_xticklabels(methods)
plt.ylabel('Speed in Mbps')
plt.xlabel('Connection Type')

# save output
plt.savefig(path + "/charts/iperf_udp_chart.png", dpi=200, transparent=True)

# CLEANUP
# drop the arrays with the iperf data
iperf_servers = [0] * 4
iperf_clients = [0] * 4

# clear legend for axes
ax.get_legend().remove()
plt.cla()


print("iPerf UDP chart generated successfully!\nGenerating ping chart...")


###################
# plot ping data
###################

rel_path = path + "/ping_test/"
filelist = sorted(os.listdir(rel_path))

for a in filelist:
    if a.endswith(".csv"):
        data = pd.read_csv(rel_path + a)
        # compute averages and plot those
        A = list(data['roundtrip_time'].astype('float'))
        min_A = min(A)
        max_A = max(A)
        avg_A = sum(A)/len(A)

        min_ping[filelist.index(a)] = min_A
        max_ping[filelist.index(a)] = max_A
        avg_ping[filelist.index(a)] = avg_A


ax = plt.subplot(111, label="ping")
ax.set_xticks(x)
ax.set_xticklabels(methods)
l1 = ax.bar([i+0.3 for i in x], min_ping, width=0.3, color='orangered', align='center')
l2 = ax.bar(x, avg_ping, width=0.3, color='forestgreen', align='center')
l3 = ax.bar([i-0.3 for i in x], max_ping, width=0.3, color='royalblue', align='center')
ax.legend((l1, l2, l3), ("Minimum roundtrip time",
                         "Average roundtrip time",
                         "Maximum roundtrip time"))
ax.set_title('Packet Roundtrip Time by Connection Type - '+ test_name)
autolabel(l1)
autolabel(l2)
autolabel(l3)
ax.set_ylim([0, 6000])
plt.xlabel('Connection Type')
plt.ylabel('Time in ms')

#save output
plt.savefig(path + "/charts/ping_chart.png", dpi=200, transparent=True)

# CLEANUP
# drop the arrays with the ping data
min_ping.clear()
max_ping.clear()
avg_ping.clear()

# clear legend for axes
ax.get_legend().remove()
plt.cla()

print("Ping chart generated successfully!\nGenerating ssh speed graphs...")


###################
# plot ssh speed data
###################

rel_path = path + "/ssh_test/"
filelist = sorted(os.listdir(rel_path))

for a in filelist:
    if a.endswith(".csv"):
        data = pd.read_csv(rel_path + a)
        # compute averages and plot those
        U = list(data['Upload_speed'].astype('float'))
        D = list(data['Download_speed'].astype('float'))

        avg_U = sum(U)/len(U)
        avg_D = sum(D)/len(D)

        ssh_upload[filelist.index(a)] = avg_U
        ssh_download[filelist.index(a)] = avg_D

                

ax = plt.subplot(111, label="ssh")
ax.set_xticks(x)
ax.set_xticklabels(methods)
l1 = ax.bar(x, ssh_upload, width=0.3, color='steelblue', align='edge')
l2 = ax.bar(x, ssh_download, width=-0.3, color='sienna', align='edge')
ax.legend((l1, l2), ("Average Upload Speed",
                     "Average Download Speed"))
autolabel(l1)
autolabel(l2)
ax.set_ylim([0, 5000])
plt.xlabel('Connection Type')
plt.ylabel('Speed in Kbps')
ax.set_title('SSH Data Transfer Speed by Connection Type - '+ test_name)

# save output
plt.savefig(path + "/charts/ssh_chart.png", dpi=200, transparent=True)

print("ssh chart generated successfully!\nGenerating rostopic graphs...")


###################
# plot rostopic bw data
###################

rel_path = path + "/rostopic_bw_test/"
filelist = sorted(os.listdir(rel_path))

for a in filelist:
    if a.endswith(".csv"):
        data = pd.read_csv(rel_path + a)
        # compute averages and plot those
        B = list(data["bandwidth"].astype('float'))

        avg_B = sum(B)/len(B)

        rostopic_bw[filelist.index(a)] = avg_B

                

ax = plt.subplot(111, label="rostopic")
ax.set_xticks(x)
ax.set_xticklabels(methods)
l1 = ax.bar(x, rostopic_bw, width=0.4, color='blueviolet', align='center')
ax.legend([l1], ["Average Bandwidth"])
autolabel(l1)
ax.set_ylim([0, 1300])
plt.xlabel('Connection Type')
plt.ylabel('Speed in KBps')
ax.set_title('ROS Topic Bandwidth of Image Stream Topic by Connection Type - '+ test_name)

# save output
plt.savefig(path + "/charts/rostopic_bw_chart.png", dpi=200, transparent=True)

print("Rostopic chart generated successfully!")



# complete!
print("All charts generated successfully!")