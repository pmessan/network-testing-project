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

# create array for the methods to map to numbers

methods = ["Yggdrasil", "CJDNS", "Husarnet",
           "Port Forwarding", "Wireguard", "Proxy Server"]

x = list(range(6))  # 6 methods being tested

plt.figure(figsize=(8.8, 5.5))

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
iperf_servers = [0] * 6
iperf_clients = [0] * 6
min_ping = [0] * 6
max_ping = [0] * 6
avg_ping = [0] * 6
ssh_upload = [0] * 6
ssh_download = [0] * 6

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
        A = list(data['bits_per_second'])
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
ax.legend((l1, l2), ("Client->Server", "Server->Client"))
autolabel(l1)
autolabel(l2)
ax.set_title('TCP Transmission Speed by Connection Type')
ax.set_xticks(x)
ax.set_xticklabels(methods)
plt.ylabel('Speed in Mbps')
plt.xlabel('Connection Type')

# export
plt.savefig(path + "/charts/iperf_tcp_chart.png", dpi=200)

# CLEANUP
# drop the arrays with the iperf data
iperf_servers = [0] * 6
iperf_clients = [0] * 6

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
        A = list(data['bits_per_second'])
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
ax.legend((l1, l2), ("Client->Server", "Server->Client"))
autolabel(l1)
autolabel(l2)
ax.set_title('UDP Transmission Speed by Connection Type')
ax.set_xticks(x)
ax.set_xticklabels(methods)
plt.ylabel('Speed in Mbps')
plt.xlabel('Connection Type')

plt.savefig(path + "/charts/iperf_udp_chart.png", dpi=200)

# CLEANUP
# drop the arrays with the iperf data
iperf_servers = [0] * 6
iperf_clients = [0] * 6

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
        A = list(data['roundtrip_time'])
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
ax.set_title('Packet Roundtrip Time by Connection Type')
autolabel(l1)
autolabel(l2)
autolabel(l3)
plt.xlabel('Connection Type')
plt.ylabel('Time in ms')

plt.savefig(path + "/charts/ping_chart.png", dpi=200)

# CLEANUP
# drop the arrays with the iperf data
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
        U = list(data['Upload_speed'])
        D = list(data['Download_speed'])

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
plt.xlabel('Connection Type')
plt.ylabel('Speed in Kbps')
ax.set_title('SSH Data Transfer Speed by Connection Type')

plt.savefig(path + "/charts/ssh_chart.png", dpi=200)

print("ssh chart generated successfully!")

# complete!
print("All charts generated successfully!")