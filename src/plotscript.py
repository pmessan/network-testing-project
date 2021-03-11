#!/usr/bin/python3
import matplotlib.pyplot as plt
import pandas as pd
import os
import argparse

"""
Setup
"""

parser = argparse.ArgumentParser(
    prog="./plotscript", description='Plot the results of the tests performed in network-test-script.sh')
parser.add_argument(
    "-p", "--path", help="Path relative to folder with input CSV files for the graphs.")
args = parser.parse_args()
path = args.path

fileList = os.listdir(path)


# create array for the methods to map to numbers

print("Setting up...") 

methods = ["Yggdrasil", "CJDNS", "Husarnet",
           "Port Forwarding", "Wireguard", "Proxy Server"]


_ = os.path.dirname(__file__)

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


iperf_servers = []
iperf_clients = []
min_ping = []
max_ping = []
avg_ping = []
ssh_upload = []
ssh_download = []


print("Setup complete.\n Generating iperf TCP chart...")

###################
# plot iperf tcp data
###################

rel_path = path + "/iperf_tcp_test/"
filelist = os.listdir(rel_path)
for x in filelist:
    if x.endswith(".csv") and x.startswith("iperf_tcp"):
        try:
            with pd.read_csv(rel_path + x) as data:
                # compute averages and plot those
                A = list(data['bits_per_second'])
                Ae = A[0::2]
                Ao = A[1::2]
                av1 = sum(Ae)/(len(Ae)*1000000)
                av2 = sum(Ao)/(len(Ao)*1000000)

                # append to to clients and servers lists
                iperf_servers.append(av2)
                iperf_clients.append(av1)
        except:
            pass


# setup graph
ax = plt.subplot(111, label="iperf_tcp")
l1 = ax.bar(x, iperf_clients, width=0.3, color='b', align='edge')
l2 = ax.bar(x, iperf_servers, width=-0.3, color='g', align='edge')
ax.legend((l1, l2), ("Client->Server", "Server->Client"))
autolabel(l1)
autolabel(l2)
ax.set_title('TCP Transmission Speed by Connection Type')
ax.set_xticks(x)
ax.set_xticklabels(methods)
plt.ylabel('Speed in Mbps')
plt.xlabel('Connection Type')

plt.savefig(_ + "/../charts/iperf_udp_chart.png", dpi=200)


# CLEANUP
# drop the arrays with the iperf data
iperf_servers.clear()
iperf_clients.clear()

# clear legend for axes
ax.get_legend().remove()
plt.cla()

print("iPerf TCP chart generated successfully!\nGenerating iPerf UDP chart...")


###################
# plot iperf udp data
###################

rel_path = path + "/iperf_udp_test/"
filelist = os.listdir(rel_path)
for x in filelist:
    if x.endswith(".csv") and x.startswith("iperf_udp"):
        try:
            with pd.read_csv(rel_path + x) as data:
                # compute averages and plot those
                A = list(data['bits_per_second'])
                Ae = A[0::2]
                Ao = A[1::2]
                av1 = sum(Ae)/(len(Ae)*1000000)
                av2 = sum(Ao)/(len(Ao)*1000000)

                # append to to clients and servers lists
                iperf_servers.append(av2)
                iperf_clients.append(av1)
        except:
            pass


ax = plt.subplot(111, label="iperf_udp")
l1 = ax.bar(x, iperf_servers, width=0.3, color='b', align='edge')
l2 = ax.bar(x, iperf_clients, width=-0.3, color='g', align='edge')
ax.legend((l1, l2), ("Client->Server", "Server->Client"))
autolabel(l1)
autolabel(l2)
ax.set_title('UDP Transmission Speed by Connection Type')
ax.set_xticks(x)
ax.set_xticklabels(methods)
plt.ylabel('Speed in Mbps')
plt.xlabel('Connection Type')

plt.savefig(_ + "/../charts/iperf_udp_chart.png", dpi=200)

# CLEANUP
# drop the arrays with the iperf data
iperf_servers.clear()
iperf_clients.clear()

# clear legend for axes
ax.get_legend().remove()
plt.cla()

print("iPerf UDP chart generated successfully!\nGenerating ping chart...")


###################
# plot ping data
###################

rel_path = path + "/iperf_udp_test/"
filelist = os.listdir(rel_path)
for x in filelist:
    if x.endswith(".csv") and x.startswith("iperf_udp"):
        try:
            with pd.read_csv(rel_path + x) as data:
                # compute averages and plot those
                C = list(data['roundtrip_time'])
                min_A = min(A)
                max_A = max(A)
                avg_A = sum(A)/len(A)

                min_ping.append(min_A)
                max_ping.append(max_A)
                avg_ping.append(avg_A)
        except:
            pass


ax = plt.subplot(111, label="ping")
ax.set_xticks(x)
ax.set_xticklabels(methods)
l1 = ax.bar([i+0.3 for i in x], min_ping, width=0.3, color='b', align='center')
l2 = ax.bar(x, avg_ping, width=0.3, color='g', align='center')
l3 = ax.bar([i-0.3 for i in x], max_ping, width=0.3, color='r', align='center')
ax.legend((l1, l2, l3), ("Minimum roundtrip time",
                         "Average roundtrip time",
                         "Maximum roundtrip time"))
ax.set_title('Packet Roundtrip Time by Connection Type')
autolabel(l1)
autolabel(l2)
autolabel(l3)
plt.xlabel('Connection Type')
plt.ylabel('Time in ms')

plt.savefig(_ + "/../charts/ping_chart.png", dpi=250)

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

rel_path = path + "/iperf_udp_test/"
filelist = os.listdir(rel_path)
for x in filelist:
    if x.endswith(".csv") and x.startswith("iperf_udp"):
        try:
            with pd.read_csv(rel_path + x) as data:
                # compute averages and plot those
                U = list(data['Upload_Speed'])
                D = list(data['Download_Speed'])

                avg_U = sum(U)/len(U)
                avg_D = sum(D)/len(D)

                ssh_upload.append(avg_U)
                ssh_download.append(avg_D)

        except:
            pass
                

ax = plt.subplot(111, label="ssh")
ax.set_xticks(x)
ax.set_xticklabels(methods)
l1 = ax.bar(x, ssh_upload, width=0.3, color='b', align='edge')
l2 = ax.bar(x, ssh_download, width=-0.3, color='g', align='edge')
ax.legend((l1, l2), ("Average Upload Speed",
                     "Average Download Speed"))
autolabel(l1)
autolabel(l2)
plt.xlabel('Connection Type')
plt.ylabel('Speed in Mbps')

plt.savefig(_ + "/../charts/ssh_chart.png", dpi=200)

## complete!
print("All charts generated successfully!")