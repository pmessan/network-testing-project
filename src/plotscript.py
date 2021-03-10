#!/usr/bin/python3
import matplotlib.pyplot as plt
import pandas as pd
import os.path

"""
Setup
"""

# create array for the methods to map to numbers

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


###################
# plot iperf data
###################
data = pd.read_csv(_ + '/../trials/tryme.csv')
data2 = pd.read_csv(_ + '/../trials/tryme2.csv')

ax = plt.subplot(111, label="iperf")

# compute averages and plot those
A = list(data['bits_per_second'])
Ae = A[0::2]
Ao = A[1::2]
B = list(data2['bits_per_second'])
Be = B[0::2]
Bo = B[1::2]
av1 = sum(Ae)/(len(Ae)*1000000)
av12 = sum(Ao)/(len(Ao)*1000000)
av2 = sum(Be)/(len(Bo)*1000000)
av22 = sum(Bo)/(len(Bo)*1000000)

# create client and server average lists
even = [av1, av2, 0, 0, 0, 0]
odd = [av12, av22, 0, 0, 0, 0]

l1 = ax.bar(x, even, width=0.3, color='b', align='edge')
l2 = ax.bar(x, odd, width=-0.3, color='g', align='edge')
ax.legend((l1, l2), ("Client->Server", "Server->Client"))
autolabel(l1)
autolabel(l2)

# plt.bar(x, data['bits_per_second'])
ax.set_title('Transmission Speed by Connection Type')
ax.set_xticks(x)
ax.set_xticklabels(methods)

plt.ylabel('Speed in Mbps')
plt.savefig(_ + "/../charts/iperf_chart.png", dpi=250)

# clear legend for axes
ax.get_legend().remove()
plt.cla()


###################
# plot ping data
###################
data_p = pd.read_csv(_ + '/../trials/ping_test_danter-ygg.csv')

C = list(data_p['roundtrip_time'])
min_c = min(C)
max_c = max(C)
avg_c = sum(C)/len(C)

C_min = [min_c, 0, 0, 0, 0, 0]
C_max = [max_c, 0, 0, 0, 0, 0]
C_avg = [avg_c, 0, 0, 0, 0, 0]

ax = plt.subplot(111, label="ping")

ax.set_xticks(x)
ax.set_xticklabels(methods)

l1 = ax.bar([i+0.3 for i in x], C_min, width=0.3, color='b', align='center')
l2 = ax.bar(x, C_avg, width=0.3, color='g', align='center')
l3 = ax.bar([i-0.3 for i in x], C_max, width=0.3, color='r', align='center')
ax.legend((l1, l2, l3), ("Minimum roundtrip time",
                         "Average roundtrip time", "Maximum roundtrip time"))
ax.set_title('Packet Roundtrip Time by Connection Type')
autolabel(l1)
autolabel(l2)
autolabel(l3)

plt.xlabel('Connection Type')
plt.ylabel('Time in ms')

plt.savefig(_ + "/../charts/ping_chart.png", dpi=250)

# clear legend for axes
ax.get_legend().remove()
plt.cla()

###################
# plot ssh speed data
###################

data_s = pd.read_csv(_ + '/../trials/scp-speed.csv')

U = list(data_s['Upload_Speed'])
D = list(data_s['Download_Speed'])

avg_U = sum(U)/len(U)
avg_D = sum(D)/len(D)

U_all = [avg_U, 0, 0, 0, 0, 0]
D_all = [avg_D, 0, 0, 0, 0, 0]

ax = plt.subplot(111, label="ssh")

ax.set_xticks(x)
ax.set_xticklabels(methods)

l1 = ax.bar(x, U_all, width=0.3, color='b', align='edge')
l2 = ax.bar(x, D_all, width=-0.3, color='g', align='edge')

ax.legend((l1, l2), ("Average Upload Speed",
                     "Average Download Speed"))

autolabel(l1)
autolabel(l2)


plt.xlabel('Connection Type')
plt.ylabel('Speed in Mbps')

plt.savefig(_ + "/../charts/ssh_chart.png", dpi=250)
