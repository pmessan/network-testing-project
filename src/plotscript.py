#!/usr/bin/python3
import matplotlib.pyplot as plt
import pandas as pd
import os.path

# %matplotlib inline  # jupyter notebook

# create dictionary for the methods to map to numbers
nums = [1, 2, 3, 4, 5, 6]
methods = ["Yggdrasil", "CJDNS", "", "", "", ""]



# Load data
_ = os.path.dirname(__file__)
data_p = pd.read_csv(_ + '/../trials/ping_test_danter-ygg.csv')
data = pd.read_csv(_ + '/../trials/tryme.csv')
data2 = pd.read_csv(_ + '/../trials/tryme2.csv')


# Plot iperf data
x = range(2)

ax = plt.subplot(111)
# plt.figure(figsize=(6.8, 4.2))

#compute averages and plot those
A=list(data['bits_per_second'])
Ae=A[0::2]
Ao=A[1::2]
B=list(data2['bits_per_second'])
Be=B[0::2]
Bo=B[1::2]
av1 = sum(Ae)/(len(Ae)*1000000)
av12 = sum(Ao)/(len(Ao)*1000000)
av2 = sum(Be)/(len(Bo)*1000000)
av22 = sum(Bo)/(len(Bo)*1000000)

#create client and server average lists
even = [av1, av2]
odd = [av12, av22]

# print(x.shape)


ax.bar(x, even, width=0.2, color='b', align='edge')
ax.bar(x, odd, width=-0.2, color='g', align='edge')



# plt.bar(x, data['bits_per_second'])
# plt.xticks(x, data['connection_type'])
plt.xlabel('Connection Type')
plt.ylabel('Speed in Mbps')
plt.savefig(_ + "/../charts/iperf_chart.png")



# plot ping data
C=list(data_p['roundtrip_time'])
C_min=min(C)
C_max=max(C)
C_avg=sum(C)/len(C)

ax.bar(1-0.2, C_min, width=0.2, color='b', align='center')
ax.bar(1, C_avg, width=0.2, color='g', align='center')
ax.bar(1+0.2, C_max, width=0.2, color='r', align='center')

plt.xlabel('Connection Type')
plt.ylabel('Time in ms')

plt.savefig(_ + "/../charts/ping_chart.png")

# plt.show()
