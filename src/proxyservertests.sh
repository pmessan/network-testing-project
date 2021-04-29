#!/bin/bash

local_host="$1"
local_port="$2"

remote_host="$3"
remote_port="$4"

trial_location="$5"

ygg_host="$6"


# iperf tests

# TCP
cd ../test-results/$trial_location

iperf -c $remote_host -p $remote_port -t 30 -y c >> iperf_tcp_test/iperf_tcp_4-local.csv

ssh $ygg_host "iperf -c $local_host -p $local_port -t 30 -y c >> iperf_tcp_4-remote.csv"

scp $ygg_host:~/iperf_tcp_4-remote.csv iperf_tcp_test/

# write headers 

echo "timestamp,source_address,source_port,destination_address,destination_port,connection_type,interval,transferred_bytes,bits_per_second" >> iperf_tcp_test/iperf_tcp_4.csv

# write nth line from each file into new file
paste -d '\n' iperf_tcp_test/iperf_tcp_4-local.csv iperf_tcp_test/iperf_tcp_4-remote.csv >> iperf_tcp_test/iperf_tcp_4.csv

# remove the unnecessary files

# rm iperf_tcp_test/iperf_tcp_4-local.csv iperf_tcp_test/iperf_tcp_4-remote.csv


# UDP

iperf -c $remote_host -p $remote_port -t 30 -y c >> iperf_udp_test/iperf_udp_4-local.csv

ssh $ygg_host "iperf -c $local_host -p $local_port -t 30 -y c >> iperf_udp_4-remote.csv"

scp $ygg_host:~/iperf_udp_4-remote.csv iperf_udp_test/

# write headers 

echo "timestamp,source_address,source_port,destination_address,destination_port,connection_type,interval,transferred_bytes,bits_per_second" >> iperf_udp_test/iperf_udp_4.csv

# write nth line from each file into new file
paste -d '\n' iperf_udp_test/iperf_udp_4-local.csv iperf_udp_test/iperf_udp_4-remote.csv >> iperf_udp_test/iperf_udp_4.csv

# remove the unnecessary files

# rm iperf_udp_test/iperf_udp_4-local.csv iperf_udp_test/iperf_udp_4-remote.csv

exit 1