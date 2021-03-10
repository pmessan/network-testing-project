#!/bin/bash
#
# network-test-script.sh - Peter-Newman Messan
#
# Usage:
#   ./network-test-script.sh user@remote-host local-host
#
# Convienience shell script to automate tests of a connection between two hosts
#
######################################################################################

REMOTE_HOST=${1?Error: no remote host address specified}
LOCAL_HOST=${2?Error: no local host specified}
trial_number=${3?Error: trial number not specified}
trial_location=${4?Error: trial location not given: campus or long-distance?}
# ip_type=${5:-ipv4}

echo "Starting tests..."

## if directory does not exist, make it

[ ! -d "../test-results/trial-$trial_number-$trial_location" ] && mkdir ../test-results/trial-$trial_number-$trial_location

cd ../test-results/trial-$trial_number-$trial_location

# ping test

echo "Starting ping test ..."

#put headers in first
echo "Destination_IP,trip_number,roundtrip_time," >> ping_test_$REMOTE_HOST.csv
echo "run: ping6 -w30 $REMOTE_HOST >> ping_test_$REMOTE_HOST.txt"
../scripts/ping-csv.sh -6 -w30 $REMOTE_HOST >> ping_test_$REMOTE_HOST.csv ## delete the 6 iff the host is not ipv6

echo -e "Test complete.\n"

 


# iperf TCP test; this time as the client
echo "Starting iperf client TCP test..."

echo "Starting iperf server on remote host..."
timeout 10s ssh $REMOTE_HOST "iperf -s -V >> iperf_s_tcp_client.txt &" ## delete -V iff the host is not ipv6

echo "run: iperf -c $REMOTE_HOST -t 30 >> test-results/iperf_tcp.csv"

#put csv headers in first
echo "timestamp,source_address,source_port,destination_address,destination_port,connection_type,interval,transferred_bytes,bits_per_second" >> iperf_tcp_$REMOTE_HOST.csv
iperf -c $REMOTE_HOST -V -t 30 -r -y c >> iperf_tcp_$REMOTE_HOST.csv  ## delete -V iff the host is not ipv6

#kill iperf server
ssh $REMOTE_HOST "pkill iperf"

echo -e "Test complete.\n"

 





# iperf UDP test; this time as the client

echo -e "Starting iperf client UDP test...\nStarting iperf server on remote host..."

#start iperf server in bg on remote host; timeout to prevent hanging
timeout 10s ssh $REMOTE_HOST "iperf -s -u -V >> iperf_s_tcp_client.txt &" ## delete -V iff the host is not ipv6

echo "run: iperf -c $REMOTE_HOST -t 30 >> test-results/iperf_udp.csv"
#put csv headers in first
echo "timestamp,source_address,source_port,destination_address,destination_port,connection_type,interval,transferred_bytes,bits_per_second" >> iperf_udp_$REMOTE_HOST.csv
iperf -c $REMOTE_HOST -V -u -t 30 -r -y c >> iperf_udp_$REMOTE_HOST.csv  ## delete -V iff the host is not ipv6

#kill iperf server
ssh $REMOTE_HOST "pkill iperf"

echo -e "Test complete.\n"




#rostopic bw, this node receives the images
# use rosrun usb_cam usb_cam_node

echo -e "Starting rostopic bw test..."

## figure how to start roscore more concretely
# X=$(pgrep roscore)

# echo "Starting roscore on local host"
# [ -z "$X" ] && roscore &   ## start roscore if it isn't already on the server
# wait 

echo "Starting ROS Node on local host"
rosrun cv_camera cv_camera_node &    ##can change the node to broadcast images

## on remote host, run rostopic bw
echo "Setting ROS Master on remote host"
timeout 10s ssh $REMOTE_HOST "echo \"export ROS_MASTER_URI=http://$LOCAL_HOST:11311/\" >> ~/.zshrc && exec zsh"

echo "run: rostopic bw /usb-cam/camera/image_raw/compressed"
ssh $REMOTE_HOST ". ~/.zshrc && timeout 30s rostopic bw /cv_camera/image_raw/compressed >> rostopic_bw_image_test.txt" ## time for 30 seconds


#kill ros node
pkill cv_camera

scp $REMOTE_HOST:~/rostopic_bw_image_test.txt .

ssh $REMOTE_HOST "rm rostopic_bw_image_test.txt"

echo -e "Test complete.\n"

 


echo -e "Starting ssh speed test...\nRunning ./scp-speed-test.sh..."
# call scp-speed-test.sh
echo "Upload_speed,Download_speed\n" >> scp-speed-results-$REMOTE_HOST.csv
../scripts/scp-speed-test.sh $REMOTE_HOST 5 >> scp-speed-results-$REMOTE_HOST.csv ## run test using 5M file

echo -e "Test complete.\n"


echo "All tests complete!"