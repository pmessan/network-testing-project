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

echo "Starting tests..."

# ping test

# echo "Starting ping test ..."

# echo "run: ping6 -w30 $REMOTE_HOST >> test-results/ping6_test_$REMOTE_HOST.txt"
# ping6 -w30 $REMOTE_HOST >> test-results/ping6_test_$REMOTE_HOST.txt ## delete the 6 iff the host is not ipv6

# echo -e "Test complete.\n"

# # read -p "Press [ENTER] to continue or ^C to abort."


# # iperf TCP test; this time as the client
# echo "Starting iperf client TCP test..."

# echo "Starting iperf server on remote host..."
# timeout 10s ssh $REMOTE_HOST "iperf -s -V >> iperf_s_tcp_client.txt &" ## delete -V iff the host is not ipv6

# echo "run: iperf -c $REMOTE_HOST -t 30 >> test-results/iperf_c_tcp_client.txt"
# iperf -c $REMOTE_HOST -V -t 30 >> test-results/iperf_c_tcp_client.txt  ## delete -V iff the host is not ipv6

# #kill iperf server
# ssh $REMOTE_HOST "pkill iperf"

# #retrieve the results file
# scp $REMOTE_HOST:~/iperf_s_tcp_client.txt ./test-results/

# ssh $REMOTE_HOST "rm iperf_s_tcp_client.txt"

# echo -e "Test complete.\n"

# # read -p "Press [ENTER] to continue or ^C to abort."


# # iperf TCP test; this time as the server

# echo "Starting iperf server TCP test..."

# echo "Starting iperf server on local host"
# iperf -s -V >> test-results/iperf_s_tcp_server.txt &  ## delete -V iff the host is not ipv6

# echo "run: iperf -c $LOCAL_HOST -V -t 30 >> iperf_c_tcp_server.txt"
# ssh $REMOTE_HOST "iperf -c $LOCAL_HOST -V -t 30 >> iperf_c_tcp_server.txt"  ## delete -V iff the host is not ipv6
# scp $REMOTE_HOST:~/iperf_c_tcp_server.txt ./test-results/

# #end server
# pkill iperf

# ssh $REMOTE_HOST "rm iperf_c_tcp_server.txt"

# echo -e "Test complete.\n"

# # read -p "Press [ENTER] to continue or ^C to abort."


# # iperf UDP test; this time as the client

# echo -e "Starting iperf client UDP test...\nStarting iperf server on remote host..."

# #start iperf server in bg on remote host; timeout to prevent hanging
# timeout 10s ssh $REMOTE_HOST "iperf -s -V -u >> iperf_s_udp_client.txt &"  ## delete -V iff the host is not ipv6

# echo "run: iperf -c $REMOTE_HOST -V -u -t 30 >> iperf_c_udp_client.txt"
# iperf -c $REMOTE_HOST -V -u -t 30 >> test-results/iperf_c_udp_client.txt  ## delete -V iff the host is not ipv6

# #kill iperf server on remote host
# ssh $REMOTE_HOST pkill iperf

# #retrieve results
# scp $REMOTE_HOST:~/iperf_s_udp_client.txt ./test-results/

# ssh $REMOTE_HOST "rm iperf_c_tcp_server.txt"

# echo -e "Test complete.\n"

# # read -p "Press [ENTER] to continue or ^C to abort."

  
# # iperf UDP test; this time as the server

# echo -e "Starting iperf server UDP test...\nStarting iperf server..."

# iperf -s -V -u >> test-results/iperf_s_udp_server.txt &  ## delete -V iff the host is not ipv6

# echo "run: iperf -c $REMOTE_HOST -V -u -t 30 >> iperf_c_udp_server.txt"
# ssh $REMOTE_HOST "iperf -c $LOCAL_HOST -V -u -t 30 >> iperf_c_udp_server.txt"  ## delete -V iff the host is not ipv6

# #end server
# pkill iperf

# # retrieve the results
# scp $REMOTE_HOST:~/iperf_c_udp_server.txt ./test-results/

# ssh $REMOTE_HOST "rm iperf_c_udp_server.txt"

# echo -e "Test complete.\n"

# read -p "Press [ENTER] to continue or ^C to abort."


# rostopic bw, this node receives the images
## use rosrun usb_cam usb_cam_node

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

scp $REMOTE_HOST:~/rostopic_bw_image_test.txt ./test-results/

ssh $REMOTE_HOST "rm rostopic_bw_image_test.txt"

echo -e "Test complete.\n"

# read -p "Press [ENTER] to continue or ^C to abort."


# echo -e "Starting ssh speed test...\nRunning ./scp-speed-test.sh..."
# # call scp-speed-test.sh
# ./scp-speed-test.sh $REMOTE_HOST 5 >> test-results/scp-speed-results.txt ## run test using 5M file

# echo -e "Test complete.\n"

# mkdir trial-$trial_number-$trial_location
mv test-results/* trial-$trial_number-$trial_location/

echo "All tests complete!"
