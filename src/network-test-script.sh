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
network_type=${3?Error: network type not specified ()}     #1-6
trial_location=${4?Error: trial location not given: campus or long-distance?}
# ip_type=${5:-ipv4}

while getopts ":t:" opt; do
  case ${opt} in
    t )
      if [ $OPTARG == "ipv4" ] || [ $OPTARG == "ipv6" ]; then
        ip=$OPTARG
      else 
        echo "Invalid option: -t requires an argument: \"ipv4\" or \"ipv6\""
      fi
      ;;
    \? )
      echo "Invalid option: $OPTARG" 1>&2
      ;;
    : )
      echo "Invalid option: $OPTARG requires an argument: \"ipv4\" or \"ipv6\"" 1>&2
      ;;
  esac
done
shift $((OPTIND -1))


echo "Starting tests..."

## if directory does not exist, make it

[ ! -d "../test-results/$trial_location/ping_test" ] && mkdir -p ../test-results/$trial_location/ping_test && mkdir -p ../test-results/$trial_location/iperf_tcp_test && mkdir -p ../test-results/$trial_location/iperf_udp_test && mkdir -p ../test-results/$trial_location/rostopic_bw_test && mkdir -p ../test-results/$trial_location/ssh_test

cd ../test-results/$trial_location/

# ping test

echo "Starting ping test ..."

#put headers in first
echo "Destination_IP,trip_number,roundtrip_time," >> ping_test.csv
echo "run: ping6 -w30 $REMOTE_HOST >> ping_test_$network_type.csv"
../scripts/ping-csv.sh -6 -w30 $REMOTE_HOST >> ping_test/ping_test_$network_type.csv ## delete the 6 iff the host is not ipv6

echo -e "Test complete.\n"

 


# iperf TCP test; this time as the client
echo "Starting iperf client TCP test..."

echo "Starting iperf server on remote host..."
timeout 10s ssh $REMOTE_HOST "iperf -s -V &" ## delete -V iff the host is not ipv6

echo "run: iperf -c $REMOTE_HOST -t 30 >> iperf_tcp_test/iperf_tcp.csv"

#put csv headers in first
echo "timestamp,source_address,source_port,destination_address,destination_port,connection_type,interval,transferred_bytes,bits_per_second" >> iperf_tcp_$network_type.csv
iperf -c $REMOTE_HOST -V -t 30 -r -y c >> iperf_tcp_test/iperf_tcp_$network_type.csv  ## delete -V iff the host is not ipv6

#kill iperf server
ssh $REMOTE_HOST "pkill iperf"

echo -e "Test complete.\n"

 





# iperf UDP test; this time as the client

echo -e "Starting iperf client UDP test...\nStarting iperf server on remote host..."

#start iperf server in bg on remote host; timeout to prevent hanging
timeout 10s ssh $REMOTE_HOST "iperf -s -u -V  &" ## delete -V iff the host is not ipv6

echo "run: iperf -c $REMOTE_HOST -t 30 >> iperf_udp_test/iperf_udp_$network_type.csv"
#put csv headers in first
echo "timestamp,source_address,source_port,destination_address,destination_port,connection_type,interval,transferred_bytes,bits_per_second" >> iperf_udp_test/iperf_udp_$network_type.csv
iperf -c $REMOTE_HOST -V -u -t 30 -r -y c >> iperf_udp_test/iperf_udp_$network_type.csv  ## delete -V iff the host is not ipv6

#kill iperf server
ssh $REMOTE_HOST "pkill iperf"

echo -e "Test complete.\n"




#rostopic bw, this node receives the images
# use rosrun usb_cam usb_cam_node

echo -e "Starting rostopic bw test..."

read -p "Please start a ROS Master on this host with the command \"roscore &\" in a separate terminal. Press ENTER when you have done so: "

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
ssh $REMOTE_HOST ". ~/.zshrc && timeout 30s rostopic bw /cv_camera/image_raw/compressed >> rostopic_bw_$network_type.txt" ## time for 30 seconds


#kill ros node
pkill cv_camera

scp $REMOTE_HOST:~/rostopic_bw_$network_type.txt ./rostopic_bw_test/

ssh $REMOTE_HOST "rm rostopic_bw.txt"

echo -e "Test complete.\n"

 


echo -e "Starting ssh speed test...\nRunning ./scp-speed-test.sh..."
# call scp-speed-test.sh
echo "Upload_speed,Download_speed\n" >> ssh_speed/scp_speed_results_$network_type.csv
../scripts/scp-speed-test.sh $REMOTE_HOST 5 >> ssh_speed/scp_speed_results_$network_type.csv ## run test using 5M file

echo -e "Test complete.\n"


echo "All tests complete!"