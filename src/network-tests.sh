#!/bin/bash
#
# network-test-s.sh - Peter-Newman Messan
#
# Usage:
#   ./network-tests.sh user@remote-host user@local-host connection_method trial_location -t [ipv4/ipv6]
#
# Convienience shell script to automate tests of a connection between two hosts
#
######################################################################################


usage() {
echo "
Usage: ./network-test-script remote_host local_host connection_method trial_location -t [ipv4/ipv6]
    remote_host ...... Remote host's hostname on the network to be tested
    local_host ........ Local host (this machine)'s hostname on the network to be tested
    connection_method .... Numerical representation of connection method to test (1-6)
    trial_location .... remote host location
    -t [ipv4/ipv6] .... Indicate type of network
"
}

removeLastLine() {
  length=$(wc -c < "$1")
  if [ "$length" -ne 0 ] && [ -z "$(tail -c -1 <file)" ]; then
    # The file ends with a newline or null
    dd if=/dev/null of="$1" obs="$((length-1))" seek=1
  fi
}

remote_host="${1?Error: no remote host address specified}"
local_host="${2?Error: no local host specified}"
connection_method="${3?Error: network type not specified (1-6)}"     #1-6
trial_location="${4?Error: trial location not given: campus or long-distance?}"

## check input
x=$(cat /etc/hosts | grep "$remote_host")
y=$(cat /etc/hosts | grep "$local_host")

if [ -z "$x" ]; then 
  echo "Remote host specified is not in /etc/hosts database. Please update the database and try again. Exiting ..."
  exit 1
fi

if [ -z "$y" ]; then 
  echo "Local host specified is not in the /etc/hosts database. Please update the database and try again. Exiting ..."
  exit 1
fi

if (( connection_method < 1 || connection_method > 6)); then
  echo "Connection method number specified is outside range. Please enter the correct corresponding number and try again. Exiting ..."
  exit 1
fi

shift 4

ip=""
while getopts "t:" opt || :; do
  case $opt in
    t )
      if [ $OPTARG == "ipv4" ] || [ $OPTARG == "ipv6" ]; then
        ip="$OPTARG"; 
      else 
        echo "Invalid option: -t requires an argument: \"ipv4\" or \"ipv6\""
        exit 1
      fi
      break;;
    [?] )
      echo "Invalid option: \"$OPTARG\""
      usage && exit 1
      ;;
    : )
      echo "Invalid option: $OPTARG requires an argument: \"ipv4\" or \"ipv6\"" 1>&2
      shift
      ;;
  esac
done

### parsing input complete


echo "Starting tests..."

## if directory does not exist, make it

if [ ! -d "../test-results/$trial_location/ping_test" ] ; then
  mkdir -p "../test-results/$trial_location/ping_test"
  mkdir -p "../test-results/$trial_location/iperf_tcp_test"
  mkdir -p "../test-results/$trial_location/iperf_udp_test"
  mkdir -p "../test-results/$trial_location/rostopic_bw_test"
  mkdir -p "../test-results/$trial_location/ssh_test"
fi

cd ../test-results/$trial_location/

# ping test

echo "Starting ping test ..."
out=$(cat ping_test/ping_test_$connection_method.csv)

[ ! -f ping_test/ping_test_$connection_method.csv ] && echo "Destination_IP,trip_number,roundtrip_time" >> ping_test/ping_test_$connection_method.csv
echo "Running ping test..."
if [ ip == "ipv6" ] ; then
  removeLastLine "ping_test/ping_test_$connection_method.csv"
  ../../src/ping-csv.sh -6 -w30 $remote_host >> ping_test/ping_test_$connection_method.csv 
else
  removeLastLine "ping_test/ping_test_$connection_method.csv"
  ../../src/ping-csv.sh -w30 $remote_host >> ping_test/ping_test_$connection_method.csv 
fi
echo "Test complete."

 


# iperf TCP test; this time as the client
echo "Starting iperf client TCP test..."

echo "Starting iperf server on remote host..."
[ $ip == "ipv6" ] && timeout 10s ssh $remote_host "iperf -s -V &" || timeout 10s ssh $remote_host "iperf -s &"

echo "run: iperf -c $remote_host -t 30 >> iperf_tcp_test/iperf_tcp.csv"

#put csv headers in first
[ ! -f iperf_tcp_test/iperf_tcp_$connection_method.csv ] && echo "timestamp,source_address,source_port,destination_address,destination_port,connection_type,interval,transferred_bytes,bits_per_second" >> iperf_tcp_test/iperf_tcp_$connection_method.csv
if [ $ip == "ipv6" ] ; then
  removeLastLine "iperf_tcp_test/iperf_tcp_$connection_method.csv"
  iperf -c $remote_host -V -t 30 -r -y c >> iperf_tcp_test/iperf_tcp_$connection_method.csv 
else
  removeLastLine "iperf_tcp_test/iperf_tcp_$connection_method.csv"
  iperf -c $remote_host -t 30 -r -y c >> iperf_tcp_test/iperf_tcp_$connection_method.csv ## delete -V iff the host is not ipv6
fi

#kill iperf server
ssh $remote_host "pkill iperf"

echo "Test complete."



# iperf UDP test; this time as the client

echo -e "Starting iperf client UDP test...\nStarting iperf server on remote host..."

#start iperf server in bg on remote host; timeout to prevent hanging
[ $ip == "ipv6" ] && timeout 10s ssh $remote_host "iperf -s -u -V  &" || timeout 10s ssh $remote_host "iperf -s -u  &"## delete -V iff the host is not ipv6

echo "run: iperf -c $remote_host -t 30 >> iperf_udp_test/iperf_udp_$connection_method.csv"
#put csv headers in first
[ ! -f iperf_udp_test/iperf_udp_$connection_method.csv ] && echo "timestamp,source_address,source_port,destination_address,destination_port,connection_type,interval,transferred_bytes,bits_per_second,jitter,cnterror,cntDatagrams,lostDatagrams,nOutOfOrder" >> iperf_udp_test/iperf_udp_$connection_method.csv
if [ $ip == "ipv6" ] ; then
  removeLastLine "iperf_udp_test/iperf_udp_$connection_method.csv"
  iperf -c $remote_host -V -u -t 30 -r -y c | sed '2d' >> iperf_udp_test/iperf_udp_$connection_method.csv 
else
  removeLastLine "iperf_udp_test/iperf_udp_$connection_method.csv"
  iperf -c $remote_host -u -t 30 -r -y c | sed '2d' >> iperf_udp_test/iperf_udp_$connection_method.csv  ## delete -V iff the host is not ipv6
fi

#kill iperf server
ssh $remote_host "pkill iperf"

echo "Test complete."




#rostopic bw, this node receives the images
# use rosrun usb_cam usb_cam_node

echo -e "Starting rostopic bw test...\nStarting roscore on local host..."

X=$(pgrep roscore)

if [ -z "$X" ] ; then 
  roscore &
  sleep 2
else 
  echo "roscore already running!"  ## start roscore if it isn't already on the server
fi

echo "Starting ROS Node on local host..."
rosrun cv_camera cv_camera_node &    ##can change the node to broadcast images

## on remote host, run rostopic bw
echo "Setting ROS Master on remote host..."
if [ $ip == "ipv6" ]; then 
  timeout 10s ssh $remote_host "echo -e \"export ROS_MASTER_URI=http://$local_host:11311/\nexport ROS_IPV6=on\nexport ROS_HOSTNAME=$remote_host\n\" >> ~/.zshrc && exec zsh"
else 
  timeout 10s ssh $remote_host "echo -e \"export ROS_MASTER_URI=http://$local_host:11311/\nexport ROS_IPV6=off\nexport ROS_HOSTNAME=$remote_host\n\" >> ~/.zshrc && exec zsh"
fi

echo "run: rostopic bw /usb-cam/camera/image_raw/compressed"
ssh $remote_host ". ~/.zshrc && timeout 30s rostopic bw /cv_camera/image_raw/compressed >> rostopic_bw_$connection_method.txt" ## time for 30 seconds

#kill ros node
pkill cv_camera
pkill roscore 
sleep 2

scp $remote_host:~/rostopic_bw_$connection_method.txt ./rostopic_bw_test/

# sed '$d' rostopic_bw_test/rostopic_bw_$connection_method.txt

ssh $remote_host "rm rostopic_bw_$connection_method.txt"

echo "Test complete."

 


echo -e "Starting ssh speed test...\nRunning ./scp-speed-test.sh with 5 MB test file..."

[ ! -f ssh_test/scp_speed_results_$connection_method.csv ] && echo "Upload_speed,Download_speed" >> ssh_test/scp_speed_results_$connection_method.csv

removeLastLine "ssh_test/scp_speed_results_$connection_method.csv"

# call scp-speed-test.sh
../../src/scp-speed-test.sh $remote_host 5 >> ssh_test/scp_speed_results_$connection_method.csv ## run test using 5M file

echo "Test complete."


## tests complete

echo "All tests complete!"

exit 1;