
#!/bin/bash
# scp-speed-test.sh
#
# Usage:
#   ./scp-speed-test.sh user@hostname [test file size in MBs]
#
#############################################################

TOKEN="$RT3_TOKEN"
DEV_KEY="$RT3_DEVKEY"
DEVICE_ADDRESS=${1:-$RT3_TRACK3R_SSH_DEVICE_ID}
HOSTIP=${2:-$RT3_LAPTOP_PUBLIC_IP}


echo "Obtaining credentials..."
## get login credntials and store in json file
curl -s -X POST \
     -H "token:$TOKEN" \
     -H "developerkey:$DEV_KEY" \
     -d "{\"wait\":\"true\",\"deviceaddress\":\"$DEVICE_ADDRESS\", \
          \"hostip\":\"$HOSTIP\" }" \
     https://api.remot3.it/apv/v27/device/connect | jq '.' >> tmp.json


## remove content of env vars
unset PRX_srv
unset PRX_prt                                                                                                                                                                                  
                                                                                                                                                                                               
## obtain new server and port number from downloaded credentials                                                                                                                               
export PRX_srv=$(jq -r '.connection | .proxyserver' tmp.json)                                                                                                                                  
export PRX_prt=$(jq -r '.connection | .proxyport' tmp.json)                                                                                                                                    
                                                                                                                                                                                               
## display new login info                                                                                                                                                                      
echo "New Proxy server: $PRX_srv"                                                                                                                                                              
echo "Port number: $PRX_prt"                                                                                                                                                                   
                                                                                                                                                                                               
## ssh into target using login                                                                                                                                                                 
# ssh rem0te@$PRX_srv -p $PRX_prt                                                                                                                                                                
                                                                                                                                                                                               
## delete file to prevent build up of entries                                                                                                                                                  
rm tmp.json 

ssh_server=PRX_srv
port=PRX_prt
test_file=".scp-test-file"

# Optional: user specified test file size in MBs
if test -z "$3"
then
  # default size is 10MB
  test_size="10"
else
  test_size=$3
fi


# generate a file of all zeros
echo "Generating $test_size MB test file..."
dd if=/dev/zero of=$test_file bs=$(echo "$test_size*1024*1024" | bc) \
  count=1 &> /dev/null
# upload test
echo "Testing upload to $ssh_server..."
up_speed=$(scp -v -P $port $test_file $ssh_server:$test_file 2>&1 | \
  grep "Bytes per second" | \
  sed "s/^[^0-9]*\([0-9.]*\)[^0-9]*\([0-9.]*\).*$/\1/g")
  up_speed=$(echo "($up_speed/1000)" | bc)

# download test
echo "Testing download from $ssh_server..."
down_speed=$(scp -v -P $port $ssh_server:$test_file $test_file 2>&1 | \
  grep "Bytes per second" | \
  sed "s/^[^0-9]*\([0-9.]*\)[^0-9]*\([0-9.]*\).*$/\2/g")
  down_speed=$(echo "($down_speed/1000)" | bc)

# clean up
echo "Removing test file on $ssh_server..."
ssh $ssh_server -p $port "rm $test_file"
echo "Removing test file locally..."
rm $test_file


# print result
echo "$up_speed,$down_speed" 
