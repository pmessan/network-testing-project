#!/bin/bash

TOKEN="$RT3_TOKEN"
DEV_KEY="$RT3_DEVKEY"

comm_and=${1?"Error: command not given."}
DEVICE_ADDRESS=${2:-$RT3_TRACK3R_SSH_DEVICE_ID}
HOSTIP=${3:-$RT3_LAPTOP_PUBLIC_IP}



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
export PRX_srv="$(jq -r '.connection | .proxyserver' tmp.json)"
export PRX_prt="$(jq -r '.connection | .proxyport' tmp.json)"

## display new login info
echo "New Proxy server: $PRX_srv"
echo "Port number: $PRX_prt"

## ssh into target using login
ssh rem0te@$PRX_srv -p $PRX_prt

## delete file to prevent build up of entries
rm tmp.json

## remove content of env vars
export PRX_srv=""
export PRX_prt=""
