# Network Testing Project

This repository contains the code, CSV data and graphs used to test the two meshnets, Husarnet, and remote .it for my bachelor thesis and project.
The repo is organised as follows:
* [src/](src/): The folder with the source code used for the tests ([network-test.sh](src/network-tests.sh)), as well as helper scripts such as:
    - [rostopic-bw-parser.py](src/rostopic-bw-parser.py): converts the output of the `rostopic bw` command to CSV format, 
    - [ping-csv.sh](src/ping-csv.sh): saves the output of the ping command into CSV
    - [plotscript.py](src/plotscript.py): generates bar charts from the CSV data stored in [test-results](test-results/).
    - [scp-speed-test.sh](src/scp-speed-test.sh): tests the upload download speeds over ssh using each of the networking approaches using scp.
    - [scp-speed-test-port.sh](src/scp-speed-test-port.sh): same as scp-speed-test.sh, but allows for specification of ports. This script was used in the tests involving remote.it since the port had to be specified in the ssh and scp commands.
    - [proxyservertest.sh](src/proxyservertest.sh): similar to network-tests.sh, but crafted specifically with remote.it in mind.
    - [Makefile](src/Makefile): sets up the Python virtual environment and installs packages specified in [requirements.txt](src/requirements.txt) using pip, which are needed for the Python scripts to run successfully on any platform, provided make is installed :).
* [test-results/](test-results/): the folder with the test results, organised according to locations of tests, and for each location, the tests that were performed.
    - [Accra, Ghana](test-results/Accra,%20Ghana)
    - [Bremen Airport](test-results/Bremen%20Airport)
    - [Girona, Spain](test-results/Girona,%20Spain)
    - [JUB Campus](test-results/JUB%20Campus)


## Commands Used

I used a map to represent the 4 approaches by numbers: 1=Yggdrasil, 2=CJDNS, 3=Husarnet, 4=remote.it. This map was used for the `$connection_method` variable, and in the file names in the various folders housing the individual test results.

```bash
./network-tests.sh user@remote-host user@local-host connection_method trial_location -t [ipv4/ipv6] # run network tests between current host and remote host
```
```bash 
make run ARGS="../test-results/Accra,\ Ghana" # used to plot the graphs
```
```bash 
./scp-speed-test.sh user@hostname [test file size in MBs]
```
```bash 
# change to directory of test results for a particular test e.g. cd test-results/Accra,\ Ghana
../../src/venv/bin/python3 ../../src/rostopic_bw-parser.py rostopic_bw_test/rostopic_bw_$connection_method-temp.txt | tail -n +2 >> rostopic_bw_testrostopic_bw_$connection_method.csv
```

More details about the code and the commands can be reveled upon close inspection of the files in their respective folders.

Thanks for Viewing!