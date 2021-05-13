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

- ``
