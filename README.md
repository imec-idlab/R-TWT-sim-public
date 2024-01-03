# Restricted Target Wake Time (R-TWT) discrete-event simulator

## Overview

The code in this repository is an open source project that has been developed and used to model various scenarios with real-time applications (RTA) in Wi-Fi 7 networks. In particular, the simulator allows modeling Wi-Fi networks with coexisting RTA and delay tolerant applications, where RTA applications use dedicated R-TWT service periods for exclusive channel access. The results of the experiments have been published in paper **A. Belogaev, et al. "Dedicated Restricted Target Wake Time for Real-Time Applications in Wi-Fi 7", in proc. of IEEE WCNC 2024**.

## Building simulator

The code can be easily built using GNU Make. For that, change directory to R-TWT-sim-public/src, and run the following command:
```shell
make
```

Note that simulator uses a 3rd party library to work with .json files (see directory R-TWT-sim-public/include).

## Running the experiment

Before running the experiment, please check the configuration file R-TWT-sim-public/src/config.json. This file is used by simulator to set all the important experiment's parameters. The default configuration file contains the following content:
```json
{
      "saveDelays": false,
      "bandwidth": 20,
      "simTime": 5e8,
      "seed": 1,
      "errorProb": 0.1,
      "edca": {
        "numSta": 1,
        "interval": 1e4,
        "pktSize": 1500
      },
      "twt": {
        "numSta": 1,
        "interval": 16000.0,
        "pktSize": 200,
        "spPeriod": 16000,
        "spDurationPkts": 3,
        "numRetx": 3
      }
    }
```
With this parameters, the experiment will be run with two Wi-Fi stations, one of which generates uplink RTA traffic, while the other one generates uplink delay tolerant traffic. For data transmission, the channel of 20 MHz is used. Delay tolerant data is transmitted using DCF, RTA data is transmitted using periodic R-TWT service periods with periodicity 16000us.

To run the experiment, execute the binary file:
```shell
./sim
```

After the simulation finishes, two files are created: delay statistics and queue size probability distribution for RTA station.
