# iFogSim-programs
This folder contains various files containing iFogSim 2 programs written in Java. The programs simulate the operation, movement, clustering and load distribution between NB-IoT devices and fog nodes for various device specifications (regular and heavy vehicles), fog servers mounted on base stations (service provider-controlled and private fog servers) and cell range (cells and microcells). Details of each of the files are given below.

1] normal device operation.java - All devices with homogeneous (same) configuration moving within a cell

2] normal device operation - different device capacities.java - Devices with heterogeneous (different) configurations moving within a cell

3] clustering of devices in base stations.java - Situation when devices choose the nearest base station during the handover with the distance between device and base station calculated using Pythagoras theorem

4] clustering of devices in independent fog server.java - Situation when devices are clustered under the private fog servers based on the range of service of the server

5] devices moving between base station and independent fog server.java - Handover of devices between fog servers situated in the base station and private fog server

6] devices moving between base station and micro cell.java - Handover of devices between fog server in the base station and microcell tower in a microcell

7] devices moving between micro cells.java - Handover of devices between two microcell towers

8] devices moving between two base stations.java - Handover of devices between two base stations
