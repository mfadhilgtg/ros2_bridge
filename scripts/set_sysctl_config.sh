#!/bin/bash

#increase UDP buffers.  This will likely be included in comm_mesh/etc/rc.local, but it's here if you need to run it for testing on other systems.
sudo sysctl -w net.ipv4.udp_mem="102400 873800 16777216"
sudo sysctl -w net.core.netdev_max_backlog="30000"
sudo sysctl -w net.core.rmem_max="16777216"
sudo sysctl -w net.core.wmem_max="16777216"
