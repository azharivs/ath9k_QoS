#!/bin/bash
# ########### In the name of Allah ##############
# Author: Mohammad H. Daei                      #
# Description: Experimet #1                     #
#              Changing A-MPDU size & plotting  #
#              the Throughput vs. A-MPDU size   #
# Execute as sudo                               #
# ###############################################


clear

# MAC address & IP Address of iperf server
iperf_server="192.168.43.1"
mac_address="10:92:66:69:b5:cc"

# Maximum number of MPDUs in the experiment
max_agg_eval=20

# DebugFS address might be diffrenet in your system
netdev="netdev:wlan2"
path="/sys/kernel/debug/ieee80211/phy0/"$netdev"/stations/"$mac_address

# Removing previous reports & logs
rm -f Exp1_Log.txt

echo "Start..."$'\n'

for ((i = 1; i <= $max_agg_eval; i++))
do
	echo $i > $path/set_max_agg
	sleep 0.01

	# Displaying percentage of the experiment progress
	echo -n $'\b\b\b\b\b\b\b\b'
	echo -n "$i/$max_agg_eval"
	
	echo -n "nframes: " >> Exp1_Log.txt
	echo $i >> Exp1_Log.txt
	iperf -c $iperf_server -u -n 500M -b 100M >> Exp1_Log.txt
	
	echo $'\n'"******************************"$'\n' >> Exp1_Log.txt
	
	sleep 5
done

# Processing the data from the log
grep "%)" Exp1_Log.txt > tmp.txt
awk -f filter_bw_awk.awk tmp.txt > throughput.txt

echo "0" > $path/set_max_agg
rm tmp.txt

# Drawing diagrams
gnuplot plot.gp
gnuplot plot_smooth.gp

chmod o+rw "Exp1_Log.txt" "throughput.txt"
chmod o+rw "agg-throughput.png" "agg-throughput(smooth).png"

echo $'\n'$'\n'"Done"$'\n'
