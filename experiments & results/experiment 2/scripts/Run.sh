#!/bin/bash
# ########### In the name of Allah ##################
# Author: Mohammad H. Daei                          #
# Description: Second experimet                     #
#              Throughput vs. A-MPDU size diagram   #
#              Waiting Time vs. A-MPDU size diagram #
# Execute as sudo                                   #
# ###################################################


clear

# MAC address & IP Address of iperf server
iperf_server="10.42.0.87"
mac_address="48:d2:24:51:3c:f8"

# Maximum number of MPDUs in the experiment
max_agg_eval=20

# Desired Access Category (VO, VI, BE, BK)
AC=BE

# DebugFS address might be diffrenet in your system
netdev="netdev:wlan0"
path="/sys/kernel/debug/ieee80211/phy0/"$netdev"/stations/"$mac_address

# Running iperf
function iperf_func() {
	iperf -c $iperf_server -u -n 500M -b 100M >> logs/iperf.txt
	echo -n "1" > flag.tmp
}

# Removing previous reports & logs
rm -f "waiting_times.txt"
rm -rf logs
mkdir -p logs

echo "Start..."$'\n'

# Changing A-MPDU size from 0 to max_agg_eval
for ((i = 1; i <= $max_agg_eval; i++))
do
	# Passing the parameter to the ath9k driver
	echo $i > $path/set_max_agg
	sleep 0.01
	
	# Displaying percentage of the experiment progress
	echo -n $'\b\b\b\b\b\b\b\b'
	echo -n "$i/$max_agg_eval"
	
	echo -n "nframes: " >> logs/iperf.txt
	echo $i >> logs/iperf.txt
	echo -n $i $'\t' >> waiting_times.txt
	
	echo -n "0" > flag.tmp
	flag=0
	
	#Thread 1 - iperf
	iperf_func &
	
	#Thread 2 - Recording waiting times
	while [ $flag -ne 1 ]
	do
		grep $AC  $path/waiting_times | awk '{ print $4 }' >> logs/waiting_times_$i.txt
		sleep 0.01
		flag=`cat flag.tmp`
	done
	
	wait
	
	awk -f sc/average_waiting_time.awk logs/waiting_times_$i.txt >> waiting_times.txt
	
	echo $'\n'"******************************"$'\n' >> logs/iperf.txt
	
	sleep 5
done

# Processing the Throughput values
grep "%)" logs/iperf.txt > tmp.txt
awk -f sc/filter_bw_awk.awk tmp.txt > throughput.txt

# Changing A-MPDU size to default value
echo "0" > $path/set_max_agg

# Drawing
gnuplot sc/waiting_time_plot.gp
gnuplot sc/waiting_time_plot_smooth.gp
gnuplot sc/throughput_plot.gp
gnuplot sc/throughput_plot_smooth.gp

# Changing file mode - Because this script has been run as sudo
chmod o+rw -R "logs"
chmod o+rw "throughput.txt" "throughput-agg.png" "throughput-agg(smooth).png"
chmod o+rw "waiting_times.txt" "waiting_time-agg.png" "waiting_time-agg(smooth).png"

# Removing temporary files
rm -f flag.tmp
rm tmp.txt

echo $'\n'$'\n'"Done"$'\n'
