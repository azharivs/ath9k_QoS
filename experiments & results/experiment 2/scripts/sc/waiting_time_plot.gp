set title "A-MPDU"
set xlabel "Number of MPDUs"
set ylabel "Waiting Time (ms)"

set xrange [1:20]

set terminal png size 800,600
set output "waiting_time-agg.png"

plot	"waiting_times.txt" using 1:2 title "A-MPDU" with line lc rgb 'blue'