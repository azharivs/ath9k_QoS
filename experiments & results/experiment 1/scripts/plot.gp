set title "A-MPDU"
set xlabel "Number of MPDUs"
set ylabel "Throughput (Mbps)"

set xrange [1:20]

set terminal png size 800,600
set output "agg-throughput.png"

plot	"throughput.txt" using 1:2 title "A-MPDU" with line lc rgb 'blue'