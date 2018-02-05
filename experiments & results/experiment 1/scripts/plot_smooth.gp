set title "A-MPDU"
set xlabel "Number of MPDUs"
set ylabel "Throughput (Mbps)"

set xrange [1:20]

set terminal png size 800,600
set output "agg-throughput(smooth).png"

plot	"throughput.txt" using 1:2 smooth bezier title "A-MPDU(Smooth)" with line lc rgb 'blue'