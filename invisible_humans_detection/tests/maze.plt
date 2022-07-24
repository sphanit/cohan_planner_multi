unset border
unset tics
set samples 15
set lmargin at screen 0
set rmargin at screen 1
set bmargin at screen 0
set tmargin at screen 1
set xrange[-1:6]
set yrange[-1:6]
set term pngcairo mono enhanced size 210,210
set output 'maze.png'
set multiplot
set arrow from 0.000000,0.000000 to 1.000000,0.000000 nohead lc'black' lw 2
set arrow from 5.000000,0.000000 to 5.000000,1.000000 nohead lc'black' lw 2
set arrow from 1.000000,0.000000 to 2.000000,0.000000 nohead lc'black' lw 2
set arrow from 2.000000,0.000000 to 3.000000,0.000000 nohead lc'black' lw 2
set arrow from 3.000000,0.000000 to 4.000000,0.000000 nohead lc'black' lw 2
set arrow from 4.000000,0.000000 to 5.000000,0.000000 nohead lc'black' lw 2
set arrow from 0.000000,1.000000 to 0.000000,2.000000 nohead lc'black' lw 2
set arrow from 5.000000,1.000000 to 5.000000,2.000000 nohead lc'black' lw 2
set arrow from 0.000000,1.000000 to 1.000000,1.000000 nohead lc'black' lw 2
set arrow from 1.000000,1.000000 to 2.000000,1.000000 nohead lc'black' lw 2
set arrow from 2.000000,1.000000 to 3.000000,1.000000 nohead lc'black' lw 2
set arrow from 4.000000,1.000000 to 5.000000,1.000000 nohead lc'black' lw 2
set arrow from 0.000000,2.000000 to 0.000000,3.000000 nohead lc'black' lw 2
set arrow from 5.000000,2.000000 to 5.000000,3.000000 nohead lc'black' lw 2
set arrow from 1.000000,2.000000 to 1.000000,3.000000 nohead lc'black' lw 2
set arrow from 2.000000,2.000000 to 2.000000,3.000000 nohead lc'black' lw 2
set arrow from 3.000000,2.000000 to 3.000000,3.000000 nohead lc'black' lw 2
set arrow from 4.000000,2.000000 to 4.000000,3.000000 nohead lc'black' lw 2
set arrow from 0.000000,3.000000 to 0.000000,4.000000 nohead lc'black' lw 2
set arrow from 5.000000,3.000000 to 5.000000,4.000000 nohead lc'black' lw 2
set arrow from 1.000000,3.000000 to 1.000000,4.000000 nohead lc'black' lw 2
set arrow from 1.000000,3.000000 to 2.000000,3.000000 nohead lc'black' lw 2
set arrow from 3.000000,3.000000 to 3.000000,4.000000 nohead lc'black' lw 2
set arrow from 3.000000,3.000000 to 4.000000,3.000000 nohead lc'black' lw 2
set arrow from 4.000000,3.000000 to 5.000000,3.000000 nohead lc'black' lw 2
set arrow from 0.000000,5.000000 to 1.000000,5.000000 nohead lc'black' lw 2
set arrow from 0.000000,4.000000 to 0.000000,5.000000 nohead lc'black' lw 2
set arrow from 1.000000,5.000000 to 2.000000,5.000000 nohead lc'black' lw 2
set arrow from 1.000000,4.000000 to 2.000000,4.000000 nohead lc'black' lw 2
set arrow from 2.000000,5.000000 to 3.000000,5.000000 nohead lc'black' lw 2
set arrow from 2.000000,4.000000 to 2.000000,5.000000 nohead lc'black' lw 2
set arrow from 3.000000,5.000000 to 4.000000,5.000000 nohead lc'black' lw 2
set arrow from 3.000000,4.000000 to 4.000000,4.000000 nohead lc'black' lw 2
set arrow from 4.000000,5.000000 to 5.000000,5.000000 nohead lc'black' lw 2
plot 1/0 notitle
unset multiplot
set output
