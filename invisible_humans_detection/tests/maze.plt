unset border
unset tics
set samples 15
set lmargin at screen 0
set rmargin at screen 1
set bmargin at screen 0
set tmargin at screen 1
set xrange[-3.59808:3.59808]
set yrange[-3.5:3.5]
set term pngcairo mono enhanced size 215,210
set output 'maze.png'
set multiplot
set arrow from -0.866025,-2.500000 to 0.000000,-2.000000 nohead lc'black' lw 2
set arrow from -1.732051,-1.000000 to -1.732051,-2.000000 nohead lc'black' lw 2
set arrow from -0.000000,-2.000000 to 0.866025,-2.500000 nohead lc'black' lw 2
set arrow from 0.866025,-2.500000 to 1.732051,-2.000000 nohead lc'black' lw 2
set arrow from 1.732051,-2.000000 to 1.732051,-1.000000 nohead lc'black' lw 2
set arrow from -0.000000,-1.000000 to 0.000000,-2.000000 nohead lc'black' lw 2
set arrow from -2.598076,-0.500000 to -1.732051,-1.000000 nohead lc'black' lw 2
set arrow from -1.732051,1.000000 to -2.598076,0.500000 nohead lc'black' lw 2
set arrow from -2.598076,0.500000 to -2.598076,-0.500000 nohead lc'black' lw 2
set arrow from -0.866025,-0.500000 to -0.000000,-1.000000 nohead lc'black' lw 2
set arrow from -0.866025,0.500000 to -0.866025,-0.500000 nohead lc'black' lw 2
set arrow from 0.866025,-0.500000 to 1.732051,-1.000000 nohead lc'black' lw 2
set arrow from 1.732051,-1.000000 to 2.598076,-0.500000 nohead lc'black' lw 2
set arrow from 2.598076,-0.500000 to 2.598076,0.500000 nohead lc'black' lw 2
set arrow from 2.598076,0.500000 to 1.732051,1.000000 nohead lc'black' lw 2
set arrow from -0.866025,0.500000 to 0.000000,1.000000 nohead lc'black' lw 2
set arrow from 0.000000,2.000000 to -0.866025,2.500000 nohead lc'black' lw 2
set arrow from -0.866025,2.500000 to -1.732051,2.000000 nohead lc'black' lw 2
set arrow from -1.732051,2.000000 to -1.732051,1.000000 nohead lc'black' lw 2
set arrow from 0.866025,0.500000 to 1.732051,1.000000 nohead lc'black' lw 2
set arrow from 1.732051,1.000000 to 1.732051,2.000000 nohead lc'black' lw 2
set arrow from 0.866025,2.500000 to 0.000000,2.000000 nohead lc'black' lw 2
plot 1/0 notitle
unset multiplot
set output
