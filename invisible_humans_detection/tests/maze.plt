unset border
unset tics
set samples 15
set lmargin at screen 0
set rmargin at screen 1
set bmargin at screen 0
set tmargin at screen 1
set xrange[-4:4]
set yrange[-4:4]
set term pngcairo mono enhanced size 240,240
set output 'maze.png'
set multiplot
set parametric; plot [-1.745329:-1.396263] 0.000000+cos(t)*3.000000,0.000000+sin(t)*3.000000 w l lc'black' lw 2 notitle;unset parametric
set parametric; plot [-1.396263:-1.047198] 0.000000+cos(t)*3.000000,0.000000+sin(t)*3.000000 w l lc'black' lw 2 notitle;unset parametric
set parametric; plot [-2.094395:-1.047198] 0.000000+cos(t)*1.000000,0.000000+sin(t)*1.000000 w l lc'black' lw 2 notitle;unset parametric
set arrow from 0.500000,-0.866025 to 0.000000,-2.000000 nohead lc'black' lw 2
set arrow from 1.000000,-1.732051 to 0.520945,-2.954423 nohead lc'black' lw 2
set arrow from 0.500000,-0.866025 to 1.000000,-1.732051 nohead lc'black' lw 2
set parametric; plot [-1.047198:-0.698132] 0.000000+cos(t)*3.000000,0.000000+sin(t)*3.000000 w l lc'black' lw 2 notitle;unset parametric
set parametric; plot [-0.698132:-0.349066] 0.000000+cos(t)*3.000000,0.000000+sin(t)*3.000000 w l lc'black' lw 2 notitle;unset parametric
set parametric; plot [-0.349066:0.000000] 0.000000+cos(t)*3.000000,0.000000+sin(t)*3.000000 w l lc'black' lw 2 notitle;unset parametric
set parametric; plot [-1.047198:0.000000] 0.000000+cos(t)*1.000000,0.000000+sin(t)*1.000000 w l lc'black' lw 2 notitle;unset parametric
set arrow from 1.000000,0.000000 to 1.732051,-1.000000 nohead lc'black' lw 2
set parametric; plot [0.000000:0.349066] 0.000000+cos(t)*3.000000,0.000000+sin(t)*3.000000 w l lc'black' lw 2 notitle;unset parametric
set parametric; plot [0.349066:0.698132] 0.000000+cos(t)*3.000000,0.000000+sin(t)*3.000000 w l lc'black' lw 2 notitle;unset parametric
set parametric; plot [0.698132:1.047198] 0.000000+cos(t)*3.000000,0.000000+sin(t)*3.000000 w l lc'black' lw 2 notitle;unset parametric
set arrow from 0.500000,0.866025 to 1.732051,1.000000 nohead lc'black' lw 2
set arrow from 2.000000,0.000000 to 2.819078,1.026060 nohead lc'black' lw 2
set parametric; plot [0.523599:1.047198] 0.000000+cos(t)*2.000000,0.000000+sin(t)*2.000000 w l lc'black' lw 2 notitle;unset parametric
set arrow from 1.000000,1.732051 to 2.298133,1.928363 nohead lc'black' lw 2
set arrow from 0.000000,0.000000 to 0.500000,0.866025 nohead lc'black' lw 2
set parametric; plot [1.396263:1.745329] 0.000000+cos(t)*3.000000,0.000000+sin(t)*3.000000 w l lc'black' lw 2 notitle;unset parametric
set parametric; plot [1.745329:2.094395] 0.000000+cos(t)*3.000000,0.000000+sin(t)*3.000000 w l lc'black' lw 2 notitle;unset parametric
set arrow from -0.500000,0.866025 to 0.000000,2.000000 nohead lc'black' lw 2
set parametric; plot [2.094395:2.443461] 0.000000+cos(t)*3.000000,0.000000+sin(t)*3.000000 w l lc'black' lw 2 notitle;unset parametric
set parametric; plot [2.443461:2.792527] 0.000000+cos(t)*3.000000,0.000000+sin(t)*3.000000 w l lc'black' lw 2 notitle;unset parametric
set parametric; plot [2.792527:3.141593] 0.000000+cos(t)*3.000000,0.000000+sin(t)*3.000000 w l lc'black' lw 2 notitle;unset parametric
set parametric; plot [2.094395:3.141593] 0.000000+cos(t)*1.000000,0.000000+sin(t)*1.000000 w l lc'black' lw 2 notitle;unset parametric
set arrow from -1.000000,0.000000 to -1.732051,1.000000 nohead lc'black' lw 2
set parametric; plot [2.094395:2.617994] 0.000000+cos(t)*2.000000,0.000000+sin(t)*2.000000 w l lc'black' lw 2 notitle;unset parametric
set arrow from -1.732051,1.000000 to -2.819078,1.026060 nohead lc'black' lw 2
set arrow from -2.000000,0.000000 to -2.819078,1.026060 nohead lc'black' lw 2
set arrow from -0.500000,-0.866025 to -1.000000,-1.732051 nohead lc'black' lw 2
set parametric; plot [3.141593:3.490659] 0.000000+cos(t)*3.000000,0.000000+sin(t)*3.000000 w l lc'black' lw 2 notitle;unset parametric
set parametric; plot [3.490659:3.839724] 0.000000+cos(t)*3.000000,0.000000+sin(t)*3.000000 w l lc'black' lw 2 notitle;unset parametric
set parametric; plot [3.839724:4.188790] 0.000000+cos(t)*3.000000,0.000000+sin(t)*3.000000 w l lc'black' lw 2 notitle;unset parametric
set arrow from -0.500000,-0.866025 to -1.732051,-1.000000 nohead lc'black' lw 2
plot 1/0 notitle
unset multiplot
set output
