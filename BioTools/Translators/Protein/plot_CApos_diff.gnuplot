
#set title "Displacement of the residues along the conformational transition"
set xlabel "Iteration number" "Times-Roman,20" 
set ylabel "Residue number" font "Times-Roman,20" 
set xtics font "Times-Roman,18"            
set ytics font "Times-Roman,18"            
set xrange [ 1 : 10 ] 
set yrange [ 1 : 214 ]
set pm3d map
set palette gray negative
splot "CApos_diff.output.iters" title ""
#set terminal png
#set output "CApos_diff.png"
set terminal postscript eps 'Times-Roman' 18     
set output "CApos_diff.eps" 
replot

