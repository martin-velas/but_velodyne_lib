#! /bin/bash

SCRIPTS_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

BP_EVAL_FILE=/media/kitti/data_tracking_velodyne_BP_out/BP_evaluation.output

function handle_zero {
	sed 's/^0 /0.0001 /'
}

if [ $# -ne 1 ]; then
	echo "ERROR, usage: ./plot-ground-nice-pr-curve.sh <graph-spec-file>" >&2
	exit 1
fi

bp_pr_data=$(mktemp)
grep '@ 1' $BP_EVAL_FILE | cut -d" " -f7-8 > $bp_pr_data
plotline="plot '$bp_pr_data' using 1:2 with points pt 2 lw 5 ps 2 lc rgb \"red\" title '[Zhang15]',"
#plotline="plot "

while read specline
do
	title=$(awk '{print $1}'<<< $specline)
	color=$(awk '{print $2}'<<< $specline)
	linetype=$(awk '{print $3}'<<< $specline)
	eval_file=$(awk '{print $4}'<<< $specline)
	echo $title, $color, $linetype, $eval_file

	pr_data=$(mktemp --suffix=.pr)
	pr_data_files="$pr_data $pr_data_files"
	grep '^@' $eval_file | cut -d" " -f7-8 > $pr_data
	
	nan_data=$(grep -i "nan" < $eval_file)
	if [ "x$nan_data" != "x" ]; then
		echo "Warning NaN data: $nan_data" >&2
	fi

	plotline="$plotline '$pr_data' using 1:2 with lines lc rgb \"$color\" lw 5 dashtype $linetype title sprintf(\"$title\"),"
#	plotline="$plotline '$pr_data' using 1:2 with lines lc rgb \"$color\" lw 5 dashtype $linetype notitle,"
done < $1

name="ground_pr_curve"
esc_name=$(echo $name | sed "s/_/-/g")
gnuplot <<< "set terminal postscript enhanced color font 'Helvetica,20'
             set output '$name.ps'
             set xrange [0.9:1]
             set yrange [0.9:1]
             set xtics 0.02
             set ytics 0.02
             set size ratio -1
             set key inside bottom left nobox
             set key out vert top right
             set key spacing 1.5
             set xlabel 'Precision' offset 0,0
             set ylabel 'Recall' offset 2,0
             set arrow from graph 0,first 0.9924 to graph 1,first 0.9924 nohead lc rgb '#ff0000' lw 1 front
             set arrow from first 0.9243,graph 0 to first 0.9243,graph 1 nohead lc rgb '#0000ff' lw 1 front
             $plotline"

ps2pdf $name.ps $name.pdf
pdfcrop $name.pdf

rm $name.ps $bp_pr_data
rm $pr_data_files
