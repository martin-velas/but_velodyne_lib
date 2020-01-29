#! /bin/bash

SCRIPTS_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

BP_EVAL_FILE=/media/kitti/data_tracking_velodyne_BP_out/BP_evaluation.output

function get_title {
	echo $1 | rev | cut -d'/' -f1-3 | rev |
		sed 's#ground_\([^/]*\)/#\1#' |
		sed 's#run01-true-ann-only/#(ann-trained#' |
		sed 's#run02-fake-ann-only/#(fake-trained#' |
		sed 's#run03-fake-pretrained/#(pretrained#' |
		sed 's#evaluation-masked-output.txt#,masked)#' |
		sed 's#evaluation-output.txt#)#' | 
		sed 's#.*apriori-prob.*#Baseline#' |
		sed "s/_/-/g"
}

function handle_zero {
	sed 's/^0 /0.0001 /'
}

bp_pr_data=$(mktemp)
grep '@ 1' $BP_EVAL_FILE | cut -d" " -f7-8 > $bp_pr_data
plotline="plot '$bp_pr_data' using 1:2 with points pt 2 lc rgb \"red\" title '[Zhang15]',"

for eval in "$@"; do
	pr_data=$(mktemp --suffix=.pr)
	pr_data_files="$pr_data $pr_data_files"
	grep '^@' $eval | cut -d" " -f7-8 > $pr_data
	
	nan_data=$(grep -i "nan" < $eval)
	if [ "x$nan_data" != "x" ]; then
		echo "Warning NaN data: $nan_data" >&2
	fi

	plotline="$plotline '$pr_data' using 1:2 with lines title '$(get_title $eval)',"
done

name="ground_pr_curve"
esc_name=$(echo $name | sed "s/_/-/g")
gnuplot <<< "set terminal postscript enhanced color  
             set output '$name.ps'
             set xrange [0.9:1]
             set yrange [0.9:1]
             set xtics 0.02
             set ytics 0.02
             set size ratio -1
             yoffset=5
             set key inside bottom left
             set xlabel 'Precision'
             set ylabel 'Recall'
             $plotline"

ps2pdf $name.ps $name.pdf

rm $name.ps $bp_pr_data
rm $pr_data_files
