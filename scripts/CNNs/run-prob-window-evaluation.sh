#! /bin/bash

SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
ROT_DATA=/media/files/cnn_velodyne_data/results/30-odometry_l335_h6_b16_rot_shuffled_eulerfix_II-it500k

if [ $# -ne 3 ]; then
	echo "ERROR, usage: $0 <dir> <axis-idx> <max-angle>" >&2
	exit 1
fi

DIR=$1
AXIS=$2
MAX_ANGLE=$3

classes=$(head -n1 $DIR/00.*probabilities | wc -w)
for win in $(seq $classes)
do
	win_dir=$DIR/win-$win
	mkdir -p $win_dir
	for prob_file in $DIR/*.probabilities
	do
		pose_file=$win_dir/$(basename $prob_file | grep '[[:digit:]]\+' -o).txt
		echo "Creating $pose_file"
		$SCRIPT_DIR/probabilities_to_poses.py -i $AXIS -c $classes -m $MAX_ANGLE -w $win < $prob_file > $pose_file
	done

	cmd="$SCRIPT_DIR/run-evaluate-rot-odometry-kitti.sh $win_dir"
	for i in 0 1 2
	do
		if [ $i -eq $AXIS ]; then
			cmd="$cmd $win_dir"
		else
			cmd="$cmd $ROT_DATA"
		fi
	done
	$cmd
done
