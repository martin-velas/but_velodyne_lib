#! /bin/bash

EMPTY_EVAL=/media/files/cnn_velodyne_data/results/empty-evaluation
RES_DIR=/media/files/cnn_velodyne_data/results/forpaper


function evaluate {
	t=$1
	rx=$2
	ry=$3
	rz=$4
	eval_dir=$5

	cp -r $EMPTY_EVAL $eval_dir

	for i in 00.txt  01.txt  02.txt  03.txt  04.txt  05.txt  06.txt  07.txt  08.txt  09.txt  10.txt
	do
		~/workspace/but_velodyne_lib/scripts/merge_pose_files.py -t $t/$i -rx $rx/$i -ry $ry/$i -rz $rz/$i > $eval_dir/results/ivelas/data/$i
	done

	pushd $eval_dir
		~/workspace/tmp-ivelas/sw/kitti-odometry-devkit/evaluate_odometry ivelas
	popd

	echo -n "Test stats: "
	~/workspace/but_velodyne_lib/scripts/kitti_test_stats.py < $eval_dir/results/ivelas/plot_error/average_te.txt | tee $eval_dir/results/ivelas/stats-test.txt
}


OUT_FOLDER=$RES_DIR/49-joined-46transl-33rx-31ry-34-rz-wineval

TRANSL_DATA=$RES_DIR/46-odometry_l335_convout_b4_j2_h5_skip_and_succ_transl-it500000
ROT_X_DATA=$RES_DIR/../33-odometry_l335_rx_comp_hdfin_eulerfix-it200000-window
ROT_Y_DATA=$RES_DIR/../31-odometry_l335_b1_shifts56_rcls_comp_fixshift_eulerfix-it200000-window
ROT_Z_DATA=$RES_DIR/../34-odometry_l335_rz_comp_hdfin_eulerfix-it200000-window

cp $0 $OUT_FOLDER

for i in $(seq 13)
do
	evaluate $TRANSL_DATA $ROT_X_DATA/win-$i $ROT_Y_DATA/win-$i $ROT_Z_DATA/win-$i $OUT_FOLDER/evaluation-win-$i
done

evaluate $TRANSL_DATA $ROT_X_DATA/win-13 $ROT_Y_DATA/win-13 $ROT_Z_DATA/win-13 $OUT_FOLDER/evaluation-full
