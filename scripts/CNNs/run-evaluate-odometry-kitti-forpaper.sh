#! /bin/bash

POSES_DIR=/media/files/cnn_velodyne_data/poses/
EMPTY_EVAL=/media/files/cnn_velodyne_data/results/empty-evaluation
RES_DIR=/media/files/cnn_velodyne_data/results

for transl in $RES_DIR/forpaper/46-* $RES_DIR/forpaper/48-*
do
	for rot in $RES_DIR/forpaper/*$(basename $transl | sed 's/_transl.*//' | grep -o 'odometry.*')_rot* $POSES_DIR
	do
		eval_dir=$transl/evaluation-with-$(basename $rot)

		cp -r $EMPTY_EVAL $eval_dir

		for i in 00.txt  01.txt  02.txt  03.txt  04.txt  05.txt  06.txt  07.txt  08.txt  09.txt  10.txt
		do 
			echo "Merging $OUT_FOLDER/$i"
			~/workspace/but_velodyne_lib/scripts/merge_pose_files.py -t $transl/$i -r $rot/$i > $eval_dir/results/ivelas/data/$i
		done

		pushd $eval_dir
			~/workspace/tmp-ivelas/sw/kitti-odometry-devkit/evaluate_odometry ivelas
		popd
		$BUT_VELODYNE_LIB/scripts/kitti_test_stats.py < $eval_dir/results/ivelas/plot_error/average_te.txt | tee $eval_dir/results/ivelas/stats-test.txt
	done
done
