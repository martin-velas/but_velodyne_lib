#! /bin/bash

CNN_GEN=/home/ivelas/workspace/ivelas-git/sw/cnn-generator
NETS=$CNN_GEN/NETS
DEPLOYS=$CNN_GEN/deploy-definitions
BATCH_SIZE=4
LAST_ITERATION=500000
OUTPUT_DIR_BASE=/media/files/cnn_velodyne_data/results/forpaper
POSES_DIR=/media/files/cnn_velodyne_data/poses
CAFFE=~/lib/caffe/build/tools/caffe
DATA_BASEDIR=/media/matylda1/cnn_velodyne_data/hdf_data/j2_h2_forpaper/h3_skip_and_succ/

for d in $NETS/odometry_l335_b4_j2_h3_skip_and_succ_transl_noshuffle # $NETS/odometry_l335_b4_j2_h3_skip_and_succ_transl_notrack
do
	label=$(basename $d)
	hsize=$(grep -o '_h[0-9]*_' <<<$label | grep -o '[0-9]*')
	deploy_proto=$DEPLOYS/odometry_l335_b4_j2_h3_skip_and_succ_transl_deploy.prototxt

	best_iteration=$(grep 'Test.*loss\|Iteration.*Testing' $d/caffe_train.log | egrep -o 'Test.*|Iteration.*' | tr '\n' ' ' | sed 's/loss)/loss)\n/g' |
		sed 's/[^0-9\.]\+/ /g' | cut -d" " -f2,5 | grep '.\+' | sort -k2 -t" " | head -n1 | cut -d" " -f1)
	if [ $best_iteration != $LAST_ITERATION ]; then
		iterations="$best_iteration $LAST_ITERATION"
	else
		iterations=$LAST_ITERATION
	fi

	for it in $iterations
	do
		model=$d/net_snapshot_iter_$it.caffemodel
		out_dir=$OUTPUT_DIR_BASE/$(($(ls $OUTPUT_DIR_BASE | egrep '^[[:digit:]]+' -o | sort -g | tail -n1)+1))-$label-it$it
		echo $d $label $hsize $it $(ls $deploy_proto $model)

		mkdir -p $out_dir
		cp $deploy_proto $model $out_dir

		for seq in 00 01 02 03 04 05 06 07 08 09 10
		do
			expected_poses=$(wc -l < $POSES_DIR/$seq.txt)
			test_iterations=$((($expected_poses-2+$BATCH_SIZE)/$BATCH_SIZE))

			ls $DATA_BASEDIR/train/$seq* $DATA_BASEDIR/test/$seq* | sort --version-sort > test.files

			head -n $hsize $POSES_DIR/$seq.txt > init.poses

			time $CAFFE test -model $deploy_proto -iterations $test_iterations -weights $model -gpu 0
			head -n $expected_poses output.poses > $out_dir/$seq.txt
		done |& tee $out_dir/forward.log
	done
done
