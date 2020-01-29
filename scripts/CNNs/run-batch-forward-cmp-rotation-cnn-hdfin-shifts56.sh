#! /bin/bash

DEPLOY_PROTOTXT=${1:-/home/ivelas/workspace/ivelas-git/sw/cnn-generator/deploy-definitions/odometry_l335_b1_shifts56_rcls_comp_deploy_fixshift_hdfin.prototxt}
NET_LABEL=${2:-odometry_l335_b1_shifts56_rcls_comp_fixshift_eulerfix}
INPUT_FILE_LISTS=${3:-/media/files/cnn_velodyne_data/hdf_data/j2_h1_b3_rad_1800d_rcls56_eulerfix/filelists}

TRAIN_ITERATIONS=${4:-200000}

CNN_GENERATOR_DIR=/home/ivelas/workspace/ivelas-git/sw/cnn-generator
TRAINED_MODEL=$CNN_GENERATOR_DIR/NETS/$NET_LABEL/net_snapshot_iter_$TRAIN_ITERATIONS.caffemodel
OUTPUT_DIR_BASE=/media/files/cnn_velodyne_data/results/forpaper
OUTPUT_DIR=$OUTPUT_DIR_BASE/$(($(ls $OUTPUT_DIR_BASE | egrep '^[[:digit:]]+' -o | sort -g | tail -n1)+1))-$NET_LABEL-it$TRAIN_ITERATIONS-win3
POSES_DIR=/media/files/cnn_velodyne_data/poses
CAFFE=~/lib/caffe/build/tools/caffe

mkdir -p $OUTPUT_DIR

cp $DEPLOY_PROTOTXT $TRAINED_MODEL $OUTPUT_DIR

#for seq in 00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21
for seq in 11 12 13 14 15 16 17 18 19 20 21
do
	OUTPUT_FILE=$OUTPUT_DIR/$seq.txt
	expected_poses=$(wc -l < $POSES_DIR/$seq.txt)
	iterations=$(($expected_poses-1))
	pushd $OUTPUT_DIR
		cp $INPUT_FILE_LISTS/$seq.files test.files
		time $CAFFE test -model $DEPLOY_PROTOTXT -iterations $iterations -weights $TRAINED_MODEL -gpu all
		mv out.probabilities $seq.probabilities
		mv out.poses $seq.txt
	popd
	head -n $expected_poses $OUTPUT_FILE > out_tmp; mv out_tmp $OUTPUT_FILE
done |& tee $OUTPUT_DIR/forward.log
