#! /bin/bash

DEPLOY_DEF=${1:-/home/ivelas/workspace/ivelas-git/sw/cnn-generator/bash-definitions/odometry_l335_comp_rotation_deploy.sh}
NET_LABEL=${2:-odometry_l335_comp_rx_b4_shifts15}
TRAIN_ITERATIONS=${3:-200000}
BATCH_SIZE=${4:-4}
SHIFTS=${5:-15}
MAX_ANGLE=${6:-1.5}
ANGLE_ID=${7:-0}


CNN_GENERATOR_DIR=/home/ivelas/workspace/ivelas-git/sw/cnn-generator
TRAINED_MODEL=$CNN_GENERATOR_DIR/NETS/$NET_LABEL/net_snapshot_iter_$TRAIN_ITERATIONS.caffemodel
OUTPUT_DIR_BASE=/media/files/cnn_velodyne_data/results/
OUTPUT_DIR=$OUTPUT_DIR_BASE/$(($(ls $OUTPUT_DIR_BASE | egrep '^[[:digit:]]+' -o | sort -g | tail -n1)+1))-$NET_LABEL-it$TRAIN_ITERATIONS-window
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
POSES_DIR=/media/files/cnn_velodyne_data/poses
CAFFE=~/lib/caffe/build/tools/caffe

mkdir -p $OUTPUT_DIR

cp $DEPLOY_DEF $TRAINED_MODEL $OUTPUT_DIR

for seq in 00 01 02 03 04 05 06 07 08 09 10
do
	DEPLOY_PROTOTXT=$CNN_GENERATOR_DIR/deploy-definitions/$(basename $DEPLOY_DEF).prototxt
	OUTPUT_FILE=$OUTPUT_DIR/$seq.txt
	bash $DEPLOY_DEF $seq $OUTPUT_FILE $BATCH_SIZE $SHIFTS $MAX_ANGLE $ANGLE_ID | $CNN_GENERATOR_DIR/buildNetwork.sh > $DEPLOY_PROTOTXT
	expected_poses=$(wc -l < $POSES_DIR/$seq.txt)
	iterations=$((($expected_poses-2+$BATCH_SIZE)/$BATCH_SIZE))
	$CAFFE test -model $DEPLOY_PROTOTXT -iterations $iterations -weights $TRAINED_MODEL -gpu 0
	head -n $expected_poses $OUTPUT_FILE > out_tmp; mv out_tmp $OUTPUT_FILE
done |& tee $OUTPUT_DIR/forward.log
