#! /bin/bash

INPUT_DATA_DIR=/media/files/darpa/only_third/hdf_data/join2_hist3_batch4_skip_succ_deg
OUTPUT_SHUFF_DATA_DIR=${INPUT_DATA_DIR}_shuffled
OUTPUT_SHUFF_CLS_DATA_DIR=${INPUT_DATA_DIR}_shuffled_classes
MATYLDA_SHUFF_OUT_DIR=/mnt/matylda1/ivelas/darpa/only_third/hdf_data/$(basename $INPUT_DATA_DIR)_shuffled
MATYLDA_SHUFF_CLS_OUT_DIR=/mnt/matylda1/ivelas/darpa/only_third/hdf_data/$(basename $INPUT_DATA_DIR)_shuffled_classes

SCRIPTS_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
POSES=/media/files/darpa/only_third/poses

mkdir -p $OUTPUT_SHUFF_DATA_DIR $OUTPUT_SHUFF_CLS_DATA_DIR
#rm -f $INPUT_DATA_DIR/*.shuffled $OUTPUT_SHUFF_DATA_DIR/* $OUTPUT_SHUFF_CLS_DATA_DIR/*
#ssh merlin "mkdir -p $MATYLDA_SHUFF_OUT_DIR; mkdir -p $MATYLDA_SHUFF_CLS_OUT_DIR; rm -f $MATYLDA_SHUFF_OUT_DIR/* $MATYLDA_SHUFF_CLS_OUT_DIR/*"

for i in $INPUT_DATA_DIR/d3*.hdf5; do
	echo $i

	cat $POSES/*.txt | $SCRIPTS_DIR/kitti_poses_rot_histogram.py $i
	shuff_target=$OUTPUT_SHUFF_DATA_DIR/$(basename $i)
	mv $i.shuffled $shuff_target
	

	$SCRIPTS_DIR/kitti_poses_rot_quantization.py $shuff_target
	(
		scp $shuff_target merlin:$MATYLDA_SHUFF_OUT_DIR 
		scp $shuff_target.rotclasses merlin:$MATYLDA_SHUFF_CLS_OUT_DIR
		rm $shuff_target $shuff_target.rotclasses
	) &
done

wait
