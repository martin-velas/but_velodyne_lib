#! /bin/bash

INPUT_DATA_DIR=/media/files/cnn_velodyne_data/hdf_data/j2_h3_b16_skip_succ_deg_rotclasses_kittitrfix
SUFFIX=shuffled
MATYLDA_OUT_DIR=/mnt/matylda1/ivelas/cnn_velodyne_data/hdf_data/$(basename $INPUT_DATA_DIR)_${SUFFIX}

SCRIPTS_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

rm -f $INPUT_DATA_DIR/*.shuffled
ssh merlin "mkdir -p $MATYLDA_OUT_DIR; rm -f $MATYLDA_OUT_DIR/*"

for i in $INPUT_DATA_DIR/*.hdf5; do
	echo "Shuffling file $i"
	$SCRIPTS_DIR/kitti_poses_rot_histogram.py $i
	(scp $i.shuffled merlin:$MATYLDA_OUT_DIR/$(basename $i); rm $i.shuffled) &
done

wait
