#! /bin/bash

export DATA_IN_LABEL=joined2_hist3_batch4_skipped_and_succ_deg_aligned_shuffled_balanced

export DATA_OUT_LABEL=${DATA_IN_LABEL}_split
export HDF_FILES_DIR=/media/kitti/dataset_odometry_velodyne_odom_cnn_data/$DATA_IN_LABEL
export MATYLDA_OUT_DIR=/mnt/matylda1/ivelas/kitti/dataset_odometry_velodyne_odom_cnn_data/$DATA_OUT_LABEL
export SCRIPTS_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

ssh merlin "mkdir -p $MATYLDA_OUT_DIR; rm -f $MATYLDA_OUT_DIR/*"

for i in $HDF_FILES_DIR/*hdf*; do
	$SCRIPTS_DIR/split_hdf_file.py $i
	(
		scp $i.* merlin:$MATYLDA_OUT_DIR
		rm $i.*
	) & 
done

wait
