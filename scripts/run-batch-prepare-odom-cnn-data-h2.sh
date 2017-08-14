#! /bin/bash

DATA_LABEL=joined2_hist2_batch4_skipped_and_succ_single_odom_new

OUT_DIR=~/kitti/dataset_odometry_velodyne_odom_cnn_data/$DATA_LABEL

SCRIPTS_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
POSES=~/cnn_velodyne_data/poses/
VELODYNE_SEQ=~/kitti/data_odometry_velodyne_fake_ground_data_original/sequences

mkdir -p $OUT_DIR
rm -f $OUT_DIR/*

cp $SCRIPTS_DIR/odometry_data_to_hdf5_by_schema_h2.py $OUT_DIR/schema.txt

for i in 00 01 02 03 04 05 06 07 08 09 10; do 
	$SCRIPTS_DIR/odometry_data_to_hdf5_by_schema_h2.py $POSES/$i.txt $OUT_DIR/$i $(ls $VELODYNE_SEQ/$i/velodyne/*.gz | sort)
done

wait
