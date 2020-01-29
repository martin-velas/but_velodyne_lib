#! /bin/bash

DATA_LABEL=j2_h1_b3_rad_1800d_rcls56_eulerfix

OUT_DIR=/media/files/cnn_velodyne_data/hdf_data/$DATA_LABEL
#MATYLDA_OUT_DIR=/mnt/matylda1/ivelas/cnn_velodyne_data/hdf_data/$DATA_LABEL
POSES=/media/files/cnn_velodyne_data/poses
VELODYNE_SEQ=/media/robodev1/kitti/data_velodyne_1800d_range_y_int/sequences

SCRIPTS_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

mkdir -p $OUT_DIR
rm -f $OUT_DIR/*
#ssh merlin "mkdir -p $MATYLDA_OUT_DIR; rm -f $MATYLDA_OUT_DIR/*"

cp $SCRIPTS_DIR/odometry_data_to_hdf5_by_schema.py $OUT_DIR/schema.txt
#scp $OUT_DIR/schema.txt merlin:$MATYLDA_OUT_DIR

for seq in $(ls $VELODYNE_SEQ); do
	echo "Processing sequence: $seq"
	pushd $VELODYNE_SEQ
		$SCRIPTS_DIR/odometry_data_to_hdf5_by_schema.py $POSES/$seq.txt $OUT_DIR/$seq $(find $seq/velodyne -name '*.npy' | sort --version-sort)
# 	scp $OUT_DIR/$seq* merlin:$MATYLDA_OUT_DIR &
	popd
done |& tee $OUT_DIR/output.log

wait
