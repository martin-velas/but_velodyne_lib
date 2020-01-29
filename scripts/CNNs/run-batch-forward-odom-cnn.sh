#! /bin/bash

DEPLOY_PROTOTXT=${1:-~/workspace/ivelas-git/sw/cnn-generator/deploy-definitions/odometry_l335_h6_b16_splitRt_deploy.prototxt}
NET_LABEL=${2:-odometry_l335_h6_b16_rot_shuffled_eulerfix_II}
TRAINED_MODEL=/home/ivelas/workspace/ivelas-git/sw/cnn-generator/NETS/$NET_LABEL/net_snapshot_iter_500000.caffemodel
OUTPUT_DIR=/media/files/cnn_velodyne_data/results/30-$NET_LABEL-it500k

DATA_DIR=/media/robodev1/kitti/data_velodyne_360d_range_y_int/sequences
POSES_DIR=/media/files/cnn_velodyne_data/poses
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

mkdir -p $OUTPUT_DIR

for i in $(ls $DATA_DIR)
do
	echo $i
	pushd $DATA_DIR/$i/velodyne
		$SCRIPT_DIR/odometry_forward.py -p $DEPLOY_PROTOTXT -m $TRAINED_MODEL $(ls *.npy | sort) -i $POSES_DIR/$i.txt -g $OUTPUT_DIR/$i.graph | tee $OUTPUT_DIR/$i.txt
	popd
done

cp $DEPLOY_PROTOTXT $TRAINED_MODEL $OUTPUT_DIR
cp $SCRIPT_DIR/odometry_forward.py $OUTPUT_DIR/schema.py
