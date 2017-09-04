#! /bin/bash

SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

DEPLOY_PROTOTXT=${1:-/home/ivelas/workspace/ivelas-git/sw/cnn-generator/deploy-definitions/odometry_l335_convout_b4_j2_h5_skip_and_succ_Rt_deploy.prototxt}
TRAINED_MODEL=${2:-/home/ivelas/workspace/ivelas-git/sw/cnn-generator/NETS/odometry_l335_convout_b4_j2_h5_skip_and_succ_transl/net_snapshot_iter_500000.caffemodel}
FORWARD_SCRIPT=${3:-$SCRIPT_DIR/odometry_forward_h5_transl.py}

OUTPUT_DIR_BASE=/media/files/cnn_velodyne_data/results/forpaper
DATA_DIR=/media/files/cnn_velodyne_data/2dMat_seq_360d_original
POSES_DIR=/media/files/cnn_velodyne_data/poses

label=$(basename $(dirname $TRAINED_MODEL))
it=$(egrep '[[:digit:]]+.caffemodel' -o <<<$TRAINED_MODEL | cut -d"." -f1)
OUTPUT_DIR=$OUTPUT_DIR_BASE/$(($(ls $OUTPUT_DIR_BASE | egrep '^[[:digit:]]+' -o | sort -g | tail -n1)+1))-$label-it$it

mkdir -p $OUTPUT_DIR
for i in $(ls $DATA_DIR)
do 
	echo $i; 
	$FORWARD_SCRIPT -p $DEPLOY_PROTOTXT -m $TRAINED_MODEL $(ls $DATA_DIR/$i/velodyne/*.yaml.gz | sort) -i $POSES_DIR/$i.txt -g $OUTPUT_DIR/$i.graph | tee $OUTPUT_DIR/$i.txt
done

cp $DEPLOY_PROTOTXT $TRAINED_MODEL $OUTPUT_DIR
cp $FORWARD_SCRIPT $OUTPUT_DIR
