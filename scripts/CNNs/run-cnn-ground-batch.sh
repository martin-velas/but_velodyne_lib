#! /bin/bash

if [ $# -ne 1 ]; then
	echo "Expected parameter: <net-id>" >&2
	exit 1
fi

ID=$1

DEPLOY_PROTOTXT=~/workspace/ivelas-git/sw/cnn-generator/deploy-definitions/${ID}_deploy.prototxt
TRAIN_DIR=~/workspace/ivelas-git/sw/cnn-generator/NETS/${ID}/
OUT_DIR=/media/kitti/data_tracking_velodyne_cnn_out/${ID}

if [ ! -d $TRAIN_DIR ]; then
	echo "Net $ID was not properly trained - missing directory: '$TRAIN_DIR'" >&2
	exit 1
fi

if [ ! -f $DEPLOY_PROTOTXT ]; then
	echo "Unable to find deploy prototxt file '$DEPLOY_PROTOTXT'"
	exit 1
fi

TEST_DATA_LIST=/media/kitti/data_tracking_velodyne/annotated_test_files.list
ANN_DATA_LIST_SORTED=/media/kitti/data_tracking_velodyne_ground_ann/ann_test_files_sorted.list
MASK_DATA_LIST_SORTED=/media/kitti/data_tracking_velodyne_BP_masks/test_masks.filelist.sorted

BIN=~/workspace/but_velodyne_lib/bin/ground-det-cnn
SCRIPTS_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

pushd $TRAIN_DIR

rm -rf $OUT_DIR/*

for run in run*; do
	last_model=$(ls $run/net_snapshot_iter_*.caffemodel | sort --version-sort | tail -n1)
	cnn_outputs_list=$OUT_DIR/$run/cnn-outputs-test-files.list
	while read cloud; do
		suffix=$(echo $cloud | rev | cut -f1-4 -d'/' | rev).ann
		out=$OUT_DIR/$run/cnn-outputs/$suffix
		mkdir -p $(dirname $out)
		$BIN -p $DEPLOY_PROTOTXT -m $last_model -o $out $cloud
		echo $out >>$cnn_outputs_list
	done < $TEST_DATA_LIST

	sort $cnn_outputs_list > $cnn_outputs_list.sorted

	$SCRIPTS_DIR/eval-ground-seg.sh $cnn_outputs_list.sorted $ANN_DATA_LIST_SORTED $OUT_DIR/$run/evaluation-output.txt
	$SCRIPTS_DIR/eval-ground-seg.sh $cnn_outputs_list.sorted $ANN_DATA_LIST_SORTED $OUT_DIR/$run/evaluation-masked-output.txt $MASK_DATA_LIST_SORTED
done

popd
