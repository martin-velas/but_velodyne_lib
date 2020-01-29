#! /bin/bash

POSES_DIR=/media/files/cnn_velodyne_data/poses/
EMPTY_EVAL=/media/files/cnn_velodyne_data/results/empty-evaluation
RES_DIR=/media/files/cnn_velodyne_data/results

OUT_FOLDER=${1:-$RES_DIR/38-odometry_l335_rz_comp_hdfin_eulerfix_notrack-it200000-window}

TRANSL_DATA=${2:-$OUT_FOLDER}
ROT_DATA=${3:-$POSES_DIR}
ROT_X_DATA=${4:-$POSES_DIR}
ROT_Y_DATA=${5:-$POSES_DIR}
ROT_Z_DATA=${6:-$POSES_DIR}

EVAL_DIR=$OUT_FOLDER/evaluation-transl-only
cp -r $EMPTY_EVAL $EVAL_DIR

cp $0 $EVAL_DIR

for i in 00.txt  01.txt  02.txt  03.txt  04.txt  05.txt  06.txt  07.txt  08.txt  09.txt  10.txt
do 
	echo "Merging $OUT_FOLDER/$i"
	~/workspace/but_velodyne_lib/scripts/merge_pose_files.py -t $TRANSL_DATA/$i -r $ROT_DATA/$i -rx $ROT_X_DATA/$i -ry $ROT_Y_DATA/$i -rz $ROT_Z_DATA/$i > $EVAL_DIR/results/ivelas/data/$i
done

pushd $EVAL_DIR
	~/workspace/tmp-ivelas/sw/kitti-odometry-devkit/evaluate_odometry ivelas
popd

echo -n "Test stats: "
~/workspace/but_velodyne_lib/scripts/kitti_test_stats.py < $EVAL_DIR/results/ivelas/plot_error/average_te.txt | tee $EVAL_DIR/results/ivelas/stats-test.txt
