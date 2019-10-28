#! /bin/bash

if [ $# -lt 2 ]
then
	echo "Missing agruments: ID CLOUDS_DIR+" >&2
	exit 1
fi

kitti_id=$1
shift
cloud_dirs=$@

for d in $cloud_dirs
do
	joined_cloud=$(mktemp --suffix .$(basename $d).pcd)
	$BUT_VELODYNE_LIB/bin/build-3d-model -p $CALIBRATION_VELODYNES $d/$kitti_id.1.pcd $d/$kitti_id.2.pcd -s 1.0 \
		-o $joined_cloud
	joined_clouds="$joined_clouds $joined_cloud"
done

pcl_viewer -multiview 1 $joined_clouds

rm $joined_clouds
