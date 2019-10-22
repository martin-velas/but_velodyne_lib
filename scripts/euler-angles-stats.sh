#! /bin/bash

function plot {
	local label=$1
	local idx=$2

	gnuplot <<< "set terminal postscript enhanced color  
             set output '$out_dir/$label.ps'
             set key inside bottom left
             set xlabel 'Time'
             set ylabel 'pitch [rad]'
             # set xrange [144:146]
             # set xtics 0.1
             set grid
             plot '$cls_eulerangles' using 1:$idx with lines title 'CLS',\
               '$out_dir/quat.euler_angles' using 1:$idx with lines title 'IMU-quat'"
	ps2pdf $out_dir/$label.ps $out_dir/$label.pdf
}

if [ $# -ne 5 ]; then
	echo "ERROR, expected arguments: <frame.borders> <cls.poses> <imu.json> <imu-velodyne.calibration> <output_dir>" >&2
	exit 0
fi

frame_borders=$1
cls_poses=$2
imu_json=$3
imu_velodyne_calibration=$4
out_dir=$5


$BUT_VELODYNE_LIB/bin/imu-orientations-extraction -l "quaternion_full" -c $imu_velodyne_calibration < $imu_json > $out_dir/quat.poses

cut -d" " -f2- $out_dir/quat.poses | $BUT_VELODYNE_LIB/bin/poses-to-dof | cut -d" " -f4-6 |
	paste -d" " <(cut -d" " -f1 $out_dir/quat.poses) - > $out_dir/quat.euler_angles

cls_eulerangles=$out_dir/$(basename $cls_poses ".txt").euler_angles
$BUT_VELODYNE_LIB/bin/poses-to-dof < $cls_poses | cut -d" " -f4-6 | paste -d" " <(head -n -1 $frame_borders) - > $cls_eulerangles

plot "pitch" 2
plot "heading" 3
plot "roll" 4
