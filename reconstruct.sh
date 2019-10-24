#!/bin/bash

img_folder=$1
working_folder=$1/recon/
bin_folder=$PWD/build/bin
mkdir -p $working_folder
cd $img_folder
DATE=`date +%Y-%m-%d:%H:%M:%S`
echo $DATE > $working_folder/reconstruction.log
echo Step 1: build_reconstruction
$bin_folder/build_reconstruction --flagfile=$img_folder/flags.txt >> $working_folder/reconstruction.log
echo Step 2: colorize_reconstruction
$bin_folder/colorize_reconstruction --image_directory=$img_folder/ --input_reconstruction_file=$working_folder/result-0 --output_reconstruction_file=$working_folder/result-0_with_color >> $working_folder/reconstruction.log
echo Step 3: export_cams
$bin_folder/export_cams --reconstruction=$working_folder/result-0_with_color --output_cams=$working_folder/pkfile.cams --output_ply=$working_folder/pkfile.ply>> $working_folder/reconstruction.log
echo Step 4: export_to_nvm_file
$bin_folder/export_to_nvm_file --input_reconstruction_file=$working_folder/result-0_with_color --output_nvm_file=$working_folder/result_with_color.nvm >> $working_folder/reconstruction.log
# echo Step 5: InterfaceVisualSFM
# /usr/local/bin/OpenMVS/InterfaceVisualSFM -i $working_folder/result_with_color.nvm -o $working_folder/result_with_color.mvs --output-image-folder $working_folder/ -w $img_folder/ >> $working_folder/reconstruction.log
# echo Step 6: DensifyPointCloud
# /usr/local/bin/OpenMVS/DensifyPointCloud -i $working_folder/result_with_color.mvs -o $working_folder/result_mvs_with_color.ply -w $img_folder/ >> $working_folder/reconstruction.log
echo Job finished
