#!/bin/bash
#Run this in my_moveit directory.

cd RAFT-Stereo

python demo.py --restore_ckpt models/raftstereo-middlebury.pth \
--mixed_precision -l=../output/*/l*.png \
-r=../output/*/r*.png --save_numpy 

cd ..

python gen_pcd.py --disp=output/*/*.npy --image=output/*/l*.png 
