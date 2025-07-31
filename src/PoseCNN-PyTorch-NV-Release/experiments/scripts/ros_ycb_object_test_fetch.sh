#!/bin/bash
	
set -x
set -e

export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True # 25/07/31: added to resolve cuda out of memory issue

export PYTHONUNBUFFERED="True"
export CUDA_VISIBLE_DEVICES=$1

time ./ros/test_images.py --gpu $1 \
  --instance 0 \
  --network posecnn \
  --pretrained data/checkpoints/ycb_object/vgg16_ycb_object_self_supervision_epoch_8.checkpoint.pth \
  --dataset ycb_object_test \
  --cfg experiments/cfgs/ycb_object_fetch.yml
