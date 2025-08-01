-- Build and activate a docker with ros1_bridge --

- build docker (already ran):
cd ~/ros1_ros2_bridge_ws
sudo docker build -t ros1_noetic_bridge -f Dockerfile.noetic_bridge .

- Activate Ros1 doceker:
sudo docker run -it \
  --name ros1_bridge_dev \
  -v $HOME/ros1_ros2_bridge_ws:/ros1_bridge_ws \
  ros1_noetic_bridge



-- Build and activate ROS1 docker with gazebo and rviz --

- build:
sudo docker build -t ros1_noetic_gui -f Dockerfile.noetic_gui .

- activate:
xhost +local:root  # Allow Docker to access your X11 display

sudo docker run -it \
  --name ros1_node_dev \
  --network=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros1_noetic_gui
  
------------
------------

- start only:
xhost +local:root
sudo docker start -ai ros1_node_dev

- exec only (multiple terminals):
xhost +local:root
sudo docker exec -it ros1_node_dev bash

------------
------------

NOTE: DO NOT USE "sudo docker run -it --rm \" if you do not want to remove the existing image and start from scratch
- if so, you may want to run "sudo docker rm ros1_node_dev" first


-----

- Mount both data and compare_SceneReplica from host to docker

sudo docker rm ros1_node_dev

sudo docker build -t ros1_noetic_gui -f Dockerfile.noetic_gui .

sudo docker run -it \
  --name ros1_node_dev \
  --network=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/Datasets:/root/Datasets \
  -v ~/compare_SceneReplica:/root/compare_SceneReplica \
  ros1_noetic_gui



-----


- TO include grasp_data:
sudo docker run -it \
  --name ros1_node_dev \
  --network=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/ros1_ros2_bridge_ws/grasp_data:/data \
  ros1_noetic_gui


-------



- save docker to image:
sudo docker commit ros1_node_dev ros1_node_dev_statble_v1


- if accidentally deleted docker container and want to create from the saved image snapshot and gpu support:

xhost +local:root  # Allow GUI access from Docker

sudo docker run -it \
  --gpus all \
  --name ros1_node_dev \
  --network=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/Datasets:/root/Datasets \
  -v ~/compare_SceneReplica:/root/compare_SceneReplica \
  ros1_node_dev_stable


- check existing dockers:
sudo docker images

- remove unused images:
sudo docker rmi ros1_node_dev_statble_v1

--------

July 18, 2025 -  Set up Docker container with GPU and X11 support 

- Installed latest NVIDIA driver 575.57.08, Confirmed installation with nvidia-smi
sudo apt install nvidia-driver-575
- Verified cuda.h and nvcc
/usr/local/cuda/include/cuda.h
nvcc --version → release 12.9

- Verified GPU support inside Docker with:
sudo docker run --rm --gpus all --runtime=nvidia nvidia/cuda:12.2.0-base-ubuntu20.04 nvidia-smi

- Committed the current container to an image:
sudo docker commit ros1_node_dev_cuda ros1_node_dev_cuda_backup:2025-07-18
- Exported the image to a tar archive for backup:
sudo docker save -o ~/ros1_node_dev_cuda_backup_2025-07-18.tar ros1_node_dev_cuda_backup:2025-07-18


  conda activate contact_graspnet_env
   - THEN:
  ./run_ros_fetch_experiment.sh
- terminal 7: (~/compare_SceneReplica/src#)
python bench_6dof_segmentation_grasping.py --grasp_method contact_gnet --seg_method msmformer --obj_order random --scene_idx 10

- output: No errors; loading, segmentation, and contact_graspnet all work; obects being detected and segmented, rostopics of /seg_labels and /seg_image are publishing messages; yet bench_6dof_segmentation_grasping returns [INFO] No object segmented
