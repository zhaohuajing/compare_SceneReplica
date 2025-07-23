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

- start only:
xhost +local:root
sudo docker start -ai ros1_node_dev

- exec only (multiple terminals):
xhost +local:root
sudo docker exec -it ros1_node_dev bash


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



- bringup simulator
  - (single terminal)
roscore & gazebo
  or 
  - terminal 1:
sudo docker exec -it ros1_node_dev bash
roscore
  - terminal 2
sudo docker exec -it ros1_node_dev bash
rviz


-------

- inside docker_gui, to run scene_replica:
root@nerve-desktop-6:~/compare_SceneReplica/src# python setup_robot.py & python setup_ycb_scene.py --index 25 --datadir ~/Datasets/benchmarking/final_scenes/metadata/ & rosrun rviz rviz -d ../config/scene_setup.rviz


python setup_ycb_scene.py --index 161 --datadir /root/Datasets/benchmarking/final_scenes/metadata/

------

- run model based grasp manipulation pipeline

python bench_model_based_grasping.py -s 10 --pose_method posecnn --obj_order random

python bench_model_based_grasping.py --pose_method gazebo --obj_order random --scene_idx 36
- return: robot moves to self collision pose and stuck there; asked whether to open gripper/continue process
- 25/07/16 after resolved scene import: able to run

- run model free options
python bench_6dof_segmentation_grasping.py --grasp_method graspnet --seg_method uois --obj_order random --scene_idx 36
- return: [INFO] [1752617678.367224, 613.172000]: No object segmented

-------

- spawn object:
root@nerve-desktop-6:~/Datasets/benchmarking/models# rosrun gazebo_ros spawn_model -file /root/Datasets/benchmarking/models/006_mustard_bottle/model.sdf -sdf -model model3 -x 1 -y 1 -z 1 -R 0 -P 0 -Y 0


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


---------

- To run model-base grasping with poseCNN, (likely) need 6 terminals open:

- terminal 1: (~/compare_SceneReplica/launch#)
roslaunch just_robot.launch
- terminal 2: (~/compare_SceneReplica/src#)
python setup_scene_sim.py
- terminal 3: 
roslaunch fetch_moveit_config demo.launch
- terminal 4: (~/compare_SceneReplica/src/PoseCNN-PyTorch-NV-Release#)
rosrun rviz rviz -d ./ros/posecnn_fetch.rviz
- terminal 5: (~/compare_SceneReplica/src/PoseCNN-PyTorch-NV-Release#)
./experiments/scripts/ros_ycb_object_test_fetch.sh 0
- terminal 6: (~/compare_SceneReplica/src#)
python bench_model_based_grasping.py -s 10 --pose_method posecnn --obj_order random

- In Rviz, add MotionPlanning after all 6 terminal's commands are functioning

- Need to edit shader files (i.e., ycb_render/shaders/vertex_shader.vert) to replace GLSL 4.60 with 1.40, i.e., #version 460 -> #version 140


------

- To run graspNet:

cd ~/compare_SceneReplica/src/contact_graspnet 
conda contact_graspnet_env
python contact_graspnet/inference.py --np_path=test_data/0.npy  --forward_passes=5  --z_range=[0.2,1.1]


