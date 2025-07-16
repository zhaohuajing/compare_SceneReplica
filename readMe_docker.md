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

python bench_model_based_grasping.py --pose_method gazebo --obj_order random --scene_idx 36
- return: robot moves to self collision pose and stuck there; asked whether to open gripper/continue process

python bench_6dof_segmentation_grasping.py --grasp_method graspnet --seg_method uois --obj_order random --scene_idx 36
- return: [INFO] [1752617678.367224, 613.172000]: No object segmented

-------

spawn object:
root@nerve-desktop-6:~/Datasets/benchmarking/models# rosrun gazebo_ros spawn_model -file /root/Datasets/benchmarking/models/006_mustard_bottle/model.sdf -sdf -model model3 -x 1 -y 1 -z 1 -R 0 -P 0 -Y 0

