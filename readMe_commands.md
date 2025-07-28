
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



- Print TF Tree in Command Line
rosrun tf view_frames


------

- For cuda out of memory:

1. kill duplicated process:
nvidia-smi
kill -9 <PID>

2. Enable expandable_segments (done)
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True

3. Clear GPU memory manually in script
import torch
torch.cuda.empty_cache()


---------
Run sceneReplica modules  
---------

* with different options of pose-estimation, perception-segmentation, and grasp loading/planning

--------

- To run model-based grasping with gazebo option:

  - terminal 1: (~/compare_SceneReplica/launch#)
roslaunch just_robot.launch
  - terminal 2: (~/compare_SceneReplica/src#)
python setup_scene_sim.py
  - terminal 3: 
roslaunch fetch_moveit_config demo.launch
  - terminal 4: (~/compare_SceneReplica/src#)
python bench_model_based_grasping.py -s 10 --pose_method gazebo --obj_order random


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
conda activate contact_graspnet_env
python contact_graspnet/inference.py --np_path=test_data/0.npy  --forward_passes=5  --z_range=[0.2,1.1]

------

- running model-free grasp with uois (unseen object clustering) and contact_graspnet:
  - terminal 1: (~/compare_SceneReplica/launch#)
roslaunch just_robot.launch
  - terminal 2: (~/compare_SceneReplica/src#)
python setup_scene_sim.py
  - terminal 3: 
roslaunch fetch_moveit_config demo.launch
  - terminal 4: (~/compare_SceneReplica/src/UnseenObjectClustering#)
./experiments/scripts/ros_seg_rgbd_add_test_segmentation_realsense.sh 0
  - terminal 5: (~/compare_SceneReplica/src/UnseenObjectClustering#)
rosrun rviz rviz -d ./ros/segmentation.rviz
  - terminal 6: ((contact_graspnet_env) root@nerve-desktop-6:~/compare_SceneReplica/src/contact_graspnet#) 
   - frist: 
  conda activate contact_graspnet_env
   - THEN:
  ./run_ros_fetch_experiment.sh
- terminal 7: (~/compare_SceneReplica/src#)
python bench_6dof_segmentation_grasping.py --grasp_method contact_gnet --seg_method uois --obj_order random --scene_idx 10

- Need to manually Add robot (MotionPlan) and scene (pointcloud2) to the rviz window brought by ./ros/segmentation.rviz

-------

run model-free grasp with MSMFormer (UnseenObjectsWithMeanShift) and contact_graspnet:
  - terminal 1: (~/compare_SceneReplica/launch#)
roslaunch just_robot.launch
  - terminal 2: (~/compare_SceneReplica/src#)
python setup_scene_sim.py
  - terminal 3: 
roslaunch fetch_moveit_config demo.launch
  - terminal 4: [not using conda env] (~/compare_SceneReplica/src/UnseenObjectsWithMeanShift#)
   - option 1:
   ./experiments/scripts/ros_seg_rgbd_add_test_segmentation_realsense.sh 0
   - option 2:
   ./experiments/scripts/ros_seg_rgbd_add_test_segmentation_fetch.sh 0
  - terminal 5: (~/compare_SceneReplica/src/UnseenObjectsWithMeanShift#)
rosrun rviz rviz -d ./ros/segmentation.rviz
  - terminal 6: ((contact_graspnet_env) root@nerve-desktop-6:~/compare_SceneReplica/src/contact_graspnet#) 
   - frist: 
   conda activate contact_graspnet_env
   - THEN:
  ./run_ros_fetch_experiment.sh
  - terminal 7: (~/compare_SceneReplica/src#)
python bench_6dof_segmentation_grasping.py --grasp_method contact_gnet --seg_method msmformer --obj_order random --scene_idx 10

- Need to manually Add robot (MotionPlan) and scene (pointcloud2) to the rviz window brought by ./ros/segmentation.rviz

