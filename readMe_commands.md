
- inside docker_gui, to run scene_replica:
root@nerve-desktop-6:~/compare_SceneReplica/src# python setup_robot.py & python setup_ycb_scene.py --index 25 --datadir ~/Datasets/benchmarking/final_scenes/metadata/ & rosrun rviz rviz -d ../config/scene_setup.rviz


python setup_ycb_scene.py --index 161 --datadir /root/Datasets/benchmarking/final_scenes/metadata/

------

- run model based grasp manipulation pipeline (25/07/14)

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
--------
  - terminal 1: (~/compare_SceneReplica/launch#)
roslaunch just_robot.launch
  - terminal 2: (~/compare_SceneReplica/src#)
python setup_scene_sim.py
  - terminal 3: 
roslaunch fetch_moveit_config demo.launch
  - terminal 4: (~/compare_SceneReplica/src#)
python bench_model_based_grasping.py -s 10 --pose_method gazebo --obj_order random

---------
- TO run model-base grasping with poseCNN, (likely) need 6 terminals open:
---------
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



---------
- TO run model-base grasping with poseRBPF, need 7 terminals open:
---------
  - terminal 1: (~/compare_SceneReplica/launch#)
roslaunch just_robot.launch
  - terminal 2: (~/compare_SceneReplica/src#)
python setup_scene_sim.py
  - terminal 3: 
roslaunch fetch_moveit_config demo.launch
  - terminal 4: (~/compare_SceneReplica/src/posecnn-pytorch#) [may be optional]
rviz -d ./ros/posecnn_fetch.rviz
  - terminal 5: (~/compare_SceneReplica/src/posecnn-pytorch#)
./experiments/scripts/ros_ycb_object_test_subset_poserbpf_realsense_ycb.sh 0 0
  - terminal 6: (~/compare_SceneReplica/src/posecnn-pytorch#) [pending shader modification]
/experiments/scripts/ros_poserbpf_ycb_object_test_subset_realsense_ycb.sh 0 0
  - terminal 7: (~/compare_SceneReplica/src#)
python bench_model_based_grasping.py -s 10 --pose_method poserbpf --obj_order random

- Also need to edit shader files (i.e., ycb_render/shaders/vertex_shader.vert) to replace GLSL 4.60 with 1.40, i.e., #version 460 -> #version 140

- similar to what have been done for poseCNN: modified ycb_object.py for Error Fix: IndexError from _extents_all[cfg.TRAIN.CLASSES] (IndexError: too many indices for array: array is 2-dimensional, but 21 were indexed) using:
self._extents = self._extents_all[cfg.TRAIN.CLASSES, :]
self._extents_test = self._extents_all[cfg.TEST.CLASSES, :] 

- Created new symlink ln -s ~/Datasets/poseCNN_data/models/YCB_Objects/models ~/compare_SceneReplica/src/posecnn-pytorch/data/models [note: not YCB_Video models]
and checkpoint: ln -s ~/Datasets/poseCNN_data/checkpoint/poseCNN/checkpoints/ycb_object ~/compare_SceneReplica/src/posecnn-pytorch/data/checkpoints/ycb_object

- Resolved ycb_render.py error with gpu device not found (Available devices: []
IndexError: list index out of range) from "self.r = CppYCBRenderer.CppYCBRenderer(width, height, get_available_devices()[gpu_id])" to equivalent of "self.r = CppYCBRenderer.CppYCBRenderer(width, height, 0)"

- installed pycuda; catkin_make to make posecnn_pytorch.msg callable; 

------
- To run graspNet:
------
cd ~/compare_SceneReplica/src/contact_graspnet 
conda activate contact_graspnet_env
python contact_graspnet/inference.py --np_path=test_data/0.npy  --forward_passes=5  --z_range=[0.2,1.1]

------
- To run model-free grasp with uois (unseen object clustering) and contact_graspnet:
------
  - terminal 1: (~/compare_SceneReplica/launch#) [optional: headless:=true gui:=false]
roslaunch just_robot.launch 
  - terminal 2: (~/compare_SceneReplica/src#)
python setup_scene_sim.py
  - terminal 3: 
roslaunch fetch_moveit_config demo.launch
  - terminal 4: (~/compare_SceneReplica/src/UnseenObjectClustering#)
./experiments/scripts/ros_seg_rgbd_add_test_segmentation_realsense.sh 0
  - terminal 5: (~/compare_SceneReplica/src/UnseenObjectClustering#) [optional! Can skip to save cuda memeory]
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
- To run model-free grasp with MSMFormer (UnseenObjectsWithMeanShift) and contact_graspnet:
--------
  - terminal 1: (~/compare_SceneReplica/launch#)
roslaunch just_robot.launch
  - terminal 2: (~/compare_SceneReplica/src#)
python setup_scene_sim.py
  - terminal 3: 
roslaunch fetch_moveit_config demo.launch
  - terminal 4: [may use msmformer_seg_env] (~/compare_SceneReplica/src/UnseenObjectsWithMeanShift#)
   - first:
conda activate msmformer_seg_env
   - then: [from sceneReplica readme]
./experiments/scripts/ros_seg_transformer_test_segmentation_fetch.sh 0
  - terminal 5: (~/compare_SceneReplica/src/UnseenObjectsWithMeanShift#) [optional]
rosrun rviz rviz -d ./ros/segmentation.rviz
  - terminal 6: ((contact_graspnet_env) root@nerve-desktop-6:~/compare_SceneReplica/src/contact_graspnet#) 
   - frist: 
conda activate contact_graspnet_env
   - THEN:
./run_ros_fetch_experiment.sh
  - terminal 7: (~/compare_SceneReplica/src#)
python bench_6dof_segmentation_grasping.py --grasp_method contact_gnet --seg_method msmformer --obj_order random --scene_idx 10

- Need to manually Add robot (MotionPlan) and scene (pointcloud2) to the rviz window brought by ./ros/segmentation.rviz


------
- To run model-free grasp with uois (unseen object clustering) and 6dof_graspnet:
------
  - terminal 1: (~/compare_SceneReplica/launch#) [optional: headless:=true gui:=false][added exports to set GUI with cpu; no need to worry]
roslaunch just_robot.launch 
  - terminal 2: (~/compare_SceneReplica/src#)
python setup_scene_sim.py
  - terminal 3: 
roslaunch fetch_moveit_config demo.launch
  - terminal 4: (~/compare_SceneReplica/src/UnseenObjectClustering#)
./experiments/scripts/ros_seg_rgbd_add_test_segmentation_realsense.sh 0
  - terminal 5: (~/compare_SceneReplica/src/UnseenObjectClustering#) [optional! Can skip to save cuda memeory]
rosrun rviz rviz -d ./ros/segmentation.rviz
  - terminal 6: (~/compare_SceneReplica/src/pytorch_6dof-graspnet#) 
./exp_publish_grasps.sh 
  - terminal 7: (~/compare_SceneReplica/src#)
python bench_6dof_segmentation_grasping.py --grasp_method graspnet --seg_method uois --obj_order random --scene_idx 10

- Need to manually Add robot (MotionPlan) and scene (pointcloud2) to the rviz window brought by ./ros/segmentation.rviz

