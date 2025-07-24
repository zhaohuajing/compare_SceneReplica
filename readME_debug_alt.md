python setup_ycb_scene.py --index 161 --datadir /root/Datasets/benchmarking/final_scenes/metadata/




poseCNN:

object 0, class 035_power_drill, z 0.724933385848999, z new 1.111829400062561
object 1, class 011_banana, z 0.8830955028533936, z new 0.40575963258743286
object 2, class 005_tomato_soup_can, z 0.961146354675293, z new 0.8840349912643433
object 3, class 024_bowl, z 0.763563871383667, z new 1.1272146701812744
object 4, class 004_sugar_box, z 0.6299688220024109, z new 1.1976063251495361
object 5, class 036_wood_block, z 0.4996073246002197, z new 0.8253579139709473
sdf 0 points for object 0, class 14 035_power_drill, no refinement
sdf 0 points for object 1, class 9 011_banana, no refinement
sdf 701 points for object 2, class 3 005_tomato_soup_can
sdf 0 points for object 3, class 12 024_bowl, no refinement
sdf 0 points for object 4, class 2 004_sugar_box, no refinement
sdf 11700 points for object 5, class 15 036_wood_block
sdf with 12401 points
sdf refinement iterations 50, time 0.090665
pose refine time 0.177455
035_power_drill 0.9958774 14
011_banana 0.99562246 9
005_tomato_soup_can 0.9885634 3
024_bowl 0.95793414 12
004_sugar_box 0.8178627 2
036_wood_block 0.38784295 15
cudaGraphicsGLRegisterImage failed: 304
cudaGraphicsMapResources failed: 400
cudaGraphicsSubResourceGetMappedArray failed: 400
cudaMemcpy2DFromArray failed: 709
cudaGraphicsUnmapResources failed: 400
cudaGraphicsGLRegisterImage failed: 304
cudaGraphicsMapResources failed: 400
cudaGraphicsSubResourceGetMappedArray failed: 400
cudaMemcpy2DFromArray failed: 709
cudaGraphicsUnmapResources failed: 400
cudaGraphicsGLRegisterImage failed: 304
cudaGraphicsMapResources failed: 400
cudaGraphicsSubResourceGetMappedArray failed: 400
cudaMemcpy2DFromArray failed: 709
cudaGraphicsUnmapResources failed: 400
cudaGraphicsGLRegisterImage failed: 304
cudaGraphicsMapResources failed: 400
cudaGraphicsSubResourceGetMappedArray failed: 400
cudaMemcpy2DFromArray failed: 709
cudaGraphicsUnmapResources failed: 400
--- 0.7388684749603271 seconds ---
cudaGraphicsGLRegisterImage failed: 304
cudaGraphicsMapResources failed: 400
cudaGraphicsSubResourceGetMappedArray failed: 400
cudaMemcpy2DFromArray failed: 709
cudaGraphicsUnmapResources failed: 400
cudaGraphicsGLRegisterImage failed: 304
cudaGraphicsMapResources failed: 400
cudaGraphicsSubResourceGetMappedArray failed: 400
cudaMemcpy2DFromArray failed: 709


UnseenObjectClusting, run pip install requirement:
ERROR: jupyter-packaging 0.12.3 has requirement setuptools>=60.2.0, but you'll have setuptools 45.2.0 which is incompatible.
ERROR: notebook 6.5.7 has requirement jupyter-client<8,>=5.3.4, but you'll have jupyter-client 8.6.3 which is incompatible.
ERROR: open3d 0.13.0 has requirement pillow>=8.2.0, but you'll have pillow 7.0.0 which is incompatible.
ERROR: open3d 0.13.0 has requirement pyyaml>=5.4.1, but you'll have pyyaml 5.3.1 which is incompatible.
ERROR: imageio 2.35.1 has requirement pillow>=8.3.2, but you'll have pillow 7.0.0 which is incompatible.


=====

running model-free grasp with UnseenObjectClustering and contact_graspnet, seeing in the terminal running just_robot:
[ERROR] [1753393692.713512, 844.292000]: bad callback: <bound method Subscriber.callback of <message_filters.Subscriber object at 0x7e021d02f450>>
Traceback (most recent call last):
  File "/root/miniconda3/envs/contact_graspnet_env/lib/python3.7/site-packages/tensorflow/python/client/session.py", line 1365, in _do_call
    return fn(*args)
  File "/root/miniconda3/envs/contact_graspnet_env/lib/python3.7/site-packages/tensorflow/python/client/session.py", line 1350, in _run_fn
    target_list, run_metadata)
  File "/root/miniconda3/envs/contact_graspnet_env/lib/python3.7/site-packages/tensorflow/python/client/session.py", line 1443, in _call_tf_sessionrun
    run_metadata)
tensorflow.python.framework.errors_impl.ResourceExhaustedError: 2 root error(s) found.
  (0) Resource exhausted: OOM when allocating tensor with shape[4,2048,128,128] and type float on /job:localhost/replica:0/task:0/device:GPU:0 by allocator GPU_0_bfc
	 [[{{node layer1/conv2_2/bn/cond/output/_242-0-0-TransposeNCHWToNHWC-LayoutOptimizer}}]]
Hint: If you want to see a list of allocated tensors when OOM happens, add report_tensor_allocations_upon_oom to RunOptions for current allocation info.

  (1) Resource exhausted: OOM when allocating tensor with shape[4,2048,128,128] and type float on /job:localhost/replica:0/task:0/device:GPU:0 by allocator GPU_0_bfc
	 [[{{node layer1/conv2_2/bn/cond/output/_242-0-0-TransposeNCHWToNHWC-LayoutOptimizer}}]]
Hint: If you want to see a list of allocated tensors when OOM happens, add report_tensor_allocations_upon_oom to RunOptions for current allocation info.

	 [[Sigmoid/_553]]
Hint: If you want to see a list of allocated tensors when OOM happens, add report_tensor_allocations_upon_oom to RunOptions for current allocation info.

0 successful operations.
0 derived errors ignored.

During handling of the above exception, another exception occurred:

Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/opt/ros/noetic/lib/python3/dist-packages/message_filters/__init__.py", line 76, in callback
    self.signalMessage(msg)
  File "/opt/ros/noetic/lib/python3/dist-packages/message_filters/__init__.py", line 58, in signalMessage
    cb(*(msg + args))
  File "/opt/ros/noetic/lib/python3/dist-packages/message_filters/__init__.py", line 330, in add
    self.signalMessage(*msgs)
  File "/opt/ros/noetic/lib/python3/dist-packages/message_filters/__init__.py", line 58, in signalMessage
    cb(*(msg + args))
  File "./ros/test_model_ros.py", line 220, in callback_points
    self.run_network(viz=False)
  File "./ros/test_model_ros.py", line 288, in run_network
    forward_passes=self.forward_passes,
  File "/root/compare_SceneReplica/src/contact_graspnet/ros/../contact_graspnet/contact_grasp_estimator.py", line 254, in predict_scene_grasps
    pred_grasps_cam[k], scores[k], contact_pts[k], gripper_openings[k] = self.predict_grasps(sess, pc_region, convert_cam_coords=True, forward_passes=forward_passes)
  File "/root/compare_SceneReplica/src/contact_graspnet/ros/../contact_graspnet/contact_grasp_estimator.py", line 192, in predict_grasps
    pred_grasps_cam, pred_scores, pred_points, offset_pred = sess.run(self.inference_ops, feed_dict=feed_dict)
  File "/root/miniconda3/envs/contact_graspnet_env/lib/python3.7/site-packages/tensorflow/python/client/session.py", line 958, in run
    run_metadata_ptr)
  File "/root/miniconda3/envs/contact_graspnet_env/lib/python3.7/site-packages/tensorflow/python/client/session.py", line 1181, in _run
    feed_dict_tensor, options, run_metadata)
  File "/root/miniconda3/envs/contact_graspnet_env/lib/python3.7/site-packages/tensorflow/python/client/session.py", line 1359, in _do_run
    run_metadata)
  File "/root/miniconda3/envs/contact_graspnet_env/lib/python3.7/site-packages/tensorflow/python/client/session.py", line 1384, in _do_call
    raise type(e)(node_def, op, message)
tensorflow.python.framework.errors_impl.ResourceExhaustedError: 2 root error(s) found.
  (0) Resource exhausted: OOM when allocating tensor with shape[4,2048,128,128] and type float on /job:localhost/replica:0/task:0/device:GPU:0 by allocator GPU_0_bfc
	 [[{{node layer1/conv2_2/bn/cond/output/_242-0-0-TransposeNCHWToNHWC-LayoutOptimizer}}]]
Hint: If you want to see a list of allocated tensors when OOM happens, add report_tensor_allocations_upon_oom to RunOptions for current allocation info.

  (1) Resource exhausted: OOM when allocating tensor with shape[4,2048,128,128] and type float on /job:localhost/replica:0/task:0/device:GPU:0 by allocator GPU_0_bfc
	 [[{{node layer1/conv2_2/bn/cond/output/_242-0-0-TransposeNCHWToNHWC-LayoutOptimizer}}]]
Hint: If you want to see a list of allocated tensors when OOM happens, add report_tensor_allocations_upon_oom to RunOptions for current allocation info.

	 [[Sigmoid/_553]]
Hint: If you want to see a list of allocated tensors when OOM happens, add report_tensor_allocations_upon_oom to RunOptions for current allocation info.

0 successful operations.
0 derived errors ignored.

---
- warning in other terminals running launch files:
[WARN] [1753393898.212546246, 977.001000000]: TF_REPEATED_DATA ignoring data with redundant timestamp for frame base_link (parent odom) at time 976.995000 according to authority unknown_publisher
[WARN] [1753393899.040372955, 977.631000000]: TF_REPEATED_DATA ignoring data with redundant timestamp for frame base_link (parent odom) at time 977.630000 according to authority unknown_publisher
[WARN] [1753393899.040994350, 977.631000000]: TF_REPEATED_DATA ignoring data with redundant timestamp for frame base_link (parent odom) at time 977.630000 according to authority unknown_publisher
[WARN] [1753393930.563567104, 998.392000000]: TF_REPEATED_DATA ignoring data with redundant timestamp for frame base_link (parent odom) at time 998.385000 according to authority unknown_publisher

---
output in motion planning python script:

7 objects segmented
save data to /root/compare_SceneReplica/src/data/experiments/bench_6dof_seg_cg/25-07-24_T214727_grasp-contact_gnet_seg-uois_scene-10_ord-random/listener_0724T214747/meta-000000.mat
Num: Labels: 8 | GT: 6
COST SHAPE: (6, 8)
Target maskid_gt: 8 | LABELS_GT: [  1   2   8  10  12 101]
Pred Label ids: [0 1 2 3 4 5 6 7]
Assignmnet: [(0, 7), (1, 4), (2, 1), (3, 5), (4, 6), (5, 0)]
*******object order: ['011_banana', '035_power_drill', '005_tomato_soup_can', '024_bowl', '004_sugar_box'] ************
Step: 0 | grasp object: 011_banana | maskid_label_GT: 8 | maskid_label_Pred: 1
(2319, 3)
(2319, 3)
Obtained target object points, publishing....
PC_ALL_SCENE shape: (269194, 3)
Publishing the PointCloud message!
Publishing the PointCloud message!
Waiting for grasp pose array message...


