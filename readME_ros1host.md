 Summary of Today's Progress and Debugging on ROS1 Local Host Computer (nerve-desktop-7)

Here’s a detailed summary of what was accomplished today regarding setup, debugging, and successful run of compare_SceneReplica with PoseCNN-PyTorch-NV-Release:
1. PoseCNN Setup and Data Linking

    Created symbolic links to:

        /home/csrobot/Datasets/poseCNN_data/checkpoint/poseCNN → data/checkpoints

        /home/csrobot/Datasets/poseCNN_data/models/YCB_Objects → data/models

    This ensured pretrained weights and 3D models could be accessed from the PoseCNN-PyTorch-NV-Release directory.

2. Error Fixes and Compilation

    Resolved Segmentation fault (core dumped) by identifying issues related to Sophus::SO3::expAndTheta().

        Attempted to recompile Sophus.

        Switched to alternative Sophus repository: https://github.com/yuxng/Sophus

        Commented out SOPHUS_ENSURE(...) checks temporarily to bypass the failure due to floating-point inaccuracies.

2.5. Error Fixes and Compilation (Expanded)

    Segmentation Fault (core dumped) traced to:

    Sophus::SO3::expAndTheta() in so3.hpp

        Specifically:
        
        SOPHUS_ENSURE(abs(q.unit_quaternion().squaredNorm() - Scalar(1)) < Sophus::Constants<Scalar>::epsilon(), ...);

        This check was too strict and caused failures due to minor floating-point numerical errors when computing the unit quaternion norm.

    Fixes Attempted:

        Attempted to recompile the existing Sophus installation:

            Found that it was installed under /usr/local/include/sophus (likely system-wide).

            Tried to rebuild and fix it locally, but not sufficient due to internal logic still enforcing strict checks.

        Switched to alternative (PoseCNN-compatible) Sophus repository:

    git clone https://github.com/yuxng/Sophus.git

        This version matches the custom use in PoseCNN.

        Replaced the old system-wide include path with this local copy to avoid conflicts.

    Temporarily disabled the SOPHUS_ENSURE(...) line in so3.hpp:

        Commented out the check:

                // SOPHUS_ENSURE(...);

                Allowed the computation to proceed even with minor unit norm errors.

                This resolved the Segmentation fault when running pose refinement.

3. Fix for Invalid deltaPose[0] = nan Values

    After resolving the segmentation fault, the script still failed during SDF-based pose refinement, with the repeated warning:

    Invalid deltaPose[0] = nan for obj_index = X

        Occurred due to unstable optimization steps during SDF pose refinement (sdf_optimizer).

    Fix Applied:

        Disabled SDF pose refinement by setting the following in the config:

            POSE_REFINE: False

            File: experiments/cfgs/ycb_object.yml

            Effect: Skips the sdf_optimizer step that attempts fine pose alignment using SDFs, which caused the NaN outputs.


4. Successful Demo with DEMO IMAGES (NO REFINE)

    Ran the following command successfully:

    ./experiments/scripts/demo.sh

        Outputted visualized PoseCNN predictions with multiple objects.

        All objects successfully detected and poses rendered.

5. Running ROS-based Test Script

    Ran:

./experiments/scripts/ros_ycb_object_test_fetch.sh 0

    Encountered missing module:

ModuleNotFoundError: No module named 'ros_numpy'

Resolved by cloning and manually installing:

        git clone https://github.com/eric-wieser/ros_numpy.git
        cd ros_numpy
        pip install .

6. Gazebo Launch: Headless and GUI-Free Setup

    Initial launch of just_robot.launch tried to open the full Gazebo GUI, which isn't needed for headless operation and causes GUI-related errors on remote or CLI-only systems.

    Fix Applied:

        Launched Gazebo in headless mode:

roslaunch YOUR_PACKAGE just_robot.launch gui:=false headless:=true

OR modified the .launch file:

    <arg name="gui" default="false"/>
    <arg name="headless" default="true"/>

This prevents Gazebo from opening its graphical client, enabling stable robot simulation in background or remote setups.


7. Real-time GPU Monitoring

    Used the command:

    watch -n 1 nvidia-smi

