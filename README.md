# CSCI585vSLAM
## Upstream Source Modifications

The `orbslam_patches/` directory contains a modified copy of
`common.cpp` from the upstream `ros2_orb_slam3` package.

These changes were made during debugging to ensure compatibility
with the camera pipeline and are included here to make the setup
reproducible on a different machine.

If rebuilding ORB-SLAM3 from source, first try the unmodified
upstream version before reapplying this patch.
