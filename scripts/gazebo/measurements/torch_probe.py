#!/usr/bin/env python3
# Does torch actually work on this GPU? An arch-list mismatch can satisfy
# torch.cuda.is_available() and still fail on the first kernel, so this runs a
# real convolution rather than trusting the flag. Run in the Jazzy dev image:
#   docker exec igvc_dev_jazzy python3 src/IGVC_robot_2026/scripts/gazebo/measurements/torch_probe.py
#
# Exit 0 only if CUDA is available, this GPU's own architecture is in torch's
# compiled arch list, and the convolution ran on the GPU. Exit 1 otherwise,
# 2 if torch does not import. Until 2026-09-24 it printed its verdict and
# always exited 0, so nothing could use it as a check.
import sys, subprocess
print("python:", sys.version.split()[0])
try:
    import torch
except Exception as e:
    print("TORCH_IMPORT_FAIL:", e); sys.exit(2)
print("torch:", torch.__version__)
print("torch.version.cuda:", torch.version.cuda)
print("cudnn:", torch.backends.cudnn.version())
print("is_available:", torch.cuda.is_available())
print("device_count:", torch.cuda.device_count())
failures = []
arch_list = []
try:
    arch_list = torch.cuda.get_arch_list()
    print("arch_list:", arch_list)
except Exception as e:
    print("arch_list FAIL:", e)
if torch.cuda.is_available():
    try:
        print("device_name:", torch.cuda.get_device_name(0))
        cap = torch.cuda.get_device_capability(0)
        print("capability:", cap)
        want = "sm_%d%d" % cap
        if want not in arch_list:
            failures.append("this GPU is %s and torch has no kernels for it" % want)
    except Exception as e:
        print("device_query FAIL:", e)
        failures.append("device query failed")
    # the real test: an actual convolution kernel on the GPU
    try:
        import torch.nn as nn
        x = torch.randn(1, 3, 224, 224, device="cuda")
        conv = nn.Conv2d(3, 16, 3, padding=1).cuda()
        y = conv(x)
        torch.cuda.synchronize()
        print("CONV_FORWARD_OK:", tuple(y.shape), "sum=%.4f" % float(y.sum()))
    except Exception as e:
        print("CONV_FORWARD_FAIL:", type(e).__name__, e)
        failures.append("the convolution did not run on the GPU")
else:
    print("CONV_FORWARD_SKIPPED: cuda not available")
    failures.append("CUDA is not available in this process")
try:
    import ultralytics; print("ultralytics:", ultralytics.__version__)
except Exception as e:
    print("ultralytics FAIL:", e)
try:
    import cv2; print("cv2:", cv2.__version__, cv2.__file__)
except Exception as e:
    print("cv2 FAIL:", e)
try:
    import numpy; print("numpy:", numpy.__version__)
except Exception as e:
    print("numpy FAIL:", e)
try:
    import onnxruntime as ort; print("onnxruntime:", ort.__version__, ort.get_available_providers())
except Exception as e:
    print("onnxruntime ABSENT:", type(e).__name__)
if failures:
    print("TORCH PROBE: FAIL -", "; ".join(failures))
    sys.exit(1)
print("TORCH PROBE: PASS")
sys.exit(0)
