# ORB-SLAM3 Library Setup

This package requires the ORB-SLAM3 library files which are not included due to size.
Follow these steps to set up the library:

## Option 1: Clone from Mechazo11's Repository (Recommended)

```bash
cd /path/to/your/workspace/src
git clone --recursive https://github.com/Mechazo11/ros2_orb_slam3.git temp_orb_slam3

# Copy the orb_slam3 folder to this package
cp -r temp_orb_slam3/orb_slam3 ros2_orb_slam3/

# Clean up
rm -rf temp_orb_slam3
```

## Option 2: Manual Setup

1. Create the required directories:
```bash
mkdir -p orb_slam3/Vocabulary
mkdir -p orb_slam3/Thirdparty/DBoW2/lib
mkdir -p orb_slam3/Thirdparty/g2o/lib
mkdir -p orb_slam3/Thirdparty/Sophus
mkdir -p orb_slam3/include
mkdir -p orb_slam3/src
```

2. Download ORB-SLAM3 from the official repository:
```bash
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git /tmp/ORB_SLAM3
```

3. Copy required files:
```bash
cp -r /tmp/ORB_SLAM3/include/* orb_slam3/include/
cp -r /tmp/ORB_SLAM3/src/* orb_slam3/src/
cp -r /tmp/ORB_SLAM3/Thirdparty/* orb_slam3/Thirdparty/
cp /tmp/ORB_SLAM3/Vocabulary/ORBvoc.txt orb_slam3/Vocabulary/
```

4. Build third-party libraries:
```bash
# Build DBoW2
cd orb_slam3/Thirdparty/DBoW2
mkdir build && cd build
cmake ..
make -j$(nproc)

# Build g2o
cd ../../g2o
mkdir build && cd build
cmake ..
make -j$(nproc)

# Build Sophus
cd ../../Sophus
mkdir build && cd build
cmake ..
make -j$(nproc)
```

5. Convert vocabulary to binary format (faster loading):
```bash
# This is done automatically when you first run ORB-SLAM3,
# or you can use a conversion script
```

## Vocabulary File

The vocabulary file `ORBvoc.txt.bin` (binary format) should be placed in:
```
orb_slam3/Vocabulary/ORBvoc.txt.bin
```

You can download it from the ORB-SLAM3 repository or convert the text version.

## Directory Structure

After setup, the structure should look like:
```
orb_slam3/
├── config/
│   └── Monocular/
│       └── Jackal_Sim.yaml
├── include/
│   ├── System.h
│   ├── Tracking.h
│   └── ... (other headers)
├── src/
│   ├── System.cc
│   ├── Tracking.cc
│   └── ... (other sources)
├── Thirdparty/
│   ├── DBoW2/
│   │   └── lib/libDBoW2.so
│   ├── g2o/
│   │   └── lib/libg2o.so
│   └── Sophus/
└── Vocabulary/
    └── ORBvoc.txt.bin
```
