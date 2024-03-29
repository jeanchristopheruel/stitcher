# Sticher: Stitching Library
Stitching library for mutlisensor cameras based on OpenCV primitives

![alt tag](assets/stitching_example.png)

**Beta Milestone:**
- [x] Extrinsic Calibration App
- [x] Image stitching (As batch of images)
- [ ] Python Binding
- [ ] Lot more Unit tests

## General Usage
### Extrinsic Calibration App
```
[Extrinsic Calibration for Cylindrical Stitching] 
This tool evaluate the homography transformation matrix R for each provided images using 
precalibrated intrinsic and distortion coefficients. It works by computing features pairing between 
all the provided images. A bundle adjuster can be used to minimise the global reprojection error. 

FLAGS
  --help                [-h]  Display this message.

  --calibration_path    [-c]  MANDATORY
                              Specify the path to the intrinsic calibration json that contains 
                              intrinsic parameters, dist_coeffs and dimensions for each registered 
                              cameras.

  --dataset_path        [-d]  MANDATORY
                              Specify the path of the json file that contains the paths to the 
                              calibration images associated with each registered cameras.

  --output_path         [-o]  MANDATORY
                              Specify the path to the json output file that will contains the path to 
                              the results for the calibration of the extrinsic parameters.

  --features_thresh           OPTIONAL
                              DEFAULT: 0.65
                              Specify the threshold for features extraction using ORB.

  --adjustor_type             OPTIONAL
                              DEFAULT: ray
                              Specify the OpenCV Bundle Adjustor Type Used for features matching.
                              Options: 'no', 'ray', 'reproj' 
```

### Image stitching App
```
[Custom Stitching] 
This tool performs the stitching of multiple camera stream for a given projection model. 
The stream can be faked by providing a dataset of images for each camera.
It could also potentially be used on live camera with further optimizations.

FLAGS
  --help                [-h]  Display this message.

  --calibration_path    [-c]  MANDATORY
                              Specify the path to the calibration json that contains 
                              calibration parameters for each registered cameras.

  --dataset_path        [-d]  MANDATORY
                              Specify the path of the json file that contains the paths to the 
                              images associated with each registered cameras.

  --blend_strength      [-s]  OPTIONAL
                              DEFAULT: 5
                              Blending strength from [0,100] range. The default is 5. 

  --do_update_exposure        OPTIONAL
                              DEFAULT: 0
                              Allow to update exposure at each stitched frame. 

  --do_update_seams           OPTIONAL
                              DEFAULT: 0
                              Allow to update seams at each stitched frame. 

  --scale_factor              OPTIONAL
                              DEFAULT: 1
                              Camera scaling factor. Used to reduce memory consumption.
                              Must be withing this interval: ]0, 1]. 
```

## How to build dev environnement
### 1) Clone Repo and Submodules
```
git clone https://github.com/ruelj2/stitcher
cd stitcher
git submodule update --init --recursive
```
### 2) Build & Run Docker development container 
```
docker build -f docker/Dockerfile.dev-amd64 -t stitcher-dev .
docker run -d --cap-add sys_ptrace -p2222:22 --name stitcher-dev stitcher-dev
```

## Launch unit tests
### C++ tests
```
mkdir build && cd build
cmake -DPACKAGE_TESTS=ON ..
ctest
```

## Json Structure
### Intrinsic json Structure
``` 
 {
     "cameras": {
         cam1: {
             "intrinsic": <list of float, len<9>>,
             "dist_coeffs": <list of load, len<5>>,
             "dims": [<width>, <height>]
         },
         ...
      }
 }
 ``` 
 
 ### Extrinsic json Structure
 ``` 
 {
     "cameras": {
         "cam1": {
             "intrinsic": <list of float, len<9>>,
             "dist_coeffs": <list of load, len<5>>,
             "dims": [<width>, <height>],
             "extrinsic": <list, len<9>>
         },
         ...
     }
 }
 ``` 
 
### Rotation json Structure
``` 
 {
     "cameras": {
         "cam1": {
             "intrinsic": <list of float, len<9>>,
             "dist_coeffs": <list of load, len<5>>,
             "dims": [<width>, <height>],
             "extrinsic": <list, len<9>>
             "focal": <double>,
         },
         ...
     }
 }
``` 

### Dataset json Structure
```
 {
     "cam1": <list of img paths, len<X>>,
     "cam2": <list of img paths, len<X>>,
     ...
 }
```
