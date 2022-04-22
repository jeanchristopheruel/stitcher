# Sticher: Stitching Library
High performance stitching library for 360 cameras based on OpenCV primitives

![alt tag](assets/stitching_example.png)

**Beta Milestone:**
- [x] Extrinsic Calibration App
- [x] Image stitching (As batch of images)
- [ ] Python Binding
- [x] Unit tests

## Index
- [Sticher: Stitching Library](#sticher-stitching-library)
    - [Index](#index)
    - [General Usage](#general-usage)
    - [Develop Environment](#develop-environment)
    - [Launch unit tests](#launch-unit-tests)

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

## How to use
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

### 3) Configure IDE to use remote environment container
#### For CLion
From the docker provided container images:
* user: `root`
* password: `password`
* address: `localhost`
* port: `2222`

**Note for CLION:**  
See https://www.jetbrains.com/help/clion/remote-projects-support.html
The .git directory is used in CMAKE to download the submodules automatically. But by default, CLION doesn't
include .git directory in remote interpreter (Probably to speed up the files transfer). To change it:
Preferences -> Build, Execution, Deployment -> Deployment -> Options ->
find “Exclude items by name”` and remove `.git` from the list.
Finally, reload all remote environment.

Workaround:
Manually log in the running docker container to update submodules with `git submodule update --init --recursive`

#### For VSCODE
Develop inside the created container "remote-cpp-env".
1. In host's VSCode, install extension [Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).
2. In host's VSCode, attach to the "remote-cpp-env" container and open directory `/home/tracktorpy`. If the volume binding is ok, the container should now share the tracktorpy directory with the host.
3. Inside the container's VSCode, install C++, Cmake and CMake-tools extensions.

You can use this setting for your vscode in `.vscode/settings.json` in order to enable the build of the c++ tests. For more info, visit [CMake-tools](https://vector-of-bool.github.io/docs/vscode-cmake-tools/settings.html).
```
{
    "cmake.configureArgs": [
        "-DPACKAGE_TESTS=ON"
    ]
}
```

## Launch unit tests
### C++ tests
No tests available yet.
```
mkdir build && cd build
cmake -DPACKAGE_TESTS=ON ..
ctest
```

## Files Structure Support
``` 
 *      intrinsic_json Structure:
 *      {
 *          "cameras": {
 *              cam1: {
 *                  "intrinsic": <list of float, len<9>>,
 *                  "dist_coeffs": <list of load, len<5>>,
 *                  "dims": [<width>, <height>]
 *              },
 *              ...
 *           }
 *      }
 *
 *      extrinsic_json Structure:
 *      {
 *          "cameras": {
 *              "cam1": {
 *                  "intrinsic": <list of float, len<9>>,
 *                  "dist_coeffs": <list of load, len<5>>,
 *                  "dims": [<width>, <height>],
 *                  "extrinsic": <list, len<9>>
 *              },
 *              ...
 *          }
 *      }
 *
 *      rotation_json Structure:
 *      {
 *          "cameras": {
 *              "cam1": {
 *                  "intrinsic": <list of float, len<9>>,
 *                  "dist_coeffs": <list of load, len<5>>,
 *                  "dims": [<width>, <height>],
 *                  "extrinsic": <list, len<9>>
 *                  "focal": <double>,
 *              },
 *              ...
 *          }
 *      }
 * @param dataset_json
 *      dataset Structure:
 *      {
 *          "cam1": <list of img paths, len<X>>,
 *          "cam2": <list of img paths, len<X>>,
 *          ...
 *      }
 */
```
