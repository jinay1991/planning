# Motion Planning 

[![pipeline status](https://gitlab.com/jinay1991/motion_planning/badges/master/pipeline.svg)](https://gitlab.com/jinay1991/motion_planning/commits/master)
[![coverage report](https://gitlab.com/jinay1991/motion_planning/badges/master/coverage.svg?job=code-coverage)](https://jinay1991.gitlab.io/motion_planning/index.html)
[[Issue Board](https://gitlab.com/jinay1991/motion_planning/-/boards)]

**WIP** Autonomous Driving Motion Planning (with conjunction with Udacity Simulator) 

## Build

* Build project
    * Release `bazel build //...`
    * Debug `bazel build -c dbg //...`
* Run Unit Tests `bazel test //... --test_output=all`

## Test

* Launch Simulator 
* Run `./bazel-bin/client-app data/highway-map.csv`

![Screenshot](example/screenshot_01.png)

## Dependencies

* [Bazel](https://docs.bazel.build/versions/1.1.0/getting-started.html) 
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
* Simulator
    * Download prebuilt binaries
        * [Linux](https://github.com/udacity/self-driving-car-sim/releases/download/T3_v1.2/term3_sim_linux.zip) 
        * [MacOS](https://github.com/udacity/self-driving-car-sim/releases/download/T3_v1.2/term3_sim_mac.zip)
        * [MacOS Catalina v10.15](https://github.com/jinay1991/motion_planning/releases/download/v1.1/term3_sim_mac_catalina.zip)
        * [Windows](https://github.com/udacity/self-driving-car-sim/releases/download/T3_v1.2/term3_sim_windows.zip)
