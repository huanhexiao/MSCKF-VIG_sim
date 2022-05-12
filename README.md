# MSCKF-VIG



This repository focuses on the simulated Monocular Vision/INS/GNSS fusion, it is based on https://github.com/huanhexiao/MSCKF-VIG (on the way).
The difference of the experimental and simulated data is mainly on the vision front-end, the format of INS and GNSS is same to the real tests.
The simulated version is easy and friendly for you to quickly master the principle of multi-sensor integrated navigation.

The data is produced by a self-developed software, 
you can contact Dr. Ronghe Jin (huanhexiao@whu.edu.cn or 773792173@qq.com) for the simulation code if you have a need.

MSCKF-VIG (https://github.com/huanhexiao/MSCKF-VIG) will be available as soon as the code and data are ready.

## Functions

- GNSS/INS integration

- Monocular Vision/INS/GNSS fusion



## Prerequisites

- glog 

- Eigen

- OpenCV 3.4

- Ceres



## Usage

```shell

git clone https://gitlab.com/huanhexiao/MSCKF-VIG

mkdir build && cd build 

cmake .. && make -j3

```

## License
The source code is released under GPLv3 license.

Feel free to propose any issue you get confused, or you can to email me directly.