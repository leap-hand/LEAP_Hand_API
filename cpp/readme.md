## Welcome to the LEAP Hand C++ SDK

In order to use the LEAP Hand with C++ sdk please read the following information.

#### Install
- Build DynamixelSDK on $(YOUR_PLATFORM) {linux32, linux64, macos} (default: linux64) and leap_hand control

```bash
  cd ~/LEAP_Hand_API/cpp
  mkdir build && cd build
  cmake . # optional -DPLATEFORM={linux_sbc, linux32, linux64, mac}
  make 
  sudo make install
```

#### Usage 

To use leap_hand_client in your project please consider adding the following line to your cmake :

```cmake
find_package(leap_hand_client REQUIRED)

add_library(test SHARED src/test.cpp)
target_link_libraries(test PUBLIC leap_hand_client::dynamixel_client)
```


#### Test

Please execute the following commands to run the test.

```bash
cd ~/LEAP_Hand_API/cpp/build/src
./test_leap_hand
```



Thank you to Albert Paik (apaik458) at the University of Auckland for the first version!