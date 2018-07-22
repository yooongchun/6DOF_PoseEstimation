Ubuntu16.04、Ubuntu17.10下编译安装PCL1.8

---

摘要：这篇文章给出在Ubunt16.04版本和Ubuntu17.10版本下编译安装PCL1.8的解决方案。安装的麻烦之处在于需要安装很多依赖库。

---

- STEP 1 :安装依赖库
  - 方法一 使用官方预编译版本安装
  ```
  sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl  
  sudo apt-get update  
  sudo apt-get install libpcl-dev
  ```
  
  - 方法二 自己编译（推荐）
    将以下内容保存为install_pcl_dependences.sh ，使用在ubuntu 命令行终端输入sudo sh install_pcl_dependences.sh 即可进行安装，在下载安装依赖库过程中会提示是否安装，都输入y 
        sudo apt-get update  
        sudo apt-get install git build-essential linux-libc-dev  
        sudo apt-get install cmake cmake-gui   
        sudo apt-get install libusb-1.0-0-dev libusb-dev libudev-dev  
        sudo apt-get install mpi-default-dev openmpi-bin openmpi-common    
        sudo apt-get install libflann1.8 libflann-dev  
        sudo apt-get install libeigen3-dev  
        sudo apt-get install libboost-all-dev  
        sudo apt-get install libvtk5.10-qt4 libvtk5.10 libvtk5-dev  
        sudo apt-get install libqhull* libgtest-dev  
        sudo apt-get install freeglut3-dev pkg-config  
        sudo apt-get install libxmu-dev libxi-dev   
        sudo apt-get install mono-complete  
        sudo apt-get install qt-sdk openjdk-8-jdk openjdk-8-jre 
  以上两个方法自己选一个即可，有的时候安装官方预编译版往往容易出现问题，因而可以采用自己编译的方式安装。
- STEP 2 ：从github 下载pcl1.8 
      git clone https://github.com/PointCloudLibrary/pcl.git 
- STEP 3 ：编译安装
      cd pcl  
      mkdir release  
      cd release  
      cmake -DCMAKE_BUILD_TYPE=None -DCMAKE_INSTALL_PREFIX=/usr \  
            -DBUILD_GPU=ON -DBUILD_apps=ON -DBUILD_examples=ON \  
            -DCMAKE_INSTALL_PREFIX=/usr ..  
      make 
      sudo make install
- STEP 4 :安装可视化库依赖
  - 安装OpenNI,OpenNI2 
      sudo apt-get install libopenni-dev   
      sudo apt-get install libopenni2-dev  
  - 安装ensensor
      sudo dpkg -i ensenso-sdk-2.0.147-x64.deb 
      sudo dpkg -i codemeter_6.40.2402.501_amd64.deb
      sudo apt-get -f install
- STEP 5 ：测试安装结果
  切换到pcl安装目录，找一个点云文件（如果没有，可以到这里下载）,输入以下代码查看
      pcl_viewer example_1.pcd
  正常情况下会显示点云文件
- 完成！


