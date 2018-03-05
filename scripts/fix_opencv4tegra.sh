sudo sed -i.bak s+arm-linux-gnueabihf/++g /opt/ros/indigo/lib/pkgconfig/cv_bridge.pc
sudo sed -i.bak s+arm-linux-gnueabihf/++g /opt/ros/indigo/lib/pkgconfig/image_geometry.pc
sudo sed -i.bak s+arm-linux-gnueabihf/++g  /opt/ros/indigo/share/cv_bridge/cmake/cv_bridgeConfig.cmake
sudo sed -i.bak s+arm-linux-gnueabihf/++g  /opt/ros/indigo/share/image_geometry/cmake/image_geometryConfig.cmake

sudo sed -i.bak s/2.4.8/2.4.12/g /opt/ros/indigo/lib/pkgconfig/cv_bridge.pc
sudo sed -i.bak s/2.4.8/2.4.12/g /opt/ros/indigo/lib/pkgconfig/image_geometry.pc
sudo sed -i.bak s/2.4.8/2.4.12/g  /opt/ros/indigo/share/cv_bridge/cmake/cv_bridgeConfig.cmake
sudo sed -i.bak s/2.4.8/2.4.12/g  /opt/ros/indigo/share/image_geometry/cmake/image_geometryConfig.cmake

sudo sed -i.bak s+-l:/usr/lib/libopencv_ocl.so.2.4.12++g /opt/ros/indigo/lib/pkgconfig/cv_bridge.pc
sudo sed -i.bak s+-l:/usr/lib/libopencv_ocl.so.2.4.12++g /opt/ros/indigo/lib/pkgconfig/image_geometry.pc
sudo sed -i.bak s+-l:/usr/lib/libopencv_ocl.so.2.4.12++g  /opt/ros/indigo/share/cv_bridge/cmake/cv_bridgeConfig.cmake
sudo sed -i s+/usr/lib/libopencv_ocl.so.2.4.12;++g  /opt/ros/indigo/share/cv_bridge/cmake/cv_bridgeConfig.cmake
sudo sed -i.bak s+-l:/usr/lib/libopencv_ocl.so.2.4.12++g  /opt/ros/indigo/share/image_geometry/cmake/image_geometryConfig.cmake
sudo sed -i s+/usr/lib/libopencv_ocl.so.2.4.12;++g  /opt/ros/indigo/share/image_geometry/cmake/image_geometryConfig.cmake
