
在IMU采集数据时，会产生两种误差：确定性误差和随机性误差，为获得精确的数据，需要对上述两种误差进行标定,加速度计和陀螺仪随机误差的标定通常使用Allan方差法，它是20世纪60年代由美国国家标准局的David Allan提出的基于时域的分析方法。

依赖项版本问题的解决方法: `sudo aptitude install libdw-dev`，对给出的方案，选择第二个，降级 libelf1[0.165-3ubuntu1.1 (now) -> 0.158-0ubuntu]


## 发布D435i的IMU数据

可以直接在rs_camera.launch基础上针对IMU校准做修改。目的是将acc、gyro数据对齐使用同一个topic发布。

```xml
<!-- 更改前原版本arg name="unite_imu_method"          default=""/-->  
<arg name="unite_imu_method"          default="linear_interpolation"/>
<!--或着将参数改为copy-->
<arg name="unite_imu_method"          default="copy"/>
```

启动: `roslaunch realsense2_camera rs_imu_calibration.launch`，然后录制imu数据包`rosbag record -O imu_calibration /camera/imu`


## 运行标定程序

根据`imu_utils`文件夹里面的`A3.launch`改写D435i标定启动文件：`d435i_imu_calib.launch`注意，记得修改max_time_min对应的参数，默认是120，意味着两个小时，如果ros包里的imu数据长度没有两个小时，等bag播放完了，还是停留在`wait for imu data`这里，不会生成标定文件。我录了1小时59分多一点，所以还得改成119

`d435i_imu_calibration.launch`如下:
```xml
<launch>
    <node pkg="imu_utils" type="imu_an" name="imu_an" output="screen">
        <!--TOPIC名称和上面一致-->
        <param name="imu_topic" type="string" value= "/camera/imu"/>
        <!--imu_name 无所谓-->
        <param name="imu_name" type="string" value= "D435i"/>
        <!--标定结果存放路径-->
        <param name="data_save_path" type="string" value= "$(find imu_utils)/data/"/>
        <!--数据录制时间-min-->
        <param name="max_time_min" type="int" value= "120"/>
        <!--采样频率，即是IMU频率，采样频率可以使用rostopic hz /camera/imu查看，设置为200，也就是rosbag play播放频率-->
        <param name="max_cluster" type="int" value= "200"/>
    </node>
</launch>
```
先启动标定程序: `roslaunch imu_utils d435i_imu_calib.launch`，再播放bag: `rosbag play -r 200 imu_calibration.bag`


标定结果是`D435i_imu_param.yaml`:
```sh
%YAML:1.0
---
type: IMU
name: D435i
Gyr:
   unit: " rad/s"
   avg-axis:
      gyr_n: 3.6673681012835031e-03
      gyr_w: 7.0017785520472972e-05
   x-axis:
      gyr_n: 3.6001489799186333e-03
      gyr_w: 6.2846247607788020e-05
   y-axis:
      gyr_n: 4.7157261366663813e-03
      gyr_w: 7.5207268006344615e-05
   z-axis:
      gyr_n: 2.6862291872654953e-03
      gyr_w: 7.1999840947286307e-05
Acc:
   unit: " m/s^2"
   avg-axis:
      acc_n: 2.7436489578044256e-02
      acc_w: 1.0915021608117670e-03
   x-axis:
      acc_n: 1.8271976632141730e-02
      acc_w: 5.5394830052109354e-04
   y-axis:
      acc_n: 2.8924134998445018e-02
      acc_w: 1.5674764920646303e-03
   z-axis:
      acc_n: 3.5113357103546017e-02
      acc_w: 1.1530816898495772e-03
```
我们一会只用到**Gyr**中的`avg-axis`的`gyr_n`和`gyr_w`, **Acc**中的`avg-axis`的`acc_n`和`acc_w`
<br>

终端输出结果:
```sh
gyr x  numData 781205
gyr x  start_t 1.59343e+09
gyr x  end_t 1.59344e+09
gyr x dt 
-------------7140.59 s
-------------119.01 min
-------------1.9835 h
gyr x  freq 109.403
gyr x  period 0.00914049
gyr y  numData 781205
gyr y  start_t 1.59343e+09
gyr y  end_t 1.59344e+09
gyr y dt 
-------------7140.59 s
-------------119.01 min
-------------1.9835 h
gyr y  freq 109.403
gyr y  period 0.00914049
gyr z  numData 781205
gyr z  start_t 1.59343e+09
gyr z  end_t 1.59344e+09
gyr z dt 
-------------7140.59 s
-------------119.01 min
-------------1.9835 h
gyr z  freq 109.403
gyr z  period 0.00914049
Gyro X 
C   -6.83161    94.2973   -19.0588      2.983 -0.0404918
 Bias Instability 2.37767e-05 rad/s
 Bias Instability 6.28462e-05 rad/s, at 63.1334 s
 White Noise 12.9453 rad/s
 White Noise 0.00360015 rad/s
  bias -0.363298 degree/s
-------------------
Gyro y 
C   -8.74367    117.584   -15.9277    2.47408 -0.0373467
 Bias Instability 6.41864e-05 rad/s
 Bias Instability 7.52073e-05 rad/s, at 104.256 s
 White Noise 16.8998 rad/s
 White Noise 0.00471573 rad/s
  bias -0.544767 degree/s
-------------------
Gyro z 
C   -4.51808    68.1919   -9.33284    1.95333 -0.0262641
 Bias Instability 8.50869e-05 rad/s
 Bias Instability 7.19998e-05 rad/s, at 63.1334 s
 White Noise 9.43212 rad/s
 White Noise 0.00268623 rad/s
  bias -0.0762471 degree/s
-------------------
==============================================
==============================================
acc x  numData 781205
acc x  start_t 1.59343e+09
acc x  end_t 1.59344e+09
acc x dt 
-------------7140.59 s
-------------119.01 min
-------------1.9835 h
acc x  freq 109.403
acc x  period 0.00914049
acc y  numData 781205
acc y  start_t 1.59343e+09
acc y  end_t 1.59344e+09
acc y dt 
-------------7140.59 s
-------------119.01 min
-------------1.9835 h
acc y  freq 109.403
acc y  period 0.00914049
acc z  numData 781205
acc z  start_t 1.59343e+09
acc z  end_t 1.59344e+09
acc z dt 
-------------7140.59 s
-------------119.01 min
-------------1.9835 h
acc z  freq 109.403
acc z  period 0.00914049
acc X 
C  3.36177e-05   0.00175435 -0.000159698  7.23303e-05 -7.16006e-07
 Bias Instability 0.000553948 m/s^2
 White Noise 0.018272 m/s^2
-------------------
acc y 
C  9.36955e-05   0.00234733   0.00012197  0.000243676 -2.66252e-06
 Bias Instability 0.00156748 m/s^2
 White Noise 0.0289241 m/s^2
-------------------
acc z 
C  5.07832e-05   0.00331104 -0.000381222  0.000199602 -2.43776e-06
 Bias Instability 0.00115308 m/s^2
 White Noise 0.0351134 m/s^2
```

经过这些标定会生成一个yaml文件和很多txt文件，主要是yaml文件，给出了加速度计和陀螺仪三轴的`noise_density`和`random_walk`，同时计算出了平均值，后面IMU+摄像头联合标定的时候需要这些均值。

将Acc和Gyr的第一组平均数据拷贝到kalibr对应的`imu.yaml`文件中
```sh
rostopic: /camera/imu
update_rate: 200.0  #Hz
 
accelerometer_noise_density: 2.89e-01 #continous
accelerometer_random_walk: 4.55e-04 
gyroscope_noise_density: 3.02e-03 #continous
gyroscope_random_walk: 2.29e-05
```
分别是加速度计和陀螺仪的高斯白噪声和随机游走的平均值，是IMU噪声模型中的两种噪声。

##　查看默认imu与相机参数

D435i相关的imu_info话题如下:
![imu_info列表](https://i.loli.net/2020/06/30/nQftEcampMWxThw.png)


![/camera/gyro/imu_info和/camera/accel/imu_info没有结果](https://i.loli.net/2020/06/30/vUu4FljdA5tJRwQ.png)

`rostopic echo -n1 /camera/color/camera_info`得到结果:
```sh
  frame_id: "camera_color_optical_frame"
height: 480
width: 640
distortion_model: "plumb_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]
K: [611.3538208007812, 0.0, 327.437744140625, 0.0, 610.015869140625, 239.99667358398438, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [611.3538208007812, 0.0, 327.437744140625, 0.0, 0.0, 610.015869140625, 239.99667358398438, 0.0, 0.0, 0.0, 1.0, 0.0]
binning_x: 0
binning_y: 0
roi: 
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: False
```


对于`/camera/depth/imu_info`，结果是:
```sh
frame_id: "camera_depth_optical_frame"
height: 480
width: 640
distortion_model: "plumb_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]
K: [380.23321533203125, 0.0, 316.4999084472656, 0.0, 380.23321533203125, 237.40985107421875, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [380.23321533203125, 0.0, 316.4999084472656, 0.0, 0.0, 380.23321533203125, 237.40985107421875, 0.0, 0.0, 0.0, 1.0, 0.0]
binning_x: 0
binning_y: 0
roi: 
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: False
```


话题`/camera/infra2/camera_info`和`/camera/infra2/camera_info`的结果完全一样:
```sh
frame_id: "camera_infra1_optical_frame"
height: 480
width: 640
distortion_model: "plumb_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]
K: [380.23321533203125, 0.0, 316.4999084472656, 0.0, 380.23321533203125, 237.40985107421875, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [380.23321533203125, 0.0, 316.4999084472656, 0.0, 0.0, 380.23321533203125, 237.40985107421875, 0.0, 0.0, 0.0, 1.0, 0.0]
binning_x: 0
binning_y: 0
roi: 
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: False
```
可以看出，实际上出厂没有标定IMU



## 绘制Allan曲线

`imu_utils/scripts`中是Matlab写的.m文件，按照`draw_allan.m`创建文件`draw_allan_acc.m`和`draw_allan_gyr.m`

由于Ubuntu下安装Matlab比较麻烦，因此将数据和.m文件都拷贝到Windows系统下绘制，注意文件路径和m文件中要对应



参考: [RealSense D435i Calibration](https://blog.csdn.net/fb_941219/article/details/104709049)