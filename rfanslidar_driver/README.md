# rfanslidar_driver
SureStar社製LiDAR R-FansシリーズのROS2ドライバ  
使用するには下記のライブラリが必要です．

- ROS2(Humble or Foxy)
- Boost
- OpenCV
- MiYALAB.Cpp https://github.com/miyalab/MiYALAB.Cpp

使用するにはlib/RFansLiDARディレクトリ内のCMakeでinstall(sudo make install)してから，colcon等でbuildしてください．  

# component node
## MiYALAB::ROS2::RFansLiDAR
### publisher
- ~/points (sensor_msgs::msg::PointCloud)  
LiDARから得られる点群データ(channel[0]にrange，channel[1]に反射強度値が格納)

- ~/points_near(sensor_msgs::msg::PointCloud)  
LiDARから得られた点群データで，パラメータ"rfans.points.near_range"で設定された値より距離が小さい点群データ(channel[0]にrange，channel[1]に反射強度値が格納)

- \~/depth_img (sensor_msgs::msg::Image)  
LiDARから得られた点群データを横軸水平角度，縦軸垂直角度で画素(32bit浮動小数点型1チャンネル)に距離(0\~200.0)を格納した画像データ（データなしの画素には-1.0が格納）

- \~/intensity_img (sensor_msgs::msg::Image)  
LiDARから得られた点群データを横軸水平角度，縦軸垂直角度で画素(32bit浮動小数点型1チャンネル)に反射強度(0\~1.0)を格納した画像データ（データなしの画素には-1.0が格納）

### parameter
- rfans:
  - frame_id
  - device
    - model  
    LiDARの型番（R-Fans-16, R-Fans-16M, R-Fans-32, R-Fans-32Mのみ）
    - ip_address  
    LiDARのIPアドレス（初期設定 192.168.0.3）
    - status_port  
    LiDARのデバイス情報ポート（初期設定 2030）
  - scan
    - rate  
    スキャン速度(0, 5, 10, 20Hz)
    - theta
      - min  
      最小水平方向スキャン角度
      - max  
      最大水平方向スキャン角度
    - phi
      - min  
      最小垂直方向スキャン角度
      - max  
      最大垂直方向スキャン角度
  - points
    - publish  
    topic(~/points)の有効化
    - near_publish  
    topic(~/points_near)の有効化
    - near_range  
    topic(~/points_near)で送信される点群の最大距離
  - img:
    - depth_publish  
    topic(~/depth_img)の有効化
    - intensity_publish  
    topic(~/intensity_img)の有効化
    - theta_resolution  
    Depth img およびIntensity imgの横方向1 pixelあたりの角度分解能
    - phi_resolution  
    Depth img およびIntensity imgの縦方向1 pixelあたりの角度分解能
  - offset  
    - linear
      - x  
      LiDARの設置X座標
      - y  
      LiDARの設置Y座標
      - z  
      LiDARの設置Z座標
    - angular
      - x  
      LiDARの設置Roll角
      - y  
      LiDARの設置Pitch角
      - z  
      LiDARの設置Yaw角
