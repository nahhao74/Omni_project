# PROJECT ROBOT OMNI
# ROS 2 Nav2: Autonomous Omni-directional Hospital Navigation Robot

**Tác giả:** Nguyễn Anh Hào & Trần Minh Cương  
**Email:** nahhao74@gmail.com , tmcuong0507@gmail.com

---

Dự án này triển khai hệ thống điều hướng tự hành toàn diện cho một robot đa hướng (omni-directional robot) hoạt động trong môi trường mô phỏng bệnh viện (AWS Hospital World). Hệ thống được xây dựng trên nền tảng ROS 2 Jazzy và thư viện Nav2 stack trong môi trường mô phỏng Gazebo.

Điểm nhấn của dự án nằm ở việc giải quyết bài toán động học phức tạp của xe omni trong không gian hẹp. Hệ thống sử dụng Smac A* làm Global Planner để tạo ra các đường đi an toàn, có tính toán đến footprint của robot. Ở cấp độ điều khiển, dự án kết hợp Rotation Shim Controller để xử lý các góc quay ưu tiên, và MPPI Controller để dự đoán và lấy mẫu quỹ đạo trong thời gian thực, giúp robot lách qua các vật cản động một cách mượt mà nhất.

---

## Phạm vi của dự án bao gồm:
1. Xây dựng mô hình robot và môi trường mô phỏng vật lý có độ tin cậy cao.
2. Xử lý tín hiệu đầu vào từ cảm biến (LiDAR, IMU, Camera) và dữ liệu Odometry.
3. Thiết lập hệ thống TF tree chuẩn xác.
4. Xây dựng bản đồ trong môi trường phức tạp (SLAM & Mapping). Sử dụng các thuật toán SLAM hiện đại trên ROS 2 để quét và xây dựng bản đồ Occupancy Grid 2D từ môi trường AWS Hospital World, xử lý nhiễu từ các hành lang dài và các phòng bệnh có cấu trúc giống nhau.
5. Cấu hình các sever phục vụ cho Nav2 để tối ưu hóa quỹ đạo di chuyển và khả năng tránh vật cản động/tĩnh.

---

## Setup

### 1. Yêu cầu hệ thống:
- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Simulator
- AWS Robotic Hospital World packages

### 2. Tạo Workspace và cài đặt thư viện:

```bash
# Tạo ROS 2 workspace
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src

# Clone repository của dự án
git clone https://github.com/nahhao74/Omni_project.git

# Build
cd ~/ros_ws
colcon build

# Khai báo đường dẫn
source install/setup.bash
```

### 3. Khởi chạy file launch

```bash
ros2 launch robot_omni localization_launch.py
```

---

## Tính năng & Demo

- Xây dựng bản đồ trong môi trường phức tạp (SLAM & Mapping): Sử dụng các thuật toán SLAM hiện đại trên ROS 2 để quét và xây dựng bản đồ Occupancy Grid 2D từ môi trường AWS Hospital World, xử lý nhiễu từ các hành lang dài và các phòng bệnh có cấu trúc giống nhau.

- Lập kế hoạch đường đi toàn cục với Smac A* (Global Planning): Smac A* Planner tính toán đường đi tối ưu nhất từ vị trí hiện tại đến phòng bệnh mục tiêu. Thuật toán cân nhắc chi phí về khoảng cách cũng như vùng rủi ro, đảm bảo quỹ đạo không bo góc quá gắt gây va chạm tường.

- Điều hướng mượt mà với MPPI & Rotation Shim Controller (Local Control): Rotation Shim giúp robot xoay hướng mặt về phía mục tiêu trước khi tiến. Ngay sau đó, MPPI Controller sẽ liên tục lấy mẫu hàng ngàn quỹ đạo để chọn ra hướng đi linh hoạt nhất, giúp đạt tới vị trí đích với đường đi tối ưu nhất và tránh va chạm.

---

## Acknowledgments

Dự án này được phát triển dựa trên nền tảng của nhiều thư viện mã nguồn mở và tài liệu tham khảo từ cộng đồng ROS. Xin gửi lời cảm ơn đặc biệt đến:

- Cartographer ROS / TurtleBot3: Các file cấu hình Launch và SLAM (như cartographer_node) được xây dựng dựa trên mã nguồn gốc của tác giả Darby Lim (Bản quyền 2019 thuộc Open Source Robotics Foundation, Inc. - Giấy phép Apache 2.0). Cấu trúc gốc đã được tùy biến lại (remapping) để có thể hợp nhất dữ liệu từ hệ thống Lidar kép và Odometry cho robot di chuyển đa hướng.

- Nav2 & MPPI: https://docs.nav2.org/configuration/packages/configuring-mppic.html
- AWS Hospital World: https://github.com/aws-robotics/aws-robomaker-hospital-world.git

---

## Xây dựng mô hình robot

### 1. Hình dạng kích thước
- Loại robot: Xe 4 bánh đa hướng (Omni/Mecanum).
- Trọng lượng và kích thước: Phần thân chính (base_link) có khối lượng hơn 34 kg, kích thước bao phủ khoảng 0.71m x 0.49m x 0.16m.
- Hệ thống bánh xe: 4 bánh (trước-phải, trước-trái, sau-phải, sau-trái) được cấp lệnh vận tốc độc lập thông qua plugin gz_ros2_control.

### 2. Hệ thống cảm biến (Sensors)

- Hệ thống quét LiDAR kép (SICK TiM551): Xe được trang bị hai cảm biến quét laser, một đặt ở phía trước (base_front_laser) và một ở phía sau (base_rear_laser). Mỗi LiDAR có góc quét 270 độ và tầm nhìn xa 25 mét. Bố trí này giúp tạo góc nhìn 360 độ, loại bỏ điểm mù.

- Camera chiều sâu (RealSense D435): Xe có 2 camera RGBD, một gắn trên thân và một gắn trên giá đỡ phía trên, phục vụ nhận dạng vật thể 3D.

- IMU: Cập nhật dữ liệu gia tốc và góc quay ở tần số 100Hz, giúp cải thiện độ chính xác odometry.

### 3. Tương thích hệ thống
- ROS 2 & Gazebo (Ignition/gz-sim)
- Plugin:
  - gz_ros2_control::GazeboSimSystem
  - gz-sim-imu-system
 
![Robot](image/robotfull.png)


# Xử lý tín hiệu đầu vào từ cảm biến và dữ liệu Odometry.

Hệ thống robot xử lý dữ liệu từ cảm biến theo pipeline:

```
Gazebo Sensors
      ↓
ROS-GZ Bridge
      ↓
ROS2 Topics (/scan, /imu, /odom)
      ↓
EKF (robot_localization)
      ↓
/odometry/filtered
      ↓
SLAM / Navigation (Nav2)
```

---

## 1. Sensor Inputs

### LiDAR

**Topics:**
- `/scan_front_raw`
- `/scan_rear_raw`

**Data:**
- Khoảng cách đến vật cản theo từng góc

**Usage:**
- Phát hiện vật cản  
- Xây dựng bản đồ (SLAM)  
- Tránh va chạm  

---

### IMU

**Topic:**
- `/base_imu`

**Data:**
- Góc quay (yaw)  
- Vận tốc góc  
- Gia tốc  

**Usage:**
- Xác định hướng robot  
- Giữ ổn định góc quay  
- Hỗ trợ EKF  

---

### RGB-D Camera

**Topics:**
- `/camera/depth_image`
- `/camera/points`

**Data:**
- Ảnh độ sâu  
- PointCloud 3D  

**Usage:**
- Nhận diện vật thể  
- Phát hiện vật cản 3D  
- Bổ sung cho LiDAR  

---

### Wheel Odometry

**Topic:**
- `/mobile_base_controller/odometry`

**Data:**
- Vận tốc tiến (Vx), ngang (Vy)  
- Vận tốc quay  

**Usage:**
- Ước lượng chuyển động robot  

---

## 2. Gazebo → ROS2 Bridge

Sử dụng:

```bash
ros2 run ros_gz_bridge parameter_bridge
```

### Launch config

```python
bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    parameters=[{'config_file': bridge_config}, {'use_sim_time': True}],
)
```

### Chức năng

- Chuyển đổi message:
  - LaserScan
  - Imu
  - Image
- Chuyển topic từ Gazebo sang ROS2

---

## 3. Time Synchronization

```yaml
use_sim_time: true
```

- Tất cả node sử dụng `/clock`
- Đảm bảo dữ liệu đồng bộ giữa các sensor

---

## 4. Sensor Fusion (EKF)

### Node

```python
ekf_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node'
)
```

### Input

- Odometry (vận tốc)  
- IMU (góc quay)  

### Process

- Lọc nhiễu  
- Kết hợp nhiều nguồn dữ liệu  
- Ước lượng trạng thái robot  

### Output

- `/odometry/filtered`

### Vai trò

Dữ liệu chính cho:
- SLAM  
- Navigation  
- RViz  

---

## 5. SLAM Integration

### Config (`slam.lua`)

```lua
use_odometry = true
use_imu_data = true
num_laser_scans = 2
```

### Input topics

```python
('/scan_1', '/scan_front_raw'),
('/scan_2', '/scan_rear_raw'),
('/imu', '/base_imu'),
('/odom', '/odometry/filtered')
```

### Chức năng

- Xây dựng bản đồ  
- Định vị robot  

---

## 6. Navigation (Nav2)

### Input

- `/odometry/filtered`
- `/scan`
- `/camera/points`
- Map

### Chức năng

- Xác định vị trí robot  
- Lập kế hoạch đường đi  
- Tránh vật cản  

---

## Summary

```
LiDAR + Camera → môi trường
IMU            → hướng
Odometry       → vận tốc
                     ↓
                   EKF
                     ↓
         /odometry/filtered
                     ↓
            SLAM / Nav2
```

## Thiết lập hệ thống TF tree chuẩn xác.
Trong ROS 2, TF tree đóng vai trò là hệ quy chiếu để liên kết tất cả các frame (base_link, odom, map, sensor frames,...) của robot lại với nhau. Việc thiết lập TF tree chuẩn và nhất quán là cực kỳ quan trọng vì:

- Đảm bảo các dữ liệu cảm biến (LiDAR, IMU, Camera) được biểu diễn đúng vị trí và hướng trong không gian
- Giúp các module như SLAM, Localization và Nav2 hiểu đúng vị trí thực của robot
- Tránh lỗi sai lệch tọa độ (drift, nhảy frame, lệch map) khi robot di chuyển
- Là nền tảng để các thuật toán lập kế hoạch và điều khiển hoạt động chính xác

Một TF tree sai hoặc thiếu sẽ dẫn đến các lỗi nghiêm trọng như:

- Robot định vị sai vị trí
- Costmap hiển thị lệch vật cản
- Planner tạo quỹ đạo sai

Vì vậy, việc thiết kế và kiểm tra TF tree (đúng frame, đúng parent-child, đúng timestamp) là bước bắt buộc trong hệ thống robot tự hành.

![TF Tree](image/frames.png)

## Xây dựng bản đồ trong môi trường phức tạp (SLAM & Mapping)
Trong hệ thống này, robot sử dụng các thuật toán SLAM (Simultaneous Localization and Mapping) để đồng thời xác định vị trí và xây dựng bản đồ môi trường dưới dạng Occupancy Grid 2D.

![SLAM](image/scan_map.gif)

Môi trường bệnh viện đặt ra nhiều thách thức:

- Hành lang dài, ít đặc trưng → dễ gây sai lệch định vị
- Các phòng có cấu trúc giống nhau → gây nhầm lẫn (perceptual aliasing)
- Nhiễu từ cảm biến và sai số odometry

Để cải thiện độ chính xác, hệ thống:

- Kết hợp dữ liệu từ LiDAR + IMU + Odometry
- Sử dụng EKF (Extended Kalman Filter) để hợp nhất dữ liệu (sensor fusion) và lọc nhiễu trạng thái robot
- Áp dụng các thuật toán SLAM hiện đại trong ROS 2 nhằm giảm drift và giữ tính nhất quán của bản đồ

Nhờ đó, robot có thể xây dựng bản đồ ổn định và chính xác, làm nền tảng cho các bước Localization và Navigation (Nav2) phía sau.
### example
```
cd ~/ros2_ws
source install/setup.bash
ros2 launch robot_omni slam.py
```
```
ros2 run nav2_map_server map_saver_cli -f my_map
```

## Cấu hình các sever phục vụ cho Nav2 để tối ưu hóa quỹ đạo di chuyển và khả năng tránh vật cản động/tĩnh.

`File cấu hình : params.yaml`

### AMCL

AMCL (Adaptive Monte Carlo Localization) là một thuật toán định vị robot trong ROS/ROS2 (đặc biệt dùng trong Nav2). Nó sử dụng phương pháp lọc hạt (particle filter) để ước lượng vị trí và hướng của robot trên bản đồ có sẵn.

Nguyên lý hoạt động (ngắn gọn)
- Robot không biết chính xác mình đang ở đâu → AMCL tạo ra hàng trăm đến hàng nghìn “hạt” (particles) giả định vị trí.
- Mỗi hạt được đánh giá dựa trên:
- Dữ liệu odometry (chuyển động robot)
- Dữ liệu LaserScan (so sánh với map)
- Các hạt phù hợp sẽ được giữ lại, hạt sai bị loại bỏ.
- Sau nhiều vòng lặp → các hạt hội tụ → cho ra pose chính xác của robot.

**Vai trò của AMCL trong hệ Nav2**

AMCL là thành phần cốt lõi để định vị, cụ thể:

- Xác định vị trí robot trên bản đồ (map → base_link)
- Cập nhật liên tục khi robot di chuyển
- Cung cấp pose cho:
  - Planner (lập kế hoạch đường đi)
  - Controller (điều khiển robot)
  - Publish TF: map → odom

### BT Navigator

BT Navigator (Behavior Tree Navigator) là thành phần trong Nav2 dùng Behavior Tree để điều phối toàn bộ quá trình điều hướng của robot. Nó không trực tiếp lập đường hay điều khiển robot, mà đóng vai trò: Quyết định robot nên làm gì, làm theo thứ tự nào để tới mục tiêu

Nguyên lý hoạt động: 
- Robot nhận goal → BT Navigator kích hoạt Behavior Tree
- Cây hành vi sẽ chạy theo từng bước:
- Gọi Planner để tạo đường
- Gọi Controller để đi theo đường
- Trong quá trình chạy:
  - Nếu đi lệch → replan
  - Nếu thất bại → recovery
- Quá trình này lặp liên tục cho đến khi: tới đích hoặc fail hoàn toàn

**Vai trò của BT Navigator trong hệ Nav2**  

BT Navigator là bộ điều phối logic, cụ thể:

- Nhận goal từ user (NavigateToPose, NavigateThroughPoses)
- Quyết định luồng xử lý:
- khi nào lập đường
- khi nào điều khiển
- khi nào lập lại đường
- khi nào xử lý lỗi
- Điều phối các server:
  - Planner
  - Controller
  - Recovery
- Xử lý các tình huống:
  - không tìm được path
  - robot bị kẹt
  - vật cản xuất hiện
