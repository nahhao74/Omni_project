# PROJECT ROBOT OMNI
# ROS 2 Nav2: Autonomous Omni-directional Hospital Navigation Robot

Tác giả: Nguyễn Anh Hào & Trần Minh Cương
Email: nahhao74@gmail.com , tmcuong0507@gmail.com

Dự án này triển khai hệ thống điều hướng tự hành toàn diện cho một robot đa hướng (omni-directional robot) hoạt động trong môi trường mô phỏng bệnh viện (AWS Hospital World). Hệ thống được xây dựng trên nền tảng ROS 2 Jazzy và thư viện Nav2 stack trong môi trường mô phỏng Gazebo.

Điểm nhấn của dự án nằm ở việc giải quyết bài toán động học phức tạp của xe omni trong không gian hẹp. Hệ thống sử dụng Smac A* làm Global Planner để tạo ra các đường đi an toàn, có tính toán đến footprint của robot. Ở cấp độ điều khiển, dự án kết hợp Rotation Shim Controller để xử lý các góc quay ưu tiên, và MPPI Controller để dự đoán và lấy mẫu quỹ đạo trong thời gian thực, giúp robot lách qua các vật cản động một cách mượt mà nhất.
**Phạm vi của dự án bao gồm:**
1. Xây dựng mô hình robot và môi trường mô phỏng vật lý có độ tin cậy cao.
2. Xử lý tín hiệu đầu vào từ cảm biến (LiDAR, IMU, Camera) và dữ liệu Odometry.
3. Thiết lập hệ thống TF tree chuẩn xác.
4. Xây dựng bản đồ trong môi trường phức tạp (SLAM & Mapping) Sử dụng các thuật toán SLAM hiện đại trên ROS 2 để quét và xây dựng bản đồ Occupancy Grid 2D từ môi trường AWS Hospital World, xử lý nhiễu từ các hành lang      dài và các phòng bệnh có cấu trúc giống nhau.
5. Tinh chỉnh các tham số cấu hình của Nav2 để tối ưu hóa quỹ đạo di chuyển và khả năng tránh vật cản động/tĩnh.

# Setup
**1. Yêu cầu hệ thống:**

Ubuntu 24.04

ROS 2 Jazzy

Gazebo Simulator

AWS Robotic Hospital World packages

**2. Tạo Workspace và cài đặt thư viện:**

    Tạo ROS 2 workspace

mkdir -p ~/ros_ws/src
cd ~/ros_ws/src

    Clone repository của dự án

git clone https://github.com/nahhao74/Omni_project.git

    Build

colcon build

    Khai báo đường dẫn

source install/setup.bash

**3. Khởi chạy file launch**

ros2 launch robot_omni localization_launch.py

✨ Tính năng & Demo

    Xây dựng bản đồ trong môi trường phức tạp (SLAM & Mapping) Sử dụng các thuật toán SLAM hiện đại trên ROS 2 để quét và xây dựng bản đồ Occupancy Grid 2D từ môi trường AWS Hospital World, xử lý nhiễu từ các hành lang dài và các phòng bệnh có cấu trúc giống nhau.
    Lập kế hoạch đường đi toàn cục với Smac A* (Global Planning) Smac A* Planner tính toán đường đi tối ưu nhất từ vị trí hiện tại đến phòng bệnh mục tiêu. Thuật toán cân nhắc chi phí về khoảng cách cũng như vùng rủi ro, đảm bảo quỹ đạo không bo góc quá gắt gây va chạm tường.
    Điều hướng mượt mà với MPPI & Rotation Shim Controller (Local Control) Rotation Shim giúp robot xoay hướng mặt về phía mục tiêu trước khi tiến. Ngay sau đó, MPPI Controller sẽ liên tục lấy mẫu hàng ngàn quỹ đạo để chọn ra hướng đi linh hoạt nhất, giúp đạt tới vị trí đích với đường đi tối ưu nhất và tránh va chạm.

# Acknowledgments

Dự án này được phát triển dựa trên nền tảng của nhiều thư viện mã nguồn mở và tài liệu tham khảo từ cộng đồng ROS. Xin gửi lời cảm ơn đặc biệt đến:

    Cartographer ROS / TurtleBot3: Các file cấu hình Launch và SLAM (như cartographer_node) được xây dựng dựa trên mã nguồn gốc của tác giả Darby Lim (Bản quyền 2019 thuộc Open Source Robotics Foundation, Inc. - Giấy phép Apache 2.0). Cấu trúc gốc đã được tùy biến lại (remapping) để có thể hợp nhất dữ liệu từ hệ thống Lidar kép và Odometry cho robot di chuyển đa hướng.
    Nav2 & MPPI: https://docs.nav2.org/configuration/packages/configuring-mppic.html
    AWS Hospital World: https://github.com/aws-robotics/aws-robomaker-hospital-world.git

# Xây dưng mô hình robot
1. Hình dạng kích thước .
    Loại robot: Xe 4 bánh đa hướng (Omni/Mecanum).
    Trọng lượng và Kích thước: Phần thân chính (base_link) có khối lượng hơn 34 kg, kích thước bao phủ khoảng 0.71m x 0.49m x 0.16m. Hệ thống bánh xe: 4 bánh (trước-phải, trước-trái, sau-phải, sau-trái) được cấp lệnh vận tốc độc lập thông qua plugin gz_ros2_control.
2. Hệ thống Cảm biến (Sensors)
    Mẫu robot này được cấu hình hình để xây dựng bản đồ (SLAM) và điều hướng (Nav2), vì vậy cần phải có các cảm biến phù hợp:
    Hệ thống quét LiDAR kép (SICK TiM551): Xe được trang bị hai cảm biến quét laser, một đặt ở góc phía trước (base_front_laser) và một ở góc phía sau (base_rear_laser). Mỗi LiDAR có góc quét rộng lên tới 270 độ và tầm nhìn xa 25 mét (với noise tiêu chuẩn). Việc bố trí chéo nhau như thế này sinh ra là để tạo ra góc nhìn 360 độ hoàn hảo xung quanh xe, loại bỏ hoàn toàn điểm mù khi đưa vào Costmap.
    Hệ thống Camera Chiều sâu (RealSense D435): Xe có sẵn 2 camera RGBD. Một chiếc gắn ngay trên thân base và một chiếc gắn trên phần giá đỡ mở rộng (cameras_add_on_link) ở trên nóc. Rất phù hợp để nhận dạng vật thể 3D phía trước.
    Cảm biến quán tính (IMU): Cập nhật dữ liệu gia tốc và góc quay liên tục ở tần số cao 100Hz, giúp bộ lọc nhiễu odometry tính toán vị trí chuẩn xác hơn khi xe thực hiện các thao tác xoay phức tạp.

3. Tương thích Hệ thống
    ROS 2 & Gazebo: File này được viết để chạy trơn tru trên các phiên bản ROS 2 mới (có tham chiếu đến đường dẫn của bản Jazzy) và sử dụng các plugin hiện đại của Gazebo mới (Ignition/gz-sim) như **gz_ros2_control::GazeboSimSystem** hay **gz-sim-imu-system**.
<img width="927" height="681" alt="image" src="https://github.com/user-attachments/assets/f4e3c76a-80d7-4ed1-9f35-0749ab168c47" />
# Xử lý tín hiệu cảm biến
    Để dùng được cảm biến trong Ros2 thì cần phải có plugin liên quan đến cảm biến, được khai báo trong file hospital_aws.world và hospital_full.wotld :
    
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
    
**1. Danh sách cảm biến & chức năng**
**LiDAR**
    LiDAR là cảm biến quan trọng nhất cho navigation vì nó cung cấp thông tin khoảng cách chính xác theo dạng 2D scan.Cụ thể hệ thống của bạn dùng LiDAR để:Phát hiện vật cản, xây dựng bản đồ (SLAM), xác định vị trí (Localization), navigation thông qua cost map. b
    Project này sử dụng 2 cảm biến lidar để ao phủ góc quét 360 độ:
            **Lidar trước**
      <gazebo reference="base_front_laser_link">
        <sensor name="base_front_laser" type="gpu_lidar">
          <pose>0 0 0 0 0 0</pose>
          <update_rate>10</update_rate>
          <visualize>true</visualize>
          <topic>scan_front_raw</topic>
          <gz_frame_id>base_front_laser_link</gz_frame_id>
          <ray>
            <scan>
              <horizontal>
                <!-- 818 (270/0.33) steps in 270deg fov -->
                <samples>818.0</samples>
                <resolution>1</resolution>
                <!-- not the sensor resolution; just 1 -->
                <min_angle>-2.356194490192345</min_angle>
                <max_angle>2.356194490192345</max_angle>
              </horizontal>
            </scan>
            <range>
              <min>0.05</min>
              <max>25.0</max>
              <resolution>0.001</resolution>
            </range>
            <noise>
              <type>gaussian</type>
              <mean>0.0</mean>
              <stddev>0.01</stddev>
            </noise>
          </ray>
        </sensor>
        <material>Gazebo/DarkGrey</material>
      </gazebo>
            **Lidar sau**
      <gazebo reference="base_rear_laser_link">
        <sensor name="base_rear_laser" type="gpu_lidar">
          <pose>0 0 0 0 0 0</pose>
          <update_rate>10</update_rate>
          <visualize>true</visualize>
          <topic>scan_rear_raw</topic>
          <gz_frame_id>base_rear_laser_link</gz_frame_id>
          <ray>
            <scan>
              <horizontal>
                <!-- 818 (270/0.33) steps in 270deg fov -->
                <samples>818.0</samples>
                <resolution>1</resolution>
                <!-- not the sensor resolution; just 1 -->
                <min_angle>-2.356194490192345</min_angle>
                <max_angle>2.356194490192345</max_angle>
              </horizontal>
            </scan>
            <range>
              <min>0.05</min>
              <max>25.0</max>
              <resolution>0.001</resolution>
            </range>
            <noise>
              <type>gaussian</type>
              <mean>0.0</mean>
              <stddev>0.01</stddev>
            </noise>
          </ray>
        </sensor>
        <material>Gazebo/DarkGrey</material>
      </gazebo>



    


