#PROJECT ROBOT OMNI
#ROS 2 Nav2: Omni-directional Robot Navigation

Dự án này triển khai hệ thống điều hướng tự động (Navigation) sử dụng **ROS2 (Jazzy), Gazebo (Harmonic)** và **Nav2** cho việc điều hướng (omni-directional robot). 

Project tập trung vào việc xây dựng mô hình mô phỏng hoàn chỉnh trong Gazebo, xử lý thu thập thông tin từ môi trường mô phỏng bằng cách sử dụng cảm biến LiDAR, IMU, Camera để tối ưu tầm nhìn, và tinh chỉnh sâu các thông số động học để robot di chuyển mượt mà, tự động né vật cản và lập kế hoạch đường đi u trong môi trường.

**Phạm vi của dự án bao gồm:**
1. Xây dựng mô hình robot và môi trường mô phỏng vật lý có độ tin cậy cao.
2. Xử lý tín hiệu đầu vào từ cảm biến (LiDAR, IMU, Camera) và dữ liệu Odometry.
3. Thiết lập hệ thống TF tree chuẩn xác.
4. Tinh chỉnh các tham số cấu hình của Nav2 để tối ưu hóa quỹ đạo di chuyển và khả năng tránh vật cản động/tĩnh.

**I.Xây dưng mô hình robot**
1. Hình dạng kích thước .
    Loại robot: Xe 4 bánh đa hướng (Omni/Mecanum).
    Trọng lượng và Kích thước: Phần thân chính (base_link) có khối lượng hơn 34 kg, kích thước bao phủ khoảng 0.71m x 0.49m x 0.16m. Hệ thống bánh xe: 4 bánh (trước-phải, trước-trái, sau-phải, sau-trái) được cấp lệnh vận tốc độc lập thông qua plugin gz_ros2_control.
2. Hệ thống Cảm biến (Sensors)
    Mẫu robot này được cấu hình hình để xây dựng bản đồ (SLAM) và điều hướng (Nav2), vì vậy cần phải có các cảm biến phù hợp:
    Hệ thống quét LiDAR kép (SICK TiM551): Xe được trang bị hai cảm biến quét laser, một đặt ở góc phía trước (base_front_laser) và một ở góc phía sau (base_rear_laser). Mỗi LiDAR có góc quét rộng lên tới 270 độ và tầm nhìn xa 25 mét (với noise tiêu chuẩn). Việc bố trí chéo nhau như thế này sinh ra là để tạo ra góc nhìn 360 độ hoàn hảo xung quanh xe, loại bỏ hoàn toàn điểm mù khi đưa vào Costmap.
    Hệ thống Camera Chiều sâu (RealSense D435): Xe có sẵn 2 camera RGBD. Một chiếc gắn ngay trên thân base và một chiếc gắn trên phần giá đỡ mở rộng (cameras_add_on_link) ở trên nóc. Rất phù hợp để nhận dạng vật thể 3D phía trước.
    Cảm biến quán tính (IMU): Cập nhật dữ liệu gia tốc và góc quay liên tục ở tần số cao 100Hz, giúp bộ lọc nhiễu odometry tính toán vị trí chuẩn xác hơn khi xe thực hiện các thao tác xoay phức tạp.

3. Tương thích Hệ thống
    ROS 2 & Gazebo: File này được viết để chạy trơn tru trên các phiên bản ROS 2 mới (có tham chiếu đến đường dẫn của bản Jazzy) và sử dụng các plugin hiện đại của Gazebo mới (Ignition/gz-sim) như gz_ros2_control::GazeboSimSystem hay gz-sim-imu-system.
Tóm lại, omni_base là một nền tảng robot xịn sò, mang hơi hướng công nghiệp. Các thông số từ cấu trúc vật lý đến cảm biến đều đã được tối ưu sẵn, tạo thành một base hoàn hảo để bạn 

<img width="927" height="681" alt="image" src="https://github.com/user-attachments/assets/f4e3c76a-80d7-4ed1-9f35-0749ab168c47" />

