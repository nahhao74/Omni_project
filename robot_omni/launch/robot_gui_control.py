#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import Buffer, TransformListener

import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import math
import time
import matplotlib.image as mpimg
import os
import numpy as np

# Import thuật toán GA
try:
    from ga_path_optimizer import optimize_route
except ImportError:
    print("⚠️ CẢNH BÁO: Không tìm thấy file ga_path_optimizer.py.")

# === CẤU HÌNH BẢN ĐỒ NỀN (PGM) ===
MAP_IMAGE_PATH = "/home/nahhao74/ros2_ws/src/robot_omni/maps/hospital_map_final.pgm"
MAP_RESOLUTION = 0.050
MAP_ORIGIN = [-18.764, -54.765, 0]

# --- DỮ LIỆU TỌA ĐỘ PHÒNG ---
RAW_ROOMS = [
    {"name": "START_POS", "x": 0.0, "y": 0.0},
    {"name": "room1", "x": 4.51, "y": 6.12},
    {"name": "room2", "x": -4.43, "y": 6.32},
    {"name": "room3", "x": 7.71, "y": 3.44},
    {"name": "room4", "x": -7.52, "y": 3.53},
    {"name": "room5", "x": 11.14, "y": 0.10},
    {"name": "room6", "x": -10.60, "y": -0.59},
    {"name": "room7", "x": 10.76, "y": -5.08},
    {"name": "room8", "x": -9.97, "y": -4.63},
    {"name": "room9", "x": 1.37, "y": -13.00},
    {"name": "room10", "x": -1.15, "y": -13.08},
    {"name": "room11", "x": 8.28, "y": -17.80},
    {"name": "room12", "x": -7.98, "y": -18.16},
    {"name": "room13", "x": 7.14, "y": -10.43},
    {"name": "room14", "x": -7.23, "y": -10.21},
    {"name": "room15", "x": 1.78, "y": -22.21},
    {"name": "room16", "x": -1.67, "y": -22.29},
    {"name": "room17", "x": 2.16, "y": -28.47},
    {"name": "room18", "x": -2.40, "y": -27.77},
    {"name": "room19", "x": 8.32, "y": -31.87},
    {"name": "room20", "x": -8.16, "y": -32.08},
    {"name": "room21", "x": 7.09, "y": -24.40},
    {"name": "room22", "x": -7.32, "y": -24.40},
    {"name": "room23", "x": 1.43, "y": -38.35},
    {"name": "room24", "x": -2.21, "y": -40.58},
    {"name": "room25", "x": -7.97, "y": -39.52},
    {"name": "room26", "x": -8.57, "y": -35.84}
]

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('gui_robot_controller', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        self.cmd_pub = self.create_publisher(TwistStamped, '/mobile_base_controller/reference', 100)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.current_twist = TwistStamped()
        self.current_twist.header.frame_id = 'base_link'
        self.is_teleop_active = False
        
        self.create_timer(0.05, self.publish_teleop_cmd)

    def publish_teleop_cmd(self):
        if self.is_teleop_active:
            self.current_twist.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(self.current_twist)

    def stop_robot(self):
        self.current_twist.twist.linear.x = 0.0
        self.current_twist.twist.linear.y = 0.0
        self.current_twist.twist.angular.z = 0.0
        self.current_twist.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(self.current_twist)

class RobotGUI:
    def __init__(self, root, ros_node):
        self.root = root
        self.node = ros_node
        self.root.title("Trạm Điều Khiển Xe AI")
        self.root.geometry("1200x800")

        self.speed_lin = 0.5
        self.speed_ang = 1.0
        self.keys_pressed = set()
        
        self.navigator = None
        self.auto_nav_thread = None
        self.stop_auto_flag = False
        
        self.run_start_time = 0.0

        self.setup_ui()
        self.bind_keys()
        self.draw_map()

        threading.Thread(target=self.init_navigator, daemon=True).start()

    def init_navigator(self):
        self.log_msg("⏳ Đang kết nối với Nav2...")
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.log_msg("✅ Nav2 đã sẵn sàng! Đang tìm tọa độ thực của xe...")
        
        tf_success = False
        for _ in range(50):
            try:
                self.update_robot_pose()
                tf_success = True
                self.log_msg("📍 Đã chốt vị trí khởi tạo của xe.")
                break
            except Exception:
                time.sleep(0.1)
                
        if not tf_success:
            self.log_msg("⚠️ LỖI: Không tìm thấy hệ tọa độ TF. Auto có thể chạy sai!")

    def setup_ui(self):
        control_frame = ttk.Frame(self.root, width=350, padding=10)
        control_frame.pack(side=tk.LEFT, fill=tk.Y)

        ttk.Label(control_frame, text="CHẾ ĐỘ ĐIỀU KHIỂN", font=("Arial", 14, "bold")).pack(pady=5)

        self.mode_var = tk.StringVar(value="MANUAL")
        ttk.Radiobutton(control_frame, text="🕹️ Lái Thủ Công (/reference)", variable=self.mode_var, value="MANUAL", command=self.switch_mode).pack(anchor=tk.W)
        ttk.Radiobutton(control_frame, text="🤖 Tự Động (Nav2)", variable=self.mode_var, value="AUTO", command=self.switch_mode).pack(anchor=tk.W)

        ttk.Separator(control_frame, orient='horizontal').pack(fill=tk.X, pady=10)

        ttk.Label(control_frame, text="LỘ TRÌNH (Bấm vào Map để thêm):", font=("Arial", 10, "bold"), foreground="blue").pack(anchor=tk.W)

        self.queue_listbox = tk.Listbox(control_frame, height=8)
        self.queue_listbox.pack(fill=tk.X, pady=5)

        btn_list_frame = ttk.Frame(control_frame)
        btn_list_frame.pack(fill=tk.X)
        
        ttk.Button(btn_list_frame, text="🗑️ Xóa Chọn", command=self.remove_selected).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=1)
        ttk.Button(btn_list_frame, text="🧹 Xóa Hết", command=self.clear_list).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=1)
        
        self.btn_run_custom = ttk.Button(btn_list_frame, text="▶️ CHẠY THỨ TỰ", command=lambda: self.start_auto("CUSTOM"))
        self.btn_run_custom.pack(side=tk.RIGHT, expand=True, fill=tk.X, padx=1)

        ttk.Label(control_frame, text="TỐI ƯU HÓA AI:", font=("Arial", 10, "bold"), foreground="purple").pack(anchor=tk.W, pady=(15,0))
        self.btn_ga = ttk.Button(control_frame, text="🧠 Chạy GA Tối ưu List", command=lambda: self.start_auto("GA"))
        self.btn_ga.pack(fill=tk.X, pady=5)

        self.btn_stop = tk.Button(control_frame, text="🛑 DỪNG KHẨN CẤP", bg="red", fg="white", font=("Arial", 12, "bold"), command=self.stop_all)
        self.btn_stop.pack(fill=tk.X, pady=20)

        self.log_box = tk.Text(control_frame, height=10, width=35, font=("Consolas", 9))
        self.log_box.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

        self.map_frame = ttk.Frame(self.root)
        self.map_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.figure, self.ax = plt.subplots(figsize=(8, 9))
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.map_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        self.canvas.mpl_connect('button_press_event', self.on_map_click)
        
        self.switch_mode()

    def clear_list(self):
        self.queue_listbox.delete(0, tk.END)
        self.draw_map() 
        self.log_msg("🧹 Đã xóa toàn bộ lộ trình.")

    def remove_selected(self):
        selected_indices = self.queue_listbox.curselection()
        if not selected_indices:
            messagebox.showinfo("Hướng dẫn", "Vui lòng click bôi xanh một trạm trong danh sách bên trên để xóa!")
            return
        
        for index in reversed(selected_indices):
            room_name = self.queue_listbox.get(index)
            self.queue_listbox.delete(index)
            self.log_msg(f"✂️ Đã xóa trạm: {room_name}")
            
        self.draw_map() 

    def log_msg(self, msg):
        self.log_box.insert(tk.END, msg + "\n")
        self.log_box.see(tk.END)

    def switch_mode(self):
        mode = self.mode_var.get()
        if mode == "MANUAL":
            self.node.is_teleop_active = True
            if self.navigator:
                self.navigator.cancelTask()
            self.btn_run_custom.state(['disabled'])
            self.btn_ga.state(['disabled'])
            self.root.focus_set() 
        else:
            self.node.is_teleop_active = False
            self.node.stop_robot()
            self.btn_run_custom.state(['!disabled'])
            self.btn_ga.state(['!disabled'])

    def on_map_click(self, event):
        if event.xdata is None or event.ydata is None:
            return
        click_x, click_y = event.xdata, event.ydata
        
        min_dist = float('inf')
        closest_room = None
        
        for room in RAW_ROOMS[1:]: 
            dist = math.hypot(room["x"] - click_x, room["y"] - click_y)
            if dist < min_dist and dist < 1.5:
                min_dist = dist
                closest_room = room["name"]
                
        if closest_room:
            current_list = self.queue_listbox.get(0, tk.END)
            if current_list and current_list[-1] == closest_room:
                self.log_msg(f"⚠️ Đã bỏ qua: {closest_room} trùng với trạm vừa thêm.")
            else:
                self.queue_listbox.insert(tk.END, closest_room)
                self.log_msg(f"📍 Đã thêm trạm: {closest_room}")

    def bind_keys(self):
        self.root.bind_all('<KeyPress>', self.on_key_press)
        self.root.bind_all('<KeyRelease>', self.on_key_release)

    def on_key_press(self, event):
        if self.mode_var.get() != "MANUAL": return
        key = event.keysym.lower() if event.keysym else event.char.lower()
        if key == 'q': self.speed_lin *= 1.1
        elif key == 'z': self.speed_lin *= 0.9
        elif key == 'w': self.speed_ang *= 1.1
        elif key == 'x': self.speed_ang *= 0.9
        if key in ['i', 'comma', 'j', 'l', 'u', 'o', 'k']: self.keys_pressed.add(key)
        self.update_twist_from_keys()

    def on_key_release(self, event):
        if self.mode_var.get() != "MANUAL": return
        key = event.keysym.lower() if event.keysym else event.char.lower()
        if key in self.keys_pressed: self.keys_pressed.remove(key)
        self.update_twist_from_keys()

    def update_twist_from_keys(self):
        x, y, th = 0.0, 0.0, 0.0
        if 'i' in self.keys_pressed: x = 1.0
        if 'comma' in self.keys_pressed: x = -1.0
        if 'j' in self.keys_pressed: y = 1.0
        if 'l' in self.keys_pressed: y = -1.0
        if 'u' in self.keys_pressed: th = 1.0
        if 'o' in self.keys_pressed: th = -1.0
        if 'k' in self.keys_pressed: x, y, th = 0.0, 0.0, 0.0

        self.node.current_twist.twist.linear.x = x * self.speed_lin
        self.node.current_twist.twist.linear.y = y * self.speed_lin
        self.node.current_twist.twist.angular.z = th * self.speed_ang

    def update_robot_pose(self):
        try:
            trans = self.node.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            RAW_ROOMS[0]["x"] = x
            RAW_ROOMS[0]["y"] = y
            self.root.after(0, lambda: self.update_robot_marker_gui(x, y))
        except Exception:
            pass

    def update_robot_marker_gui(self, x, y):
        if hasattr(self, 'robot_marker') and hasattr(self, 'robot_text'):
            self.robot_marker.set_offsets([[x, y]])
            self.robot_text.set_position((x + 0.5, y + 0.5))
            self.canvas.draw_idle()

    # =======================================================================
    # BỘ NÃO BFS: QUÉT BẢN ĐỒ BẰNG HÌNH ẢNH ĐỂ NÉ TƯỜNG (SIÊU TỐC)
    # =======================================================================
    def process_ga_and_navigate(self, selected_rooms, original_indices):
        n = len(selected_rooms)
        self.log_msg(f"🕵️ Đang quét {n} trạm. Quá trình đọc ảnh PGM để né tường diễn ra cực nhanh...")

        try:
            # 1. Đọc và xử lý ảnh Map PGM
            img = mpimg.imread(MAP_IMAGE_PATH)
            if len(img.shape) > 2: img = img[:,:,0]
            img = np.flipud(img) 
            grid = (img > 200).astype(int) # 1: Đường đi, 0: Tường
            H, W = grid.shape

            def get_px(x, y):
                px = int((x - MAP_ORIGIN[0]) / MAP_RESOLUTION)
                py = int((y - MAP_ORIGIN[1]) / MAP_RESOLUTION)
                return max(0, min(W-1, px)), max(0, min(H-1, py))

            from collections import deque
            matrix = [[0.0]*n for _ in range(n)]

            # 2. Chạy thuật toán loang BFS cho từng phòng
            for i in range(n):
                if self.stop_auto_flag: return
                sx, sy = get_px(selected_rooms[i]["x"], selected_rooms[i]["y"])

                # Ép điểm xuất phát vào vùng trống nếu user click sát vách tường
                if grid[sy, sx] == 0:
                    for r in range(1, 10):
                        found = False
                        for dx in range(-r, r+1):
                            for dy in range(-r, r+1):
                                nx, ny = sx + dx, sy + dy
                                if 0 <= nx < W and 0 <= ny < H and grid[ny, nx] == 1:
                                    sx, sy = nx, ny
                                    found = True; break
                            if found: break
                        if found: break

                visited = np.zeros((H, W), dtype=bool)
                queue = deque([(sx, sy, 0.0)])
                visited[sy, sx] = True

                targets = {j: get_px(selected_rooms[j]["x"], selected_rooms[j]["y"]) for j in range(n) if i != j}
                found_dists = {}

                while queue and targets:
                    cx, cy, dist = queue.popleft()

                    to_remove = []
                    for j, (tx, ty) in targets.items():
                        if abs(cx - tx) <= 2 and abs(cy - ty) <= 2: 
                            found_dists[j] = dist * MAP_RESOLUTION
                            to_remove.append(j)
                    for j in to_remove:
                        del targets[j]

                    for dx, dy in [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,1), (1,-1), (-1,-1)]:
                        nx, ny = cx + dx, cy + dy
                        if 0 <= nx < W and 0 <= ny < H:
                            if not visited[ny, nx] and grid[ny, nx] == 1:
                                visited[ny, nx] = True
                                cost = 1.414 if dx != 0 and dy != 0 else 1.0
                                queue.append((nx, ny, dist + cost))

                for j in range(n):
                    if i == j:
                        matrix[i][j] = 0.0
                    else:
                        matrix[i][j] = found_dists.get(j, 9999.0) # Phạt nặng nếu tắc đường

        except Exception as e:
            self.log_msg(f"⚠️ Lỗi đọc PGM: {e}. Dùng Pytago dự phòng.")
            matrix = [[math.hypot(r1["x"]-r2["x"], r1["y"]-r2["y"]) for r2 in selected_rooms] for r1 in selected_rooms]

        self.log_msg("🧠 Đã xây xong Ma trận khoảng cách xuyên tường. Tối ưu GA...")
        
        try:
            optimized_sub_indices = optimize_route(matrix)
            route_indices = [original_indices[idx] for idx in optimized_sub_indices]
        except Exception as e:
            self.log_msg(f"⚠️ Lỗi GA: {e}")
            return

        self.root.after(0, lambda: self.draw_map(route_indices))
        self.run_navigation(route_indices)

    def start_auto(self, strategy):
        if not self.navigator:
            messagebox.showwarning("Chờ chút", "Nav2 chưa sẵn sàng!")
            return
            
        custom_names = self.queue_listbox.get(0, tk.END)
        if not custom_names:
            messagebox.showwarning("Trống", "Bạn chưa click chọn phòng nào trên bản đồ!")
            return

        if strategy == "GA" and len(custom_names) < 2:
            messagebox.showwarning("Chưa đủ trạm", "Vui lòng chọn ít nhất 2 phòng!")
            return

        self.stop_auto_flag = False
        self.run_start_time = time.time()
        self.update_robot_pose()
        
        selected_rooms = [RAW_ROOMS[0]] 
        original_indices = [0] 

        for name in custom_names:
            for i, r in enumerate(RAW_ROOMS):
                if r["name"] == name:
                    selected_rooms.append(r)
                    original_indices.append(i)
                    break

        if strategy == "CUSTOM":
            route_indices = original_indices 
            self.draw_map(route_indices)
            self.auto_nav_thread = threading.Thread(target=self.run_navigation, args=(route_indices,))
            self.auto_nav_thread.start()
            
        elif strategy == "GA":
            self.auto_nav_thread = threading.Thread(target=self.process_ga_and_navigate, args=(selected_rooms, original_indices))
            self.auto_nav_thread.start()

    def run_navigation(self, route_indices):
        final_route = route_indices[1:] 
        route_names = [RAW_ROOMS[i]["name"] for i in final_route]
        self.log_msg(f"🚀 XUẤT PHÁT: {' -> '.join(route_names)}")

        try:
            for i, idx in enumerate(final_route):
                if self.stop_auto_flag:
                    self.log_msg("🛑 Đã hủy hành trình!")
                    self.navigator.cancelTask()
                    break

                target_room = RAW_ROOMS[idx]
                self.log_msg(f"---> Trạm {i+1}: Tiến về {target_room['name']}")
                
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.node.get_clock().now().to_msg()
                pose.pose.position.x = float(target_room["x"])
                pose.pose.position.y = float(target_room["y"])
                pose.pose.orientation.w = 1.0 

                self.navigator.goToPose(pose)

                while not self.navigator.isTaskComplete():
                    if self.stop_auto_flag:
                        self.navigator.cancelTask()
                        break
                    time.sleep(0.5)

                if not self.stop_auto_flag:
                    res = self.navigator.getResult()
                    if res == TaskResult.SUCCEEDED:
                        self.log_msg(f"✅ Đã đến: {target_room['name']}")
                        self.update_robot_pose()
                        
                        remaining_route = route_indices[i+1:]
                        self.root.after(0, lambda r=remaining_route: self.draw_map(r))
                    else:
                        self.log_msg(f"❌ Thất bại tại: {target_room['name']}")
                        break
        finally:
            elapsed = time.time() - self.run_start_time
            mins = int(elapsed // 60)
            secs = int(elapsed % 60)
            self.log_msg(f"⏱️ Tổng thời gian chạy: {mins:02d} phút {secs:02d} giây")
            if not self.stop_auto_flag:
                self.log_msg("🏁 Hoàn thành chuyến đi.")

    def stop_all(self):
        self.stop_auto_flag = True
        self.node.stop_robot()
        if self.navigator:
            self.navigator.cancelTask()
        self.log_msg("🛑 ĐÃ DỪNG XE!")

    def draw_map(self, route_indices=None):
        self.ax.clear()
        if os.path.exists(MAP_IMAGE_PATH):
            try:
                img = mpimg.imread(MAP_IMAGE_PATH)
                img = np.flipud(img) 
                h, w = img.shape
                width_m = w * MAP_RESOLUTION
                height_m = h * MAP_RESOLUTION
                extent = [MAP_ORIGIN[0], MAP_ORIGIN[0] + width_m, MAP_ORIGIN[1], MAP_ORIGIN[1] + height_m]
                self.ax.imshow(img, cmap='gray', extent=extent, origin='lower')
            except Exception as e:
                self.log_msg(f"⚠️ Lỗi ảnh: {e}")
        else:
            self.log_msg("⚠️ Thiếu file PGM. Vẽ nền trắng.")

        for i, room in enumerate(RAW_ROOMS):
            if i == 0:
                self.robot_marker = self.ax.scatter(room["x"], room["y"], color='red', s=150, zorder=5)
                self.robot_text = self.ax.text(room["x"] + 0.5, room["y"] + 0.5, "XE", color='red', fontweight='bold',
                                               bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))
            else:
                self.ax.scatter(room["x"], room["y"], color='blue', s=30, zorder=5)
                self.ax.text(room["x"] + 0.5, room["y"] + 0.5, room["name"], color='blue', fontsize=8,
                             bbox=dict(facecolor='white', alpha=0.5, edgecolor='none'))

        if route_indices and len(route_indices) > 1:
            rx = [RAW_ROOMS[i]["x"] for i in route_indices]
            ry = [RAW_ROOMS[i]["y"] for i in route_indices]
            self.ax.plot(rx, ry, color='green', linestyle='-', linewidth=2.5, marker='o', zorder=4)
            
            for step, idx in enumerate(route_indices):
                if step == 0: continue 
                self.ax.text(RAW_ROOMS[idx]["x"] - 0.5, RAW_ROOMS[idx]["y"] - 1.5, f"({step})", 
                             color='green', fontweight='bold', fontsize=10,
                             bbox=dict(facecolor='yellow', alpha=0.8, edgecolor='none', pad=1))

        self.ax.set_title("Bản Đồ Bệnh Viện (Click để chọn trạm)")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_aspect('equal') 
        self.canvas.draw()

def main():
    rclpy.init()
    ros_node = RobotControllerNode()
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    root = tk.Tk()
    app = RobotGUI(root, ros_node)
    
    def on_closing():
        app.stop_all()
        executor.shutdown()
        ros_node.destroy_node()
        rclpy.shutdown()
        root.destroy()
        
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == '__main__':
    main()