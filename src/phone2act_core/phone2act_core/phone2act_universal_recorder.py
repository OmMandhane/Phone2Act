#!/usr/bin/env python3
"""
Phone2Act Universal Recorder (Hardware-Agnostic)

Synchronizes multi-camera RGB streams with robot state feedback and teleoperation 
commands. Records high-quality, VLA-compatible datasets (Meters/Radians) in 
Parquet and MP4 formats, ready for GR00T/LeRobot policy training.

Records:
- RGB streams (front + wrist)
- Robot state (joints + EE pose)
- Actions (delta pose + gripper)

Outputs:
- MP4 videos
- Parquet dataset (VLA-ready)

Design Goals:
- Robot-agnostic
- Clean synchronization
- Real-time safe
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from tf_transformations import euler_from_quaternion

import cv2
import json
import math
import threading
import time
from pathlib import Path
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq


# ==============================================================================
# HELPER: THREADED CAMERA
# ==============================================================================
class FastCam:
    """Reads camera frames in a background daemon thread to prevent I/O blocking."""
    def __init__(self, src: int, width: int, height: int, fps: int):
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.stream.set(cv2.CAP_PROP_FPS, fps)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        self.grabbed, self.frame = self.stream.read()
        self.stopped = False

    def start(self):
        threading.Thread(target=self.update, daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            if not self.grabbed:
                self.stop()
            else:
                self.grabbed, self.frame = self.stream.read()

    def read(self):
        return self.grabbed, self.frame

    def stop(self):
        self.stopped = True
        self.stream.release()


# ==============================================================================
# UNIVERSAL RECORDER NODE
# ==============================================================================
class UniversalRecorder(Node):
    def __init__(self, episode_index: int, task_index: int, task_description: str, start_frame_index: int):
        super().__init__("universal_recorder")
        
        # --- PARAMETERS ---
        self.declare_parameter('data_root', 'dataset_repo_universal')
        self.declare_parameter('chunk_name', 'chunk-000')
        self.declare_parameter('fps', 20.0)
        self.declare_parameter('cam_id_front', 0)
        self.declare_parameter('cam_id_wrist', 2)
        self.declare_parameter('resolution_width', 640)
        self.declare_parameter('resolution_height', 480)
        self.declare_parameter('joint_topic', '/joint_states')

        # Load parameter values
        self.DATA_ROOT = Path(self.get_parameter('data_root').value)
        self.CHUNK = self.get_parameter('chunk_name').value
        self.FPS = self.get_parameter('fps').value
        self.WIDTH = self.get_parameter('resolution_width').value
        self.HEIGHT = self.get_parameter('resolution_height').value
        
        # Episode Metadata
        self.episode_index = episode_index
        self.task_index = task_index
        self.task_description = task_description
        self.global_start_index = start_frame_index 
        
        # Timing State
        self.start_time_ros = None
        self.last_loop_time = self.get_clock().now()

        # RAM Buffers
        self.frames_front = [] 
        self.frames_wrist = [] 
        self.records = []
        
        # Hardware State Buffers
        self.latest_joints = None
        self.latest_robot_pose = None  
        self.latest_target_pose = None 
        self.latest_target_gripper = 1.0 
        self.last_target_time = 0 
        self.simulated_gripper_state = 1.0

        # Initialize Cameras
        self.get_logger().info("Starting Cameras (Hardware Threading Mode)...")
        self.cam_front = FastCam(self.get_parameter('cam_id_front').value, self.WIDTH, self.HEIGHT, self.FPS).start()
        self.cam_wrist = FastCam(self.get_parameter('cam_id_wrist').value, self.WIDTH, self.HEIGHT, self.FPS).start()
        time.sleep(1.0) 

        # --- SUBSCRIBERS (Hardware Agnostic) ---
        joint_topic = self.get_parameter('joint_topic').value
        self.create_subscription(JointState, joint_topic, self.cb_joint, 10)
        self.create_subscription(PoseStamped, "/phone2act/robot_feedback", self.cb_robot_feedback, 10)
        self.create_subscription(PoseStamped, "/phone2act/target_pose", self.cb_target_pose, 10)
        self.create_subscription(Float32, "/phone2act/gripper_cmd", self.cb_target_gripper, 10)

        # Control Loop
        self.create_timer(1.0 / self.FPS, self.loop)
        self.get_logger().info(f"Recording Episode {episode_index} | Task: '{task_description}'")

    # --- CALLBACKS ---
    def cb_joint(self, msg): 
        self.latest_joints = msg

    def cb_robot_feedback(self, msg):
        """Converts Standard Pose -> Dictionary (Meters/Radians)"""
        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        (rx, ry, rz) = euler_from_quaternion(q)
        self.latest_robot_pose = {
            'x': msg.pose.position.x, 'y': msg.pose.position.y, 'z': msg.pose.position.z,
            'rx': rx, 'ry': ry, 'rz': rz
        }

    def cb_target_pose(self, msg): 
        """Converts Target Pose -> List (Meters/Radians)"""
        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        (rx, ry, rz) = euler_from_quaternion(q)
        self.latest_target_pose = [
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
            rx, ry, rz
        ]
        self.last_target_time = time.time() 

    def cb_target_gripper(self, msg):
        self.latest_target_gripper = msg.data

    # --- MAIN RECORDING LOOP ---
    def loop(self):
        now_ros = self.get_clock().now()
        dt = (now_ros - self.last_loop_time).nanoseconds / 1e9
        self.last_loop_time = now_ros
        
        if dt > 0:
            print(f"\r[REC] Ep {self.episode_index} | Rate: {1.0/dt:.1f} Hz | Buffer: {len(self.records)}", end="")

        # 1. GET FRAMES
        ret1, frame1 = self.cam_front.read()
        ret2, frame2 = self.cam_wrist.read()
        if not ret1 or not ret2:
            return

        # 2. SYNC CHECK
        if self.latest_joints is None or self.latest_robot_pose is None:
            return 
        
        # Drop frame if teleop command is stale (>0.5s old)
        if (time.time() - self.last_target_time) > 0.5 and self.latest_target_pose is not None:
             return
             
        if self.start_time_ros is None:
            self.start_time_ros = now_ros

        # 3. BUFFER IMAGES
        self.frames_front.append(frame1.copy())
        self.frames_wrist.append(frame2.copy())

        # 4. CURRENT STATE (Meters, Radians)
        curr = np.array([
            self.latest_robot_pose['x'], self.latest_robot_pose['y'], self.latest_robot_pose['z'], 
            self.latest_robot_pose['rx'], self.latest_robot_pose['ry'], self.latest_robot_pose['rz']
        ])

        # 5. ACTION DELTA CALCULATION
        if self.latest_target_pose is None:
            action_delta = [0.0] * 6 
        else:
            tgt = np.array(self.latest_target_pose)
            xyz_delta = tgt[:3] - curr[:3]
            
            # Shortest-path angle wrapping for radians
            rpy_delta = tgt[3:] - curr[3:]
            rpy_delta = (rpy_delta + math.pi) % (2 * math.pi) - math.pi
            
            action_delta = list(xyz_delta) + list(rpy_delta)

        # 6. PARQUET RECORD FORMATTING
        state_vec = list(self.latest_joints.position) + list(curr) + [self.simulated_gripper_state]
        action_vec = action_delta + [self.latest_target_gripper]
        ts_value = (now_ros - self.start_time_ros).nanoseconds / 1e9

        record = {
            "observation.state": np.array(state_vec, dtype=np.float32), 
            "action": np.array(action_vec, dtype=np.float32), 
            "timestamp": float(ts_value),
            "episode_index": self.episode_index,
            "index": self.global_start_index + len(self.records),
            "task_index": self.task_index,
            "annotation.human.action.task_description": self.task_index, 
            "annotation.human.validity": 1,
            "next.done": False, 
            "next.reward": 0.0
        }
        self.records.append(record)
        self.simulated_gripper_state = self.latest_target_gripper

    def stop_hardware(self):
        print("\n\n[STOP] Shutting down camera streams...")
        self.cam_front.stop()
        self.cam_wrist.stop()

    def save_to_disk(self):
        print(f"[SAVE] Writing {len(self.records)} frames to disk...")

        path_f = self.DATA_ROOT / "videos" / self.CHUNK / "observation.images.front"
        path_w = self.DATA_ROOT / "videos" / self.CHUNK / "observation.images.wrist"
        path_f.mkdir(parents=True, exist_ok=True)
        path_w.mkdir(parents=True, exist_ok=True)
        
        vid_name = f"episode_{self.episode_index:06d}.mp4"
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        
        out_f = cv2.VideoWriter(str(path_f / vid_name), fourcc, self.FPS, (self.WIDTH, self.HEIGHT))
        out_w = cv2.VideoWriter(str(path_w / vid_name), fourcc, self.FPS, (self.WIDTH, self.HEIGHT))

        for i in range(len(self.frames_front)):
            out_f.write(self.frames_front[i])
            out_w.write(self.frames_wrist[i])
        
        out_f.release()
        out_w.release()

        p_path = self.DATA_ROOT / "data" / self.CHUNK / f"episode_{self.episode_index:06d}.parquet"
        p_path.parent.mkdir(parents=True, exist_ok=True)
        
        if len(self.records) > 0:
            table = pa.Table.from_pylist(self.records)
            pq.write_table(table, str(p_path))
            
            meta_entry = {
                "episode_index": self.episode_index, 
                "tasks": [self.task_description], 
                "length": len(self.records)
            }
            
            meta_file = self.DATA_ROOT / "meta" / "episodes.jsonl"
            with open(meta_file, "a") as f:
                f.write(json.dumps(meta_entry) + "\n")
            
            update_info_stats(self.DATA_ROOT)
            print("[DONE] Dataset saved successfully.")


# ==============================================================================
# METADATA HELPERS
# ==============================================================================
def ensure_metadata_files(data_root: Path, fps: float):
    meta_dir = data_root / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)
    
    info_path = meta_dir / "info.json"
    if not info_path.exists():
        info_data = {
            "codebase_version": "v2.0",
            "robot_type": "universal_vla", 
            "total_episodes": 0,
            "total_frames": 0,
            "fps": fps,
            "splits": { "train": "0:0" },
            "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
            "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
            "features": {
                "observation.state": { "dtype": "float32", "shape": [13], "names": ["joints", "eef_pos_m", "gripper"] },
                "action": { "dtype": "float32", "shape": [7], "names": ["delta_eef_m", "gripper"] },
            }
        }
        with open(info_path, "w") as f:
            json.dump(info_data, f, indent=4)
    
    tasks_path = meta_dir / "tasks.jsonl"
    if not tasks_path.exists():
        tasks_path.touch()

def update_info_stats(data_root: Path):
    ep_path = data_root / "meta" / "episodes.jsonl"
    info_path = data_root / "meta" / "info.json"
    
    if ep_path.exists() and info_path.exists():
        total_eps = 0
        total_frames = 0
        with open(ep_path, 'r') as f:
            for line in f:
                try: 
                    d = json.loads(line)
                    total_eps += 1
                    total_frames += d.get("length", 0)
                except json.JSONDecodeError: 
                    pass
                    
        with open(info_path, 'r') as f: 
            data = json.load(f)
            
        data["total_episodes"] = total_eps
        data["total_frames"] = total_frames
        data["splits"]["train"] = f"0:{total_eps}"
        
        with open(info_path, 'w') as f: 
            json.dump(data, f, indent=4)

def get_current_frame_count(data_root: Path):
    ep_path = data_root / "meta" / "episodes.jsonl"
    total_frames = 0
    if ep_path.exists():
        with open(ep_path, 'r') as f:
            for line in f:
                try: 
                    total_frames += json.loads(line).get("length", 0)
                except json.JSONDecodeError: 
                    pass
    return total_frames

def get_task_id(data_root: Path, desc: str):
    t_path = data_root / "meta" / "tasks.jsonl"
    tasks = {}
    if t_path.exists():
        with open(t_path, "r") as f:
            for line in f:
                try: 
                    d = json.loads(line)
                    tasks[d["task"]] = d["task_index"]
                except json.JSONDecodeError: 
                    pass
                    
    if desc in tasks: 
        return tasks[desc]
        
    new_id = max(tasks.values()) + 1 if tasks else 0
    with open(t_path, "a") as f: 
        f.write(json.dumps({"task_index": new_id, "task": desc}) + "\n")
    return new_id 


# ==============================================================================
# MAIN EXECUTABLE
# ==============================================================================
def main():
    rclpy.init()
    
    # We initialize a temporary node just to read parameters cleanly 
    # before kicking off the interactive prompt
    temp_node = rclpy.create_node('temp_param_reader')
    temp_node.declare_parameter('data_root', 'dataset_repo_universal')
    temp_node.declare_parameter('chunk_name', 'chunk-000')
    temp_node.declare_parameter('fps', 20.0)
    
    data_root = Path(temp_node.get_parameter('data_root').value)
    chunk_name = temp_node.get_parameter('chunk_name').value
    fps = temp_node.get_parameter('fps').value
    temp_node.destroy_node()
    
    ensure_metadata_files(data_root, fps)
    
    print("\n" + "="*50)
    print(" 🎬 PHONE2ACT UNIVERSAL RECORDER")
    print("="*50)
    desc = input("Task Description (e.g., 'pick object'): ").strip() or "pick object"
    
    ep_idx = 0 
    chunk_dir = data_root / "data" / chunk_name
    if chunk_dir.exists():
        files = [f.name for f in chunk_dir.iterdir() if f.name.endswith(".parquet")]
        if files: 
            ep_idx = max([int(f.split("_")[1].split(".")[0]) for f in files]) + 1

    start_frame_idx = get_current_frame_count(data_root)

    print(f"\n[READY] Episode: {ep_idx} | Global Frame Start: {start_frame_idx} | Task: '{desc}'")
    input(">>> Press [ENTER] to begin recording...")
    
    rec = UniversalRecorder(ep_idx, get_task_id(data_root, desc), desc, start_frame_idx)
    
    try: 
        rclpy.spin(rec)
    except KeyboardInterrupt: 
        pass 
    finally:
        rec.stop_hardware()
        if len(rec.records) > 0:
            if input(">>> SAVE DATASET? (y/n): ").strip().lower() == 'y': 
                rec.save_to_disk()
            else: 
                print("[DISCARDED] Data was not saved.")
        if rclpy.ok(): 
            rclpy.shutdown()

if __name__ == "__main__": 
    main()