import cv2
import json
import os
import numpy as np
from collections import defaultdict
import sys

class JSONAnnotationVisualizer:
    def __init__(self, video_path, json_path, output_path="annotated_video.mp4"):
        # 加载视频
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            raise ValueError(f"无法打开视频文件: {video_path}")
        
        # 获取视频属性
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.frame_count = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        print(f'w:{self.width}, h:{self.height}, fps:{self.fps}')
        
        # 创建输出视频
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.output = cv2.VideoWriter(output_path, fourcc, self.fps, (self.width, self.height))
        
        # 加载JSON标注 - 更健壮的方式
        self.annotations = self.load_annotations(json_path)
        
        # 按时间戳组织标注
        self.annotation_dict = defaultdict(list)
        for detection in self.annotations:
            # 确保检测项是字典
            if not isinstance(detection, dict):
                print(f"警告: 跳过非字典类型的标注项: {detection}")
                continue
                
            if "timestamp" in detection and "bboxes" in detection:
                timestamp = detection["timestamp"]
                self.annotation_dict[timestamp] = detection["bboxes"]
        
        # 获取所有时间戳并排序
        self.timestamps = sorted(self.annotation_dict.keys())
        print(f"加载了 {len(self.timestamps)} 个时间戳的标注")
        
        # 类型到名称的映射
        self.type_mapping = {
            4: "Person",
            6: "Car",
            13: "Traffic Cone",
            15: "Pedestrian",
            17: "Bicycle",
            21: "Motorcycle"
        }
        
        # 颜色表
        self.colors = self.generate_colors(len(self.type_mapping) if self.type_mapping else 10)
        
        # 跟踪历史
        self.track_history = defaultdict(list)
        
        # 保存输出路径
        self.output_path = output_path
    
    def load_annotations(self, json_path):
        """加载并解析JSON标注文件"""
        try:
            with open(json_path, 'r') as f:
                data = f.read()
                
            # 尝试解析为JSON对象
            annotations = json.loads(data)
            
            # 检查是否是我们期望的列表格式
            if isinstance(annotations, list):
                return annotations
            elif isinstance(annotations, dict):
                # 检查是否有包含检测结果的键
                possible_keys = ['detections', 'annotations', 'frames']
                for key in possible_keys:
                    if key in annotations and isinstance(annotations[key], list):
                        return annotations[key]
                # 如果没有找到预期的键，返回整个字典作为列表项
                return [annotations]
            else:
                print(f"警告: JSON文件的根元素不是列表或字典: {type(annotations)}")
                return []
                
        except Exception as e:
            print(f"加载JSON文件时出错: {e}")
            return []
    
    def generate_colors(self, n):
        """生成不同颜色列表"""
        return [tuple(np.random.randint(0, 255, 3).tolist()) for _ in range(n)]
    
    def get_closest_timestamp(self, current_time):
        """获取最接近当前时间的时间戳"""
        if not self.timestamps:
            return None
        
        # 找到最接近的时间戳
        closest = min(self.timestamps, key=lambda x: abs(x - current_time))
        
        # 如果时间差太大，则不使用该标注
        if abs(closest - current_time) > 1.0 / self.fps:
            return None
        
        return closest
    
    def draw_annotations(self, frame, annotations):
        """在帧上绘制标注"""
        for ann in annotations:
            # 确保标注是字典
            if not isinstance(ann, dict):
                print(f"警告: 跳过非字典类型的标注: {ann}")
                continue
                
            # 提取标注信息
            obj_id = ann.get("id", 0)
            obj_type = ann.get("type", 0)
            distance = ann.get("distance", 0.0)
            bbox = ann.get("bbox", [])
            
            # 确保边界框格式正确
            if len(bbox) != 4:
                print(f"警告: 跳过无效的边界框: {bbox}")
                continue
                
            x1, y1, x2, y2 = map(int, bbox)
            if x1 > x2 : 
                temp=x1
                x1=x2
                x2=temp
            
            if y1 > y2 : 
                temp=y1
                y1=y2
                y2=temp
            
            x1 = max(0, min(x1, self.width - 1))
            y1 = max(0, min(y1, self.height - 1))
            x2 = max(0, min(x2, self.width - 1))
            y2 = max(0, min(y2, self.height - 1))
            
            # 确保坐标在有效范围内
            if not (0 <= x1 < self.width and 0 <= x2 <= self.width and
                    0 <= y1 < self.height and 0 <= y2 <= self.height):
                print(f"警告: 跳过超出范围的边界框: {x1},{y1},{x2},{y2}")
                continue
            
            # 获取颜色和类型名称
            color = self.colors[obj_type % len(self.colors)] if self.colors else (0, 255, 0)
            type_name = self.type_mapping.get(obj_type, f"Type{obj_type}")
            
            # 绘制边界框
            print(x2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            
            # 绘制ID标签
            id_text = f"ID: {obj_id}"
            cv2.putText(frame, id_text, (x1, y1 - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            # 绘制类型和距离
            type_text = f"{type_name}: {distance:.1f}m"
            cv2.putText(frame, type_text, (x1, y2 + 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            # 更新跟踪历史
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            self.track_history[obj_id].append((center_x, center_y))
            
            # 绘制跟踪轨迹 (最近30帧)
            points = np.array(self.track_history[obj_id][-30:], dtype=np.int32)
            if len(points) > 1:
                cv2.polylines(frame, [points], False, color, 2)
    
    def visualize(self):
        """主函数：逐帧可视化标注"""
        if not self.timestamps:
            print("错误: 没有有效的时间戳数据")
            return
            
        frame_idx = 0
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            
            # 计算当前时间 (秒)
            current_time = frame_idx / self.fps
            
            # 获取最接近的标注
            closest_timestamp = self.get_closest_timestamp(current_time)
            
            # 绘制标注
            if closest_timestamp is not None:
                annotations = self.annotation_dict[closest_timestamp]
                self.draw_annotations(frame, annotations)
            
            # 显示帧信息
            info_text = f"Frame: {frame_idx}/{self.frame_count} | Time: {current_time:.2f}s"
            cv2.putText(frame, info_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 显示对象数量
            obj_count = len(annotations) if closest_timestamp else 0
            count_text = f"Objects: {obj_count}"
            cv2.putText(frame, count_text, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 显示帧
            cv2.imshow("JSON Annotation Visualization", frame)
            self.output.write(frame)
            
            # 控制播放速度 (按帧率延迟)
            delay = max(1, int(1000 / self.fps))
            key = cv2.waitKey(delay) & 0xFF
            
            # 控制命令
            if key == ord('q'):  # 退出
                break
            elif key == ord('p'):  # 暂停
                while cv2.waitKey(0) != ord('c'):
                    pass
            elif key == ord('f'):  # 前进一帧
                pass
            elif key == ord('b'):  # 后退一帧
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, frame_idx - 2))
                frame_idx -= 2
            
            frame_idx += 1
        
        # 释放资源
        self.cap.release()
        self.output.release()
        cv2.destroyAllWindows()
        print(f"处理完成! 结果保存为 {self.output_path}")

if __name__ == "__main__":
    # 配置参数
    VIDEO_PATH = sys.argv[1] if len(sys.argv) > 1 else "/home/zero/Code/CarStreamDetection/视觉算法训练场景集-15/测试场景2-五车道路口-晴朗上午.mp4"
    JSON_PATH = sys.argv[2] if len(sys.argv) > 2 else "/home/zero/Code/CarStreamDetection/视觉算法训练场景集-15/测试场景2-五车道路口-晴朗上午.json"
    
    print(f"视频文件: {VIDEO_PATH}")
    print(f"JSON标注: {JSON_PATH}")
    
    # 创建可视化器并运行
    visualizer = JSONAnnotationVisualizer(
        video_path=VIDEO_PATH,
        json_path=JSON_PATH
    )
    visualizer.visualize()