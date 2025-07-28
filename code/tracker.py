import cv2
import numpy as np
import math
from collections import defaultdict
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors

def angle_between_vectors(v1, v2):
    dot_product = np.dot(v1, v2)
    v1_norm = np.linalg.norm(v1)
    v2_norm = np.linalg.norm(v2)
    cos_theta = dot_product / (v1_norm * v2_norm)
    theta = math.acos(cos_theta)
    return np.degrees(theta)

def crossdot(v1,v2):
    x1,y1 = v1
    x2,y2 = v2
    crossdot = x1*y2 - x2*y1

    return crossdot


class DirectionTracker:
    """跟踪目标并判断其运动方向（左转、右转、直行）"""
    
    def __init__(self, direction_threshold=60, min_points=16, alpha = 0.9):
        """
        :param direction_threshold: 方向判断阈值（像素）
        :param min_points: 判断方向所需的最小轨迹点数
        :param alpha 惯性滤波系数
        """
        self.tracks = defaultdict(list)  # 存储每个ID的轨迹点
        self.directions = {}  # 存储每个ID的当前方向
        self.direction_threshold = direction_threshold
        self.min_points = min_points
        self.alpha = alpha
        self.prev_ct = {}
        self.filnum = 6
    
    def update(self, track_id, center):
        """更新轨迹点并计算方向"""
        # 添加新的中心点
        # self.tracks[track_id].append(center)

        
        # 惯性滤波
        if track_id in self.prev_ct:
            center = (
                int(self.alpha * center[0] + (1 - self.alpha) * self.prev_ct[track_id][0]),
                int(self.alpha * center[1] + (1 - self.alpha) * self.prev_ct[track_id][1])
            )
        self.prev_ct[track_id] = center
        self.tracks[track_id].append(center)

        # 只保留最近的点（避免轨迹过长）
        # if len(self.tracks[track_id]) > 0:
        #     self.tracks[track_id] = self.tracks[track_id][-20:]
        
        # 当有足够点时才计算方向
        if len(self.tracks[track_id]) >= self.min_points:
            # self._calculate_direction(track_id)
            self._speedArrow_direction(track_id)
    
    def _calculate_direction(self, track_id):
        """计算目标运动方向"""
        points = self.tracks[track_id]
        start_point = points[0]
        end_point = points[-1]

        
        # 计算水平位移
        dx = end_point[0] - start_point[0]
        dy = end_point[1] - start_point[1]
        
        # 判断方向
        if dx < -self.direction_threshold and dy < -self.direction_threshold:  # 向左移动
            self.directions[track_id] = "left"
        elif dx > self.direction_threshold and dy > self.direction_threshold:  # 向右移动
            self.directions[track_id] = "right"
        elif dx < -self.direction_threshold and dy > self.direction_threshold:
            self.directions[track_id] = "right"
        elif dx > self.direction_threshold and dy < -self.direction_threshold:
            self.directions[track_id] = "left"
        else:  # 垂直移动或位移不足
            self.directions[track_id] = "straight"

    def _speedArrow_direction(self, track_id):
        points = self.tracks[track_id]
        # length = len(points)
        # x0,y0 = points[0]
        # xe,ye = points[-1]
        # xm,ym = points[int(length*0.5)]

        # startArrow = (int(xm-x0),int(ym-y0))
        # endArrow = (int(xe-xm),int(ye-ym))

        x1,y1 = points[9]
        x0,y0 = points[0]
        x3,y3 = points[-1]
        x2,y2 = points[-9]

        startArrow = (int(x1-x0),int(y1-y0))
        endArrow = (int(x3-x2),int(y3-y2))



        # theta = angle_between_vectors(startArrow, endArrow)
        # if theta > 30 
        cross = crossdot(startArrow,endArrow)
        if cross > 150:
            self.directions[track_id] = "right"
        elif cross <-150:
            self.directions[track_id] = "left"
        else:
            self.directions[track_id] = "straight"

        if -30<y3-y0<30 or -30<x3-x0<30:
            self.directions[track_id] = "straight"

        if -30<y3-y0<30 and -40<x3-x0<40:
            self.directions[track_id] = "unknown"
    
    def get_direction(self, track_id):
        """获取目标的当前方向"""
        return self.directions.get(track_id, "unknown")

def main():
    # 配置参数
    MODEL_PATH = "/home/zero/Code/PRML/yolov12/runs/detect/train13/weights/best.pt"  # 训练好的YOLO模型路径
    VIDEO_SOURCE = "/home/zero/Code/CarStreamDetection/dataset/video/016.mp4"  # 视频文件路径或摄像头ID
    TRACKER_CONFIG = "bytetrack.yaml"  # 跟踪器配置文件
    SHOW_TRACKS = True  # 是否显示轨迹
    OUTPUT_VIDEO = "output_tracking4.mp4"  # 输出视频路径
    
    # 初始化模型
    model = YOLO(MODEL_PATH)
    
    # 初始化方向跟踪器
    direction_tracker = DirectionTracker()
    
    # 打开视频源
    cap = cv2.VideoCapture(VIDEO_SOURCE)
    if not cap.isOpened():
        print(f"错误: 无法打开视频源 {VIDEO_SOURCE}")
        return
    
    # 获取视频属性
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    # 创建视频写入器
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(OUTPUT_VIDEO, fourcc, fps, (frame_width, frame_height))
    
    # 方向颜色映射
    direction_colors = {
        "left": (0, 0, 255),    # 红色 - 左转
        "right": (0, 255, 0),    # 绿色 - 右转
        "straight": (255, 0, 0), # 蓝色 - 直行
        "unknown": (200, 200, 200)  # 灰色 - 未知
    }
    
    frame_count = 0
    object_list = []
    while True:
        # 读取帧
        ret, frame = cap.read()
        if not ret:
            break
        
        # 使用YOLO进行目标跟踪
        results = model.track(
            frame, 
            persist=True, 
            tracker=TRACKER_CONFIG,
            classes=[0, 1, 2, 3, 4, 5, 6, 7]  # 指定要跟踪的类别ID
        )
        
        # 获取跟踪结果
        boxes = results[0].boxes.xywh.cpu()
        track_ids = results[0].boxes.id.int().cpu().tolist() if results[0].boxes.id is not None else []
        
        # 创建注释器
        annotator = Annotator(frame, line_width=2)
        
        # 处理每个检测到的目标
        for box, track_id in zip(boxes, track_ids):
            x, y, w, h = box
            center = (int(x), int(y))
            
            # 更新方向跟踪器
            direction_tracker.update(track_id, center)
            
            # 获取当前方向
            direction = direction_tracker.get_direction(track_id)
            color = direction_colors[direction]
            
            # 绘制边界框和方向
            label = f"ID:{track_id} {direction}"
            annotator.box_label([x - w/2, y - h/2, x + w/2, y + h/2], label, color)
            
            # 绘制中心点
            # cv2.circle(frame, center, 5, color, -1)prev_ct[track_id]
            cv2.circle(frame, direction_tracker.prev_ct[track_id], 5, color, -1)

            # 绘制轨迹（如果需要）
            if SHOW_TRACKS and track_id in direction_tracker.tracks:
                points = direction_tracker.tracks[track_id]
                for i in range(1, len(points)):
                    cv2.line(frame, points[i-1], points[i], color, 2)
        
        # 添加帧计数
        cv2.putText(frame, f"Frame: {frame_count}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # # 添加方向图例
        # legend_y = 60
        # for direction, color in direction_colors.items():
        #     cv2.putText(frame, f"{direction}", (frame_width - 150, legend_y), 
        #                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        #     cv2.rectangle(frame, (frame_width - 180, legend_y - 20), 
        #                  (frame_width - 160, legend_y - 5), color, -1)
        #     legend_y += 30
        
        # 显示和保存结果
        cv2.imshow("Traffic Direction Tracking", frame)
        out.write(frame)
        
        # 按 'q' 退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        frame_count += 1
    
    # 释放资源
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    print(f"跟踪完成! 结果保存至: {OUTPUT_VIDEO}")

if __name__ == "__main__":
    main()