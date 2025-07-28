import cv2
import numpy as np
import math
import csv
import pandas as pd
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
    """通过向量叉乘得到的符号征服来辨别车辆行驶方向"""
    x1,y1 = v1
    x2,y2 = v2
    crossdot = x1*y2 - x2*y1

    return crossdot


class DirectionTracker:
    """跟踪目标并判断其运动方向（左转、右转、直行）"""
    
    def __init__(self, direction_threshold=60, min_points=20, alpha = 0.5):
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
        self.classes = {}
        self.counted_ids = set()
    
    def update(self, track_id, center, cls):
        """更新轨迹点并计算方向"""
        # 添加新的中心点
        # self.tracks[track_id].append(center)

        if track_id not in self.classes:
            self.classes[track_id] = cls
        
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
            self._speedArrow_direction(track_id, cls)
    
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

    def _speedArrow_direction(self, track_id, cls):
        points = self.tracks[track_id]
        
        x1,y1 = points[9]
        x0,y0 = points[0]
        x3,y3 = points[-1]
        x2,y2 = points[-9]

        startArrow = (int(x1-x0),int(y1-y0))
        endArrow = (int(x3-x2),int(y3-y2))

        if cls == 7:
            length = len(points)
            print(points[-10:-1])

            in_list = points[0:9]
            mid_list = points[int(length/2)-4:int(length/2)+5]
            out_list = points[-10:-1]
            x0,y0 = avg_list(in_list)
            xm,ym = avg_list(mid_list)
            xe,ye = avg_list(out_list)

            
            print(f'length:{length}')

            startArrow = (int(xm-x0),int(ym-y0))
            endArrow = (int(xe-xm),int(ye-ym))   # 行人因为是小物体，会相互遮挡产生识别上的坐标抖动。特取整条路径的起点、中点、终点做两向量判断方向

            ## x0,y0,xe,ye,xm,ym = 0


        cross_threshold = 200
        # print(f'cls:{cls}')
        if cls == 1 or cls == 0: #truck & bus
            cross_threshold = 50
        elif cls==7:
            cross_threshold = 15
        # theta = angle_between_vectors(startArrow, endArrow)
        # if theta > 30 
        cross = crossdot(startArrow,endArrow)
        if cross > cross_threshold:
            self.directions[track_id] = "right"
        elif cross <-cross_threshold:
            self.directions[track_id] = "left"
        else:
            self.directions[track_id] = "straight"

        if cls != 7:
            if -100<y3-y0<100 or -100<x3-x0<100:
                self.directions[track_id] = "straight"    # 针对 “仅仅更换了车道线、而初末向量采样偏差” 造成【误判为左转或者右转】的现象做的约束
        else:
            # if -20<ye-(ym+y0)/2 <20 or -20<xe-(xm+x0)/2 <20:
            #     self.directions[track_id] = "straight"  # 针对行人采用更灵敏的方向辨别
            None

        if -30<y3-y0<30 and -40<x3-x0<40:
            self.directions[track_id] = "unknown"
    
    def get_direction(self, track_id):
        """获取目标的当前方向"""
        return self.directions.get(track_id, "unknown")
    
    def get_tracks(self, track_id):
        return self.tracks[track_id]
    
    def mark_obj(self,track_id):
        self.counted_ids.add(track_id)

    def iscounted(self,track_id):
        return track_id in self.counted_ids

def avg_list(lst):
    avg = [sum(pair)/len(pair) for pair in zip(*lst)]
    return avg        

def main():
    # 配置参数
    MODEL_PATH = "/home/zero/Code/CarStreamDetection/ultralytics/ultralytics/runs/finalversion_yolov11l_30e/weights/best.pt"  # 训练好的YOLO模型路径
    # /home/zero/Code/CarStreamDetection/ultralytics/ultralytics/train2/weights/best.pt
    # MODEL_PATH = "/home/zero/Code/PRML/yolov12/runs/detect/train13/weights/best.pt"  # 训练好的YOLO模型路径
    VIDEO_SOURCE = "/home/zero/Code/CarStreamDetection/dataset/train_video/007.mp4"  # 视频文件路径或摄像头ID
    TRACKER_CONFIG = "bytetrack.yaml"  # 跟踪器配置文件
    # TRACKER_CONFIG = "botsort.yaml"  # 跟踪器配置文件
    SHOW_TRACKS = True  # 是否显示轨迹
    OUTPUT_VIDEO = "output_tracking_never.mp4"  # 输出视频路径
    
    model = YOLO(MODEL_PATH)
    direction_tracker = DirectionTracker()
    
    cap = cv2.VideoCapture(VIDEO_SOURCE)
    if not cap.isOpened():
        print(f"错误: 无法打开视频源 {VIDEO_SOURCE}")
        return
    
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(OUTPUT_VIDEO, fourcc, fps, (frame_width, frame_height))
    
    direction_colors = {
        "left": (0, 0, 255),    # 红色 - 左转
        "right": (0, 255, 0),    # 绿色 - 右转
        "straight": (255, 0, 0), # 蓝色 - 直行
        "unknown": (200, 200, 200)  # 灰色 - 未知
    }
    
    class_direction_stats = defaultdict(lambda: 
    {
        "total": 0,
        "left": 0,
        "right": 0,
        "straight": 0,
        "unknown": 0
    })

    frame_count = 0
    prev_track_ids = set()

    while True:
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
        clses = results[0].boxes.cls.int().cpu().tolist()
        track_ids = results[0].boxes.id.int().cpu().tolist() if results[0].boxes.id is not None else []
        
        current_track_ids = set(track_ids)

        disappeared_ids = prev_track_ids - current_track_ids
        obj_list = []
        for track_id in disappeared_ids:
            tracks = direction_tracker.get_tracks(track_id)
            if abs(tracks[-1][0]-tracks[0][0]) < 30 and abs(tracks[-1][1]-tracks[0][1]):        #去抖动用
                continue
            if not direction_tracker.iscounted(track_id):
                direction = direction_tracker.get_direction(track_id)
                cls_id = direction_tracker.classes.get(track_id, -1)
                if cls_id != -1:
                    class_direction_stats[cls_id][direction] += 1
                    class_direction_stats[cls_id]["total"] += 1
                    direction_tracker.mark_obj(track_id)

        prev_track_ids = current_track_ids


        annotator = Annotator(frame, line_width=2)
        
        # 处理每个检测到的目标
        for box, cls_id, track_id in zip(boxes, clses, track_ids):
            x, y, w, h = box
            center = (int(x), int(y))
            
            # 更新方向跟踪器
            direction_tracker.update(track_id, center, cls_id)
            
            # 获取当前方向
            direction = direction_tracker.get_direction(track_id)
            color = direction_colors[direction]
            
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
        
        cv2.imshow("Traffic Direction Tracking", frame)
        out.write(frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):        # 按 'q' 退出
            break
        
        frame_count += 1

    # for track_id in prev_track_ids:
    #     if not direction_tracker.iscounted(track_id):
    #         direction = direction_tracker.get_direction(track_id)
    #         cls_id = direction_tracker.classes.get(track_id, -1)
            
    #     if cls_id != -1:
    #         class_direction_stats[cls_id][direction] += 1
    #         class_direction_stats[cls_id]["total"] += 1
    
    cap.release()
    out.release()
    cv2.destroyAllWindows()

    csv_list = []

    print("\n最终统计结果:")
    print(f"{'类别':<12} {'总数':<3} {'左转':<3} {'右转':<3} {'直行':<3} {'未知':<3}")
    for cls_id, stats in sorted(class_direction_stats.items()):
        class_name = model.names.get(cls_id, f"Class{cls_id}")
        dynamic_obj = stats['left']+stats['right']+stats['straight']
        csv_list.append([class_name,dynamic_obj,stats['left'],stats['right'],stats['straight'], stats['unknown']])
        print(f"{class_name:<15} {dynamic_obj:<6} {stats['left']:<6} {stats['right']:<6} {stats['straight']:<6} {stats['unknown']:<6}")
    print(f"跟踪完成! 结果保存至: {OUTPUT_VIDEO}")

    column=['类别','总数','左转','右转','直行','未知']
    csv = pd.DataFrame(columns=column, data=csv_list)
    csv.to_csv('output.csv')


if __name__ == "__main__":
    main()