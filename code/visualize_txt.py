import cv2
import numpy as np

# 配置参数
ANNOTATION_FILE = "/home/zero/Code/CarStreamDetection/视觉算法训练场景集-15/测试场景1-五车道路口-晴朗早晨.txt"  # 标注文件路径
VIDEO_FILE = "/home/zero/Code/CarStreamDetection/视觉算法训练场景集-15/测试场景1-五车道路口-晴朗早晨.mp4"       # 输入视频路径
OUTPUT_FILE = "output_video.mp4"     # 输出视频路径
# CLASSES_LIST = "MOTclasses.txt"
FONT = cv2.FONT_HERSHEY_SIMPLEX      # 字体类型
FONT_SCALE = 0.6                     # 字体大小
FONT_THICKNESS = 2                   # 字体粗细

# 为不同类别定义颜色 (BGR格式)
COLORS = {
    "truck": (0,122,122),
    "bus": (122,0,122),
    "mixer": (0,0,0),
    "car": (0, 0, 255),      # 红色
    "tricycle":(255,255,255),
    "motor":(0,0,122),
    "bicycle":(122,0,0),
    "person": (0, 255, 0)   # 绿色
}

# 读取标注数据
def parse_annotations(file_path):
    annotations = {}
    with open(file_path, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            frame_id = int(parts[0])
            class_name = parts[1]
            obj_id = int(parts[2])
            x1 = int(parts[3])
            y1 = int(parts[4])
            w = int(parts[5])
            h = int(parts[6])
            
            if frame_id not in annotations:
                annotations[frame_id] = []
            
            annotations[frame_id].append({
                'class': class_name,
                'id': obj_id,
                'bbox': (x1, y1, w, h)
            })
    return annotations

# 主处理函数
def visualize_annotations():
    # 读取标注数据
    annotations = parse_annotations(ANNOTATION_FILE)
    
    # 打开视频
    cap = cv2.VideoCapture(VIDEO_FILE)
    if not cap.isOpened():
        print("Error: Could not open video file")
        return
    
    # 获取视频属性
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    # 创建视频写入器
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(OUTPUT_FILE, fourcc, fps, (width, height))
    
    frame_idx = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # 绘制当前帧的标注
        if frame_idx in annotations:
            for obj in annotations[frame_idx]:
                class_name = obj['class']
                obj_id = obj['id']
                x1, y1, w, h = obj['bbox']
                x2, y2 = x1 + w, y1 + h
                
                # 获取颜色，如果类别不存在则使用黄色
                color = COLORS.get(class_name, (0, 255, 255))
                
                # 绘制边界框
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                
                # 创建标签文本
                label = f"{class_name}-{obj_id}"
                
                # 计算文本位置（框上方）
                text_size = cv2.getTextSize(label, FONT, FONT_SCALE, FONT_THICKNESS)[0]
                text_x = x1
                text_y = max(20, y1 - 5)
                
                # 绘制文本背景
                cv2.rectangle(frame, 
                             (text_x, text_y - text_size[1]), 
                             (text_x + text_size[0], text_y + 5), 
                             color, -1)
                
                # 绘制文本
                cv2.putText(frame, label, (text_x, text_y), 
                           FONT, FONT_SCALE, (255, 255, 255), FONT_THICKNESS)
        
        # 显示帧号
        cv2.putText(frame, f"Frame: {frame_idx}", (10, 30), 
                   FONT, 0.7, (0, 255, 255), 2)
        
        # 写入输出视频
        out.write(frame)
        
        # 显示处理进度
        print(f"Processing frame: {frame_idx}")
        frame_idx += 1
    
    # 释放资源
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    print(f"Processing completed. Output saved to: {OUTPUT_FILE}")

if __name__ == "__main__":
    visualize_annotations()