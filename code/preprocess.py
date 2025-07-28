import os
import cv2
from tqdm import tqdm

cutset = 5  # 每5帧截取1帧


def convert_to_yolo_format(frame_width, frame_height, x1, y1, w, h):
    """将边界框转换为YOLO格式 (归一化的中心坐标和宽高)"""
    x_center = (x1 + w / 2) / frame_width
    y_center = (y1 + h / 2) / frame_height
    width = w / frame_width
    height = h / frame_height
    return x_center, y_center, width, height

def process_video(video_path, annotation_path, output_dir):
    """处理单个视频并提取帧和标注"""
    # 创建输出目录
    images_dir = os.path.join(output_dir, "images")
    labels_dir = os.path.join(output_dir, "labels")
    os.makedirs(images_dir, exist_ok=True)
    os.makedirs(labels_dir, exist_ok=True)
    
    # 读取标注数据
    annotations = {}
    with open(annotation_path, 'r') as f:
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
            
            annotations[frame_id].append((class_name, x1, y1, w, h))
    
    # 类别映射
    class_mapping = {
        "truck":   0,
        "bus":     1,
        "mixer":   2,
        "car":     3,     
        "tricycle":4,
        "motor":   5,
        "bicycle": 6,
        "person":  7  
    }

    auged_class = {
        "bus":     1,
        "mixer":   2,
        "tricycle":4,
        "bicycle": 6
    }
    
    # 处理视频
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"无法打开视频: {video_path}")
        return
    
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    # 使用tqdm显示进度条
    for frame_idx in tqdm(range(total_frames), desc=f"处理 {os.path.basename(video_path)}"):
        ret, frame = cap.read()
        if not ret:
            break
        
        need = False

        #             # 创建标签文件
        # label_filename = f"{os.path.splitext(os.path.basename(video_path))[0]}_frame{frame_idx:06d}.txt"
        # label_path = os.path.join(labels_dir, label_filename)

        # if frame_idx in annotations:
        #     with open(label_path, 'w') as label_file:
        #         for obj in annotations[frame_idx]:
        #             class_name, x1, y1, w, h = obj
        #             if class_name not in class_mapping:
        #                 continue  # 跳过未定义类别
        #             if class_name in auged_class:
        #                 need = True


        # 只保存有标注的帧
        if (frame_idx%cutset==0 or need==True) and frame_idx in annotations:
            # 保存图像
            img_filename = f"{os.path.splitext(os.path.basename(video_path))[0]}_frame{frame_idx:06d}.jpg"
            img_path = os.path.join(images_dir, img_filename)
            cv2.imwrite(img_path, frame)
            
            # 创建标签文件
            label_filename = f"{os.path.splitext(os.path.basename(video_path))[0]}_frame{frame_idx:06d}.txt"
            label_path = os.path.join(labels_dir, label_filename)
            
            with open(label_path, 'w') as label_file:
                for obj in annotations[frame_idx]:
                    class_name, x1, y1, w, h = obj
                    if class_name not in class_mapping:
                        continue  # 跳过未定义类别
                    
                    class_id = class_mapping[class_name]
                    x_center, y_center, width, height = convert_to_yolo_format(
                        frame_width, frame_height, x1, y1, w, h
                    )
                    
                    label_file.write(f"{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n")
    
    cap.release()

def main():
    # 配置参数
    VIDEO_DIR = "/home/zero/Code/CarStreamDetection/dataset/new"          # 视频目录
    ANNOTATION_DIR = "/home/zero/Code/CarStreamDetection/dataset/annotations" # 标注目录
    OUTPUT_DIR = "/home/zero/Code/CarStreamDetection/dataset/output"                # 输出数据集目录
    
    # 确保输出目录存在
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    # 遍历所有视频文件
    video_files = [f for f in os.listdir(VIDEO_DIR) if f.endswith(('.mp4', '.avi', '.mov'))]
    
    for video_file in video_files:
        video_path = os.path.join(VIDEO_DIR, video_file)
        annotation_file = os.path.join(ANNOTATION_DIR, os.path.splitext(video_file)[0] + ".txt")
        
        if os.path.exists(annotation_file):
            process_video(video_path, annotation_file, OUTPUT_DIR)
        else:
            print(f"警告: 找不到 {video_file} 的标注文件")

if __name__ == "__main__":
    main()