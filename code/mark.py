import os
import cv2
import glob
import argparse
import numpy as np
from tqdm import tqdm

# 默认路径配置
DEFAULT_IMAGE_DIR = '/home/zero/Code/CarStreamDetection/dataset/output/yolo_dataset/train/images'
DEFAULT_LABEL_DIR = '/home/zero/Code/CarStreamDetection/dataset/output/yolo_dataset/train/labels'
DEFAULT_OUTPUT_DIR = '/home/zero/Code/CarStreamDetection/dataset/output/visualized_labels'

def parse_args():
    parser = argparse.ArgumentParser(description='可视化YOLO格式的标注')
    parser.add_argument('--image-dir', type=str, default=DEFAULT_IMAGE_DIR, 
                        help=f'包含图片的目录路径 (默认: {DEFAULT_IMAGE_DIR})')
    parser.add_argument('--label-dir', type=str, default=DEFAULT_LABEL_DIR, 
                        help=f'包含标签文件的目录路径 (默认: {DEFAULT_LABEL_DIR})')
    parser.add_argument('--output-dir', type=str, default=DEFAULT_OUTPUT_DIR, 
                        help=f'输出可视化结果的目录 (默认: {DEFAULT_OUTPUT_DIR})')
    parser.add_argument('--show-conf', action='store_true', 
                        help='在标签上显示置信度（对于真实标签通常不需要）')
    parser.add_argument('--font-scale', type=float, default=0.6,
                        help='标签字体大小 (默认: 0.6)')
    parser.add_argument('--box-thickness', type=int, default=2,
                        help='边界框线条粗细 (默认: 2)')
    parser.add_argument('--auto-create-dirs', action='store_true',
                        help='自动创建缺失的输入/输出目录')
    return parser.parse_args()

def load_class_colors():
    """为您的8种类别定义固定颜色"""
    return {
        "truck": (0, 0, 255),        # 红色
        "bus": (0, 165, 255),         # 橙色
        "mixer": (0, 255, 255),       # 黄色
        "car": (0, 255, 0),           # 绿色
        "tricycle": (255, 0, 0),      # 蓝色
        "motor": (255, 0, 255),       # 紫色
        "bicycle": (180, 105, 255),   # 粉色
        "person": (255, 255, 255)     # 白色
    }

def get_class_name(class_id):
    """根据类别ID获取类别名称"""
    class_mapping = {
        0: "truck",
        1: "bus",
        2: "mixer",
        3: "car",
        4: "tricycle",
        5: "motor",
        6: "bicycle",
        7: "person"
    }
    return class_mapping.get(class_id, f"class_{class_id}")

def denormalize_bbox(bbox, img_width, img_height):
    """将归一化的YOLO边界框转换为像素坐标"""
    class_id, x_center, y_center, width, height = bbox
    
    # 转换为像素坐标
    x_center *= img_width
    y_center *= img_height
    width *= img_width
    height *= img_height
    
    # 计算左上角坐标
    x1 = int(x_center - width / 2)
    y1 = int(y_center - height / 2)
    
    return int(class_id), x1, y1, int(width), int(height)

def visualize_annotations(args):
    # 检查并创建目录
    if args.auto_create_dirs:
        os.makedirs(args.image_dir, exist_ok=True)
        os.makedirs(args.label_dir, exist_ok=True)
        os.makedirs(args.output_dir, exist_ok=True)
    
    # 加载类别颜色映射
    class_colors = load_class_colors()
    
    # 获取所有图片文件
    image_extensions = ('*.jpg', '*.jpeg', '*.png', '*.bmp')
    image_files = []
    for ext in image_extensions:
        image_files.extend(glob.glob(os.path.join(args.image_dir, ext)))
    
    if not image_files:
        print(f"警告: 在 {args.image_dir} 中没有找到图片文件")
        print("支持的格式: .jpg, .jpeg, .png, .bmp")
        return
    
    print(f"找到 {len(image_files)} 张图片")
    
    # 处理每张图片
    processed_count = 0
    for img_path in tqdm(image_files):
        # 读取图片
        img = cv2.imread(img_path)
        if img is None:
            print(f"无法读取图片: {img_path}")
            continue
        
        img_height, img_width = img.shape[:2]
        filename = os.path.basename(img_path)
        base_name = os.path.splitext(filename)[0]
        
        # 构建标签文件路径
        label_path = os.path.join(args.label_dir, f"{base_name}.txt")
        
        # 检查标签文件是否存在
        if not os.path.exists(label_path):
            # print(f"警告: 找不到标签文件 {label_path}")
            # 保存没有标注的原始图片
            output_path = os.path.join(args.output_dir, filename)
            cv2.imwrite(output_path, img)
            continue
        
        # 读取标签文件
        try:
            with open(label_path, 'r') as f:
                lines = f.readlines()
        except Exception as e:
            print(f"读取标签文件错误: {label_path} - {str(e)}")
            continue
        
        # 处理每个边界框
        objects_count = 0
        for line in lines:
            parts = line.strip().split()
            if len(parts) < 5:
                continue
            
            # 解析边界框
            try:
                bbox = [float(x) for x in parts[:5]]
            except ValueError:
                continue
            
            # 转换为像素坐标
            class_id, x1, y1, width, height = denormalize_bbox(bbox, img_width, img_height)
            
            # 确保坐标在图像范围内
            x1 = max(0, min(x1, img_width - 1))
            y1 = max(0, min(y1, img_height - 1))
            width = max(1, min(width, img_width - x1))
            height = max(1, min(height, img_height - y1))
            
            # 计算右下角坐标
            x2 = x1 + width
            y2 = y1 + height
            
            # 获取类别名称
            class_name = get_class_name(class_id)
            
            # 获取颜色
            color = class_colors.get(class_name, (0, 255, 255))  # 默认为黄色
            
            # 绘制边界框
            cv2.rectangle(img, (x1, y1), (x2, y2), color, args.box_thickness)
            
            # 创建标签文本
            label = f"{class_name}"
            
            # 添加置信度（如果可用）
            if args.show_conf and len(parts) > 5:
                conf = float(parts[5])
                label += f" {conf:.2f}"
            
            # 计算文本尺寸
            (text_width, text_height), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, args.font_scale, 1
            )
            
            # 计算文本背景位置
            bg_x1 = x1
            bg_y1 = max(0, y1 - text_height - 5)
            bg_x2 = x1 + text_width
            bg_y2 = y1
            
            # 绘制文本背景
            cv2.rectangle(img, (bg_x1, bg_y1), (bg_x2, bg_y2), color, -1)
            
            # 绘制文本
            cv2.putText(img, label, (x1, y1 - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, args.font_scale, (0, 0, 0), 1)
            
            objects_count += 1
        
        # 添加文件名和大小信息
        info_text = f"{filename} ({img_width}x{img_height})"
        cv2.putText(img, info_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 添加标注数量信息
        count_text = f"Objects: {objects_count}"
        cv2.putText(img, count_text, (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 保存结果
        output_path = os.path.join(args.output_dir, filename)
        cv2.imwrite(output_path, img)
        
        # 显示进度
        print(f"已处理: {filename} (检测到 {objects_count} 个对象)")
        processed_count += 1
    
    print(f"\n处理完成! 共处理 {processed_count}/{len(image_files)} 张图片")
    print(f"输出保存在: {os.path.abspath(args.output_dir)}")

def print_welcome():
    print("\n" + "="*60)
    print("YOLO 标注可视化工具")
    print(f"默认图像目录: {DEFAULT_IMAGE_DIR}")
    print(f"默认标签目录: {DEFAULT_LABEL_DIR}")
    print(f"默认输出目录: {DEFAULT_OUTPUT_DIR}")
    print("="*60 + "\n")

def main():
    print_welcome()
    args = parse_args()
    
    # 打印配置信息
    print("当前配置:")
    print(f"  图像目录: {os.path.abspath(args.image_dir)}")
    print(f"  标签目录: {os.path.abspath(args.label_dir)}")
    print(f"  输出目录: {os.path.abspath(args.output_dir)}")
    print(f"  字体大小: {args.font_scale}")
    print(f"  框线粗细: {args.box_thickness}")
    print(f"  显示置信度: {'是' if args.show_conf else '否'}")
    print(f"  自动创建目录: {'是' if args.auto_create_dirs else '否'}")
    
    print("\n类别颜色映射:")
    class_colors = load_class_colors()
    for class_name, color in class_colors.items():
        color_bgr = f"({color[0]}, {color[1]}, {color[2]})"
        print(f"  {class_name.ljust(10)}: {color_bgr}")
    
    print("\n开始处理...")
    visualize_annotations(args)

if __name__ == "__main__":
    main()