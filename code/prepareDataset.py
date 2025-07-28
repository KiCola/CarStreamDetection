import os
import random
import shutil
from tqdm import tqdm

def create_yolo_dataset(image_dir, label_dir, output_dir, train_ratio=0.8, val_ratio=0.2):
    """
    创建YOLOv8格式的数据集
    :param image_dir: 原始图像目录
    :param label_dir: 原始标签目录
    :param output_dir: 输出数据集目录
    :param train_ratio: 训练集比例
    :param val_ratio: 验证集比例
    """
    # 确保输出目录存在
    os.makedirs(os.path.join(output_dir, "train", "images"), exist_ok=True)
    os.makedirs(os.path.join(output_dir, "train", "labels"), exist_ok=True)
    os.makedirs(os.path.join(output_dir, "val", "images"), exist_ok=True)
    os.makedirs(os.path.join(output_dir, "val", "labels"), exist_ok=True)
    
    # 获取所有图像文件
    image_files = [f for f in os.listdir(image_dir) if f.endswith(('.jpg', '.jpeg', '.png'))]
    print(f"找到 {len(image_files)} 张图像")
    
    # 随机打乱文件列表
    random.shuffle(image_files)
    
    # 计算划分点
    total = len(image_files)
    train_end = int(total * train_ratio)
    val_end = train_end + int(total * val_ratio)
    
    # 创建训练集
    print("创建训练集...")
    for img_file in tqdm(image_files[:train_end]):
        base_name = os.path.splitext(img_file)[0]
        
        # 复制图像
        src_img = os.path.join(image_dir, img_file)
        dst_img = os.path.join(output_dir, "train", "images", img_file)
        shutil.copy(src_img, dst_img)
        
        # 复制标签
        label_file = f"{base_name}.txt"
        src_label = os.path.join(label_dir, label_file)
        dst_label = os.path.join(output_dir, "train", "labels", label_file)
        
        if os.path.exists(src_label):
            shutil.copy(src_label, dst_label)
    
    # 创建验证集
    print("创建验证集...")
    for img_file in tqdm(image_files[train_end:val_end]):
        base_name = os.path.splitext(img_file)[0]
        
        # 复制图像
        src_img = os.path.join(image_dir, img_file)
        dst_img = os.path.join(output_dir, "val", "images", img_file)
        shutil.copy(src_img, dst_img)
        
        # 复制标签
        label_file = f"{base_name}.txt"
        src_label = os.path.join(label_dir, label_file)
        dst_label = os.path.join(output_dir, "val", "labels", label_file)
        
        if os.path.exists(src_label):
            shutil.copy(src_label, dst_label)
    
    print(f"\n数据集准备完成:")
    print(f"  训练集: {train_end} 张图像")
    print(f"  验证集: {val_end - train_end} 张图像")
    print(f"  输出目录: {output_dir}")

def create_data_yaml(output_dir, class_names):
    """
    创建YOLOv8数据集配置文件
    :param output_dir: 数据集目录
    :param class_names: 类别名称列表
    """
    yaml_path = os.path.join(output_dir, "data.yaml")
    
    content = f"""# YOLOv8 数据集配置文件
# 由数据集准备工具生成

# 数据集路径
path: {os.path.abspath(output_dir)}  # 数据集根目录
train: train/images  # 训练图像相对路径
val: val/images      # 验证图像相对路径

# 类别信息
nc: {len(class_names)}  # 类别数量
names: {class_names}  # 类别名称列表
"""
    
    with open(yaml_path, 'w') as f:
        f.write(content)
    
    print(f"\n数据集配置文件已创建: {yaml_path}")
    print("内容预览:")
    print(content)

if __name__ == "__main__":
    # 配置参数
    IMAGE_DIR = "/home/zero/Code/CarStreamDetection/dataset/output/images"        # 原始图像目录
    LABEL_DIR = "/home/zero/Code/CarStreamDetection/dataset/output/labels"        # 原始标签目录
    OUTPUT_DIR = "/home/zero/Code/CarStreamDetection/dataset/output/yolo_dataset" # 输出数据集目录
    CLASS_NAMES = ["truck", "bus", "mixer", "car", "tricycle", "motor", "bicycle", "person"]
    
    # 创建数据集目录结构
    # create_yolo_dataset(IMAGE_DIR, LABEL_DIR, OUTPUT_DIR)
    
    # 创建数据集配置文件
    # create_data_yaml(OUTPUT_DIR, CLASS_NAMES)
    
    print("\n数据集准备完成，您现在可以使用以下命令训练YOLOv8模型:")
    print(f"yolo train data={os.path.join(OUTPUT_DIR, 'data.yaml')} model=yolov8n.pt epochs=100 imgsz=640")