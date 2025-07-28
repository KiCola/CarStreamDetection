from ultralytics import YOLO

if __name__ == '__main__':
    # Load a pretrained YOLOv11 model
    model = YOLO("yolo11l")

    # Train the model on a custom dataset
    model.train(
        data='/home/zero/Code/CarStreamDetection/dataset/output/finalDataset/data.yaml',  # 数据集配置文件路径
        epochs=30,  # 训练轮数
        imgsz=640,  # 输入图像大小
        batch=4,  # 批量大小
        workers=1,  # 数据加载线程数
        project='/home/zero/Code/CarStreamDetection/ultralytics/ultralytics',  # 指定输出目录为 yolov11 下的 runs 文件夹
        name='train'  
    )
