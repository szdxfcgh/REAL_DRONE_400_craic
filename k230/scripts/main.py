import os
import sys
import gc
import time
import image

# 导入 CanMV K230 的核心流控和 AI 推理库
from libs.PipeLine import PipeLine, ScopedTiming
from libs.YOLO import YOLOv5  

def load_labels(label_path):
    labels = []
    try:
        with open(label_path, "r") as f:
            for line in f:
                name = line.strip()
                if name:
                    labels.append(name)
    except Exception as exc:
        print("Failed to load labels:", exc)
        return []
    return labels

def main():
    # ------------------- 配置区 -------------------
    rgb888p_size = [1280, 720]
    display_size = [640, 480]
    display_mode = "lcd" 

    # 模型与标签路径 (根据你最新的训练情况，如果你用了k2305则用resnet，否则用mobilenet)
    kmodel_path = "/sdcard/kmodel/resnet18_cifar100.kmodel" 
    labels_path = "/sdcard/kmodel/labels.txt"
    
    model_input_size = [224, 224]
    confidence_threshold = 0.50

    stable_frames = 3   
    unknown_frames = 3  
    # ----------------------------------------------

    labels = load_labels(labels_path)
    if len(labels) == 0:
        print(f"Error: No labels loaded. Abort.")
        raise SystemExit

    # 1. 创建摄像头与屏幕管道
    pl = PipeLine(rgb888p_size=rgb888p_size, display_size=display_size, display_mode=display_mode)
    pl.create()

    # ========================================================
    # 阶段一：识别二维码获取目标类别
    # ========================================================
    target_a = ""
    target_b = ""
    landing_side = "left"
    print("\n[INFO] Phase 1: Please scan the QR Code...")

    while True:
        os.exitpoint() 
        frame = pl.get_frame() # 获取当前帧图像
        
        # 调用 CanMV 自带的二维码识别函数
        qrcodes = frame.find_qrcodes()
        
        pl.osd_img.clear()
        
        if qrcodes:
            qr = qrcodes[0]
            payload = qr.payload()
            print(f"\n[INFO] QR Code Scanned! Raw data: {payload}")
            
            # 解析以逗号分隔的单词
            parts = payload.split(',')
            if len(parts) >= 2:
                target_a = parts[0].strip()
                target_b = parts[1].strip()
                if len(parts) >= 3 and parts[2].strip().lower() in ("left", "right"):
                    landing_side = parts[2].strip().lower()
                print(f"[INFO] Successfully set Target A = '{target_a}', Target B = '{target_b}'")
                print(f"ROS_MSG:qr,{target_a},{target_b},{landing_side}")
                break # 成功获取到两个目标后，跳出二维码扫描循环
            else:
                pl.osd_img.draw_string_advanced(10, 50, 32, "Invalid QR Format!", color=(255, 0, 0))

        # 屏幕上的提示语
        pl.osd_img.draw_string_advanced(10, 10, 32, "Phase 1: Scanning QR Code...", color=(255, 255, 255))
        pl.osd_img.draw_string_advanced(10, 50, 24, "Format: word1,word2,word3", color=(200, 200, 200))
        
        pl.show_image()
        gc.collect()

    # ========================================================
    # 阶段二：过渡与模型加载 (休息 3 秒钟)
    # ========================================================
    pl.osd_img.clear()
    # 在屏幕上提示获取到的目标，并倒计时
    pl.osd_img.draw_string_advanced(10, 10, 40, f"Target A: {target_a}", color=(0, 255, 0))
    pl.osd_img.draw_string_advanced(10, 60, 40, f"Target B: {target_b}", color=(0, 255, 0))
    pl.osd_img.draw_string_advanced(10, 120, 32, "Loading AI Model...", color=(255, 255, 0))
    pl.osd_img.draw_string_advanced(10, 160, 32, "Please wait 3 seconds.", color=(255, 255, 0))
    pl.show_image()
    
    print("\n[INFO] Phase 2: Resting for 3 seconds and loading AI model...")
    time.sleep(3)

    # ========================================================
    # 阶段三：初始化 AI 模型并进行指定目标分类
    # ========================================================
    print("\n[INFO] Phase 3: Initializing KPU and starting classification task...")
    classifier = YOLOv5(
        task_type="classify",
        mode="video",
        kmodel_path=kmodel_path,
        labels=labels,
        rgb888p_size=rgb888p_size,
        model_input_size=model_input_size,
        display_size=display_size,
        conf_thresh=confidence_threshold,
        debug_mode=0,
    )
    classifier.config_preprocess()

    candidate_id = -1
    candidate_hits = 0
    unknown_hits = 0
    stable_id = -1
    stable_score = 0.0
    last_print_name = ""

    print("========================================")
    print(f"Target Mission: Look for [{target_a}] or [{target_b}]")
    print("========================================")

    try:
        while True:
            os.exitpoint() 
            with ScopedTiming("total", 0): 
                frame = pl.get_frame()
                res = classifier.run(frame)
                cur_id, cur_score = res[0], res[1]

                # 防抖逻辑
                if cur_id != -1:
                    unknown_hits = 0
                    if cur_id == candidate_id:
                        candidate_hits += 1
                    else:
                        candidate_id = cur_id
                        candidate_hits = 1

                    if candidate_hits >= stable_frames:
                        stable_id = cur_id
                        stable_score = cur_score
                else:
                    candidate_id = -1
                    candidate_hits = 0
                    unknown_hits += 1
                    if unknown_hits >= unknown_frames:
                        stable_id = -1
                        stable_score = 0.0

                pl.osd_img.clear()
                
                # 顶部绘制当前的任务目标
                pl.osd_img.draw_rectangle(0, 0, display_size[0], 40, color=(30, 30, 30), thickness=1, fill=True)
                pl.osd_img.draw_string_advanced(10, 5, 24, f"Mission: Find [{target_a}] or [{target_b}]", color=(0, 255, 255))

                if stable_id != -1:
                    show_name = labels[stable_id]
                    
                    # 判断当前识别到的物体，是不是我们要找的目标 A 或 B
                    is_target = (show_name == target_a or show_name == target_b)
                    
                    if is_target:
                        ui_color = (0, 255, 0) # 命中目标：绿色
                        pl.osd_img.draw_string_advanced(10, 50, 40, f"SUCCESS: {show_name}", color=ui_color)
                        
                        # ---------------- 核心逻辑 ----------------
                        # 只有是目标类别，且类别发生变化时，才在终端输出特定格式消息 
                        if show_name != last_print_name:
                            print(f"ROS_MSG:target,{show_name},{stable_score:.2f},{display_size[0]//2},{display_size[1]//2}")
                            last_print_name = show_name
                    else:
                        ui_color = (255, 100, 100) # 非目标：红色
                        pl.osd_img.draw_string_advanced(10, 50, 32, f"Ignored: {show_name}", color=ui_color)
                        
                        # 非目标类别不输出，但更新 last_print_name 以便状态切换
                        if show_name != last_print_name:
                            last_print_name = show_name
                else:
                    pl.osd_img.draw_string_advanced(10, 50, 32, "Scanning...", color=(150, 150, 150))
                    # 丢失目标时重置 print 状态，这样下次重新检测到还能打印
                    last_print_name = ""

                pl.show_image()
                gc.collect()

    except Exception as exc:
        sys.print_exception(exc)
    finally:
        classifier.deinit()
        pl.destroy()
        print("\n[INFO] Mission Closed.")

if __name__ == "__main__":
    main()
