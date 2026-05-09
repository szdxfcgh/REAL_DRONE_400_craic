import os
import sys
import gc
import time
import image
from media.sensor import CAM_CHN_ID_1

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

def draw_reticle(img, cx, cy, size, color, thickness=2, length=20):
    """
    在画面中心绘制一个具有科技感的瞄准十字/边角框
    用来引导用户将物体放在画面中央
    """
    half = size // 2
    # 左上角
    img.draw_line(cx - half, cy - half, cx - half + length, cy - half, color, thickness)
    img.draw_line(cx - half, cy - half, cx - half, cy - half + length, color, thickness)
    # 右上角
    img.draw_line(cx + half, cy - half, cx + half - length, cy - half, color, thickness)
    img.draw_line(cx + half, cy - half, cx + half, cy - half + length, color, thickness)
    # 左下角
    img.draw_line(cx - half, cy + half, cx - half + length, cy + half, color, thickness)
    img.draw_line(cx - half, cy + half, cx - half, cy + half - length, color, thickness)
    # 右下角
    img.draw_line(cx + half, cy + half, cx + half - length, cy + half, color, thickness)
    img.draw_line(cx + half, cy + half, cx + half, cy + half - length, color, thickness)

    # 中心小十字
    img.draw_line(cx - 10, cy, cx + 10, cy, color, 1)
    img.draw_line(cx, cy - 10, cx, cy + 10, color, 1)

def main():
    # ------------------- 配置区 -------------------
    rgb888p_size = [1280, 720]
    display_size = [640, 480]
    display_mode = "lcd"

    kmodel_path = "/sdcard/kmodel/resnet50_cifar100.kmodel"
    labels_path = "/sdcard/kmodel/labels_resnet50.txt"

    model_input_size = [224, 224]
    confidence_threshold = 0.50
    stable_frames = 3
    unknown_frames = 3
    # ----------------------------------------------

    labels = load_labels(labels_path)
    if len(labels) == 0:
        print("Error: No labels loaded. Abort.")
        raise SystemExit

    pl = PipeLine(rgb888p_size=rgb888p_size, display_size=display_size, display_mode=display_mode)
    pl.create()

    # ========================================================
    # 阶段一：识别二维码获取目标类别
    # ========================================================
    target_a = ""
    target_b = ""
    land_location = ""
    print("\n[INFO] Phase 1: Please scan the QR Code...")

    while True:
        os.exitpoint()
        img = pl.sensor.snapshot(chn=CAM_CHN_ID_1)
        qrcodes = img.find_qrcodes()

        pl.osd_img.clear()
        
        if qrcodes:
            payload = qrcodes[0].payload()
            print(f"\n[INFO] QR Code Scanned! Raw data: {payload}")
            parts = payload.split(",")
            if len(parts) >= 3:
                target_a = parts[0].strip()
                target_b = parts[1].strip()
                temp_loc = parts[2].strip().lower()
                if temp_loc in ["left", "right"]:
                    land_location = temp_loc
                    print(f"[INFO] Successfully set Target A='{target_a}', Target B='{target_b}', Location='{land_location}'")
                    print(f"ROS_MSG:qr,{target_a},{target_b},{land_location}")
                    break
                else:
                    pl.osd_img.draw_string_advanced(10, 50, 32, "Loc must be left or right!", color=(255, 0, 0))
            else:
                pl.osd_img.draw_string_advanced(10, 50, 32, "Invalid QR Format!", color=(255, 0, 0))

        pl.osd_img.draw_string_advanced(10, 10, 32, "Phase 1: Scanning QR Code...", color=(255, 255, 255))
        pl.osd_img.draw_string_advanced(10, 50, 24, "Format: word1,word2,left/right", color=(200, 200, 200))
        
        pl.show_image()
        gc.collect()

    # ========================================================
    # 阶段二：过渡与模型加载 (休息 3 秒钟)
    # ========================================================
    pl.osd_img.clear()
    pl.osd_img.draw_string_advanced(10, 10, 40, f"Target A: {target_a}", color=(0, 255, 0))
    pl.osd_img.draw_string_advanced(10, 60, 40, f"Target B: {target_b}", color=(0, 255, 0))
    pl.osd_img.draw_string_advanced(10, 110, 40, f"Location: {land_location}", color=(0, 255, 0))
    pl.osd_img.draw_string_advanced(10, 160, 32, "Loading AI Model...", color=(255, 255, 0))
    pl.osd_img.draw_string_advanced(10, 200, 32, "Please wait 3 seconds.", color=(255, 255, 0))
    pl.show_image()
    
    print("\n[INFO] Phase 2: Resting for 3 seconds and loading AI model...")
    time.sleep(3)

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
    
    target_lock_start_time = 0

    print("\n[INFO] Phase 3: Initializing KPU and starting classification task...")
    print("========================================")
    print(f"Target Mission: Look for [{target_a}] or [{target_b}]")
    print("========================================")

    fps_time = time.ticks_ms()

    try:
        while True:
            os.exitpoint()
            with ScopedTiming("total", 0):
                frame = pl.get_frame()
                res = classifier.run(frame)
                cur_id, cur_score = res[0], res[1]

                # 防抖
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

                # ---------------- FPS 计算 ----------------
                cur_time = time.ticks_ms()
                fps = 1000.0 / (cur_time - fps_time) if cur_time > fps_time else 30.0
                fps_time = cur_time

                pl.osd_img.clear()
                
                # 1. 绘制科技感中心瞄准框
                draw_reticle(pl.osd_img, display_size[0]//2, display_size[1]//2, size=240, color=(150, 200, 255), thickness=2)

                # 2. 顶部绘制当前的任务目标 (深色背景板)
                pl.osd_img.draw_rectangle(0, 0, display_size[0], 40, color=(30, 30, 30), thickness=1, fill=True)
                pl.osd_img.draw_string_advanced(10, 5, 24, f"Mission: [{target_a}] | [{target_b}]", color=(0, 255, 255))

                if stable_id != -1:
                    show_name = labels[stable_id]
                    
                    # 判断当前识别到的物体，是不是我们要找的目标 A 或 B
                    is_target = (show_name == target_a or show_name == target_b)
                    
                    if is_target:
                        ui_color = (0, 255, 0) # 命中目标：亮绿色
                        status_text = f"LOCKED: {show_name.upper()}"
                        
                        # ---------------- 核心逻辑 ----------------
                        if show_name != last_print_name:
                            print(f"ROS_MSG:target,{show_name},{stable_score:.2f},{display_size[0]//2},{display_size[1]//2}")
                            last_print_name = show_name
                            
                            # 重置计时器
                            target_lock_start_time = time.ticks_ms()
                        else:
                            # 目标名称没有发生变化，即持续锁定该目标
                            if target_lock_start_time > 0:
                                current_time = time.ticks_ms()
                                # 检查距离上一次发送是否已经超过 8000 毫秒 (8秒)
                                if time.ticks_diff(current_time, target_lock_start_time) >= 8000:
                                    print(f"ROS_MSG:target,{show_name},{stable_score:.2f},{display_size[0]//2},{display_size[1]//2}")
                                    # 再次重置计时器，让它能继续循环下一个 8 秒
                                    target_lock_start_time = current_time

                    else:
                        ui_color = (255, 100, 100) # 非目标：警示红色
                        status_text = f"IGNORED: {show_name.upper()}"
                        
                        if show_name != last_print_name:
                            last_print_name = show_name
                            
                        # 如果识别到了非目标物体，重置计时器
                        target_lock_start_time = 0

                    # 3. 屏幕中央下方显示识别状态
                    pl.osd_img.draw_string_advanced(display_size[0]//2 - 120, display_size[1] - 80, 32, status_text, color=ui_color)

                    # 4. 绘制置信度条 (右上角)
                    bar_x = display_size[0] - 220
                    bar_y = 12
                    bar_w_max = 140
                    bar_h = 14
                    current_bar_w = int(stable_score * bar_w_max)

                    # 进度条底框与填充
                    pl.osd_img.draw_rectangle(bar_x, bar_y, bar_w_max, bar_h, color=(80, 80, 80), thickness=1, fill=True)
                    pl.osd_img.draw_rectangle(bar_x, bar_y, current_bar_w, bar_h, color=ui_color, thickness=1, fill=True)
                    # 进度条数值
                    pl.osd_img.draw_string_advanced(bar_x + bar_w_max + 10, bar_y - 4, 20, f"{stable_score*100:.0f}%", color=(255, 255, 255))

                else:
                    # 扫描中状态 (呼吸灯效)
                    pl.osd_img.draw_string_advanced(display_size[0]//2 - 60, display_size[1] - 80, 32, "Scanning", color=(180, 180, 180))
                    glow_w = int((time.ticks_ms() % 1000) / 1000.0 * 3) # 生成 0-2 的点
                    dots = "." * glow_w
                    pl.osd_img.draw_string_advanced(display_size[0]//2 + 65, display_size[1] - 80, 32, dots, color=(180, 180, 180))

                    last_print_name = ""
                    target_lock_start_time = 0

                # 5. 左下角绘制实时 FPS
                pl.osd_img.draw_string_advanced(10, display_size[1] - 30, 24, f"FPS: {fps:.1f}", color=(0, 255, 255))

                pl.show_image()
                gc.collect()

    except Exception as exc:
        sys.print_exception(exc)
    finally:
        classifier.deinit()
        pl.destroy()
        print("[INFO] Mission Closed.")


if __name__ == "__main__":
    main()
