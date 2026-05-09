import gc
import os
import sys
import time

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
    half = size // 2
    img.draw_line(cx - half, cy - half, cx - half + length, cy - half, color, thickness)
    img.draw_line(cx - half, cy - half, cx - half, cy - half + length, color, thickness)
    img.draw_line(cx + half, cy - half, cx + half - length, cy - half, color, thickness)
    img.draw_line(cx + half, cy - half, cx + half, cy - half + length, color, thickness)
    img.draw_line(cx - half, cy + half, cx - half + length, cy + half, color, thickness)
    img.draw_line(cx - half, cy + half, cx - half, cy + half - length, color, thickness)
    img.draw_line(cx + half, cy + half, cx + half - length, cy + half, color, thickness)
    img.draw_line(cx + half, cy + half, cx + half, cy + half - length, color, thickness)
    img.draw_line(cx - 10, cy, cx + 10, cy, color, 1)
    img.draw_line(cx, cy - 10, cx, cy + 10, color, 1)


def main():
    rgb888p_size = [1280, 720]
    display_size = [640, 480]
    display_mode = "lcd"

    kmodel_path = "/sdcard/kmodel/resnet50_cifar100.kmodel"
    labels_path = "/sdcard/kmodel/labels_resnet50.txt"

    model_input_size = [224, 224]
    confidence_threshold = 0.50
    stable_frames = 3
    unknown_frames = 3
    target_repeat_ms = 1000

    labels = load_labels(labels_path)
    if len(labels) == 0:
        print("Error: No labels loaded. Abort.")
        raise SystemExit

    pl = PipeLine(rgb888p_size=rgb888p_size, display_size=display_size, display_mode=display_mode)
    pl.create()

    target_a = ""
    target_b = ""
    landing_side = ""
    print("[INFO] Phase 1: scan QR Code.")

    while True:
        os.exitpoint()
        img = pl.sensor.snapshot(chn=CAM_CHN_ID_1)
        qrcodes = img.find_qrcodes()

        pl.osd_img.clear()

        if qrcodes:
            payload = qrcodes[0].payload()
            parts = payload.split(",")
            if len(parts) >= 3:
                target_a = parts[0].strip().lower()
                target_b = parts[1].strip().lower()
                landing_side = parts[2].strip().lower()
                if landing_side in ("left", "right") and target_a and target_b:
                    print("ROS_MSG:qr,%s,%s,%s" % (target_a, target_b, landing_side))
                    break
                pl.osd_img.draw_string_advanced(10, 50, 32, "QR side must be left/right", color=(255, 0, 0))
            else:
                pl.osd_img.draw_string_advanced(10, 50, 32, "QR format: a,b,left/right", color=(255, 0, 0))

        pl.osd_img.draw_string_advanced(10, 10, 32, "Scan QR Code", color=(255, 255, 255))
        pl.osd_img.draw_string_advanced(10, 50, 24, "Format: class1,class2,left/right", color=(200, 200, 200))
        pl.show_image()
        gc.collect()

    pl.osd_img.clear()
    pl.osd_img.draw_string_advanced(10, 10, 40, "Target A: %s" % target_a, color=(0, 255, 0))
    pl.osd_img.draw_string_advanced(10, 60, 40, "Target B: %s" % target_b, color=(0, 255, 0))
    pl.osd_img.draw_string_advanced(10, 110, 40, "Landing: %s" % landing_side, color=(0, 255, 0))
    pl.osd_img.draw_string_advanced(10, 170, 32, "Loading model...", color=(255, 255, 0))
    pl.show_image()
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
    last_target_name = ""
    last_target_time = 0
    fps_time = time.ticks_ms()
    center_x = display_size[0] // 2
    center_y = display_size[1] // 2

    print("[INFO] Phase 2: classify targets.")

    try:
        while True:
            os.exitpoint()
            with ScopedTiming("total", 0):
                frame = pl.get_frame()
                res = classifier.run(frame)
                cur_id, cur_score = res[0], res[1]

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

                cur_time = time.ticks_ms()
                fps = 1000.0 / (cur_time - fps_time) if cur_time > fps_time else 30.0
                fps_time = cur_time

                pl.osd_img.clear()
                draw_reticle(pl.osd_img, center_x, center_y, size=240, color=(150, 200, 255), thickness=2)
                pl.osd_img.draw_rectangle(0, 0, display_size[0], 40, color=(30, 30, 30), thickness=1, fill=True)
                pl.osd_img.draw_string_advanced(
                    10,
                    5,
                    24,
                    "Mission: %s | %s" % (target_a, target_b),
                    color=(0, 255, 255),
                )

                if stable_id != -1:
                    show_name = labels[stable_id].strip().lower()
                    is_target = show_name == target_a or show_name == target_b
                    ui_color = (0, 255, 0) if is_target else (255, 100, 100)
                    status = "LOCKED: %s" % show_name.upper() if is_target else "IGNORED: %s" % show_name.upper()

                    if is_target:
                        repeat_due = time.ticks_diff(cur_time, last_target_time) >= target_repeat_ms
                        if show_name != last_target_name or repeat_due:
                            print("ROS_MSG:target,%s,%.2f,%d,%d" % (show_name, stable_score, center_x, center_y))
                            last_target_name = show_name
                            last_target_time = cur_time
                    else:
                        last_target_name = ""
                        last_target_time = 0

                    pl.osd_img.draw_string_advanced(center_x - 120, display_size[1] - 80, 32, status, color=ui_color)
                    bar_x = display_size[0] - 220
                    bar_y = 12
                    bar_w_max = 140
                    bar_h = 14
                    current_bar_w = int(stable_score * bar_w_max)
                    pl.osd_img.draw_rectangle(bar_x, bar_y, bar_w_max, bar_h, color=(80, 80, 80), thickness=1, fill=True)
                    pl.osd_img.draw_rectangle(bar_x, bar_y, current_bar_w, bar_h, color=ui_color, thickness=1, fill=True)
                    pl.osd_img.draw_string_advanced(
                        bar_x + bar_w_max + 10,
                        bar_y - 4,
                        20,
                        "%.0f%%" % (stable_score * 100.0),
                        color=(255, 255, 255),
                    )
                else:
                    pl.osd_img.draw_string_advanced(center_x - 60, display_size[1] - 80, 32, "Scanning", color=(180, 180, 180))
                    last_target_name = ""
                    last_target_time = 0

                pl.osd_img.draw_string_advanced(10, display_size[1] - 30, 24, "FPS: %.1f" % fps, color=(0, 255, 255))
                pl.show_image()
                gc.collect()

    except Exception as exc:
        sys.print_exception(exc)
    finally:
        classifier.deinit()
        pl.destroy()
        print("[INFO] Mission closed.")


if __name__ == "__main__":
    main()
