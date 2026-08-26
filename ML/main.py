from maix import camera, display, image, app, uart, pinmap, touchscreen

report_on = True

# --- UART1 SETUP ---
pinmap.set_pin_function("A19", "UART1_TX")
pinmap.set_pin_function("A18", "UART1_RX")
serial = uart.UART("/dev/ttyS1", 115200)
serial.write(b"Start\n")

# Class IDs for protocol (0: Red, 1: Green)
RED_CLASS_ID = 0
GREEN_CLASS_ID = 1

# Base LAB thresholds
BASE_RED_THRESHOLD = [0, 100, 30, 127, 10, 127]

# Default sensitivity configuration
DEFAULT_SENSITIVITY = 50
sensitivity = DEFAULT_SENSITIVITY

# Minimum blob dimensions
MIN_BLOB_W = 15
MIN_BLOB_H = 15

# Camera, Display, Touchscreen initialization
cam = camera.Camera(224, 224, image.Format.FMT_RGB888)
dis = display.Display()
ts = touchscreen.TouchScreen()


def clamp(value, low, high):
    return max(low, min(high, value))


def get_green_threshold(sens):
    l_max = 35 + sens * 20 // 100
    a_max = -30 + sens * 35 // 100
    b_min = -5 - sens * 30 // 100
    b_max = 35 + sens * 30 // 100
    return [[0, l_max, -128, a_max, b_min, b_max]]


def update_sensitivity_from_touch(img, sens):
    x, y, pressed = ts.read()
    if not pressed:
        return sens

    x, y = image.resize_map_pos_reverse(
        img.width(),
        img.height(),
        dis.width(),
        dis.height(),
        image.Fit.FIT_CONTAIN,
        x,
        y,
    )

    slider_x = 12
    slider_y = img.height() - 54
    slider_w = img.width() - 24
    if y >= slider_y:
        return clamp((x - slider_x) * 100 // slider_w, 0, 100)
    return sens


def draw_tuning_ui(img, sens):
    img_w = img.width()
    img_h = img.height()
    slider_x = 12
    slider_y = img_h - 18
    slider_w = img_w - 24
    knob_x = slider_x + sens * slider_w // 100

    img.draw_rect(0, img_h - 40, img_w, 40, color=image.Color.from_rgb(0, 0, 0), thickness=-1)
    img.draw_string(8, img_h - 36, f"SENSITIVITY: {sens}%", color=image.COLOR_WHITE)
    img.draw_rect(slider_x, slider_y, slider_w, 4, color=image.COLOR_WHITE, thickness=-1)
    img.draw_rect(knob_x - 4, slider_y - 8, 8, 20, color=image.COLOR_GREEN, thickness=-1)


while not app.need_exit():
    img = cam.read()

    # Dynamic touch slider sensitivity update
    sensitivity = update_sensitivity_from_touch(img, sensitivity)
    green_thresholds = get_green_threshold(sensitivity)

    # Detect blobs (Blob returned as tuple-like: x, y, w, h, pixels...)
    red_blobs = img.find_blobs(
        [BASE_RED_THRESHOLD],
        pixels_threshold=MIN_BLOB_W * MIN_BLOB_H,
        area_threshold=MIN_BLOB_W * MIN_BLOB_H,
    )
    green_blobs = img.find_blobs(
        green_thresholds,
        pixels_threshold=MIN_BLOB_W * MIN_BLOB_H,
        area_threshold=MIN_BLOB_W * MIN_BLOB_H,
    )

    detected_objs = []

    for blob in red_blobs:
        x, y, w, h = blob[0], blob[1], blob[2], blob[3]
        if w >= MIN_BLOB_W and h >= MIN_BLOB_H:
            detected_objs.append((RED_CLASS_ID, x, y, w, h))

    for blob in green_blobs:
        x, y, w, h = blob[0], blob[1], blob[2], blob[3]
        if w >= MIN_BLOB_W and h >= MIN_BLOB_H:
            detected_objs.append((GREEN_CLASS_ID, x, y, w, h))

    # Transmit via UART using the string protocol: $N;cls,x,y,w,h;...*\n
    if report_on:
        if len(detected_objs) > 0:
            parts = [str(len(detected_objs))]
            for cls_id, x, y, w, h in detected_objs:
                parts.append(f"{cls_id},{x},{y},{w},{h}")
            frame = "$" + ";".join(parts) + "*\n"
            serial.write(frame.encode())
        else:
            # Heartbeat frame
            serial.write(b"$0*\n")

    # Draw detected bounding boxes
    for cls_id, x, y, w, h in detected_objs:
        if cls_id == RED_CLASS_ID:
            img.draw_rect(x, y, w, h, color=image.COLOR_RED)
            img.draw_string(x, y, f"red w:{w}", color=image.COLOR_RED)
        else:
            img.draw_rect(x, y, w, h, color=image.COLOR_GREEN)
            img.draw_string(x, y, f"green w:{w}", color=image.COLOR_GREEN)

    # Render interactive UI slider
    draw_tuning_ui(img, sensitivity)

    dis.show(img)