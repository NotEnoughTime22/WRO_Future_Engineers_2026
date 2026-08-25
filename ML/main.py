from maix import camera, display, image, nn, app, i2c, pinmap, err
import struct, os

report_on = True

# --- I2C CONFIGURATION ---
TARGET_I2C_ADDR = 0x24  # Replace with your receiving microcontroller's I2C address


err.check_raise(pinmap.set_pin_function("A15", "I2C5_SCL"), "set SCL failed")
err.check_raise(pinmap.set_pin_function("A27", "I2C5_SDA"), "set SDA failed")

i2c_bus = i2c.I2C(5, i2c.Mode.MASTER)
# -------------------------

def encode_objs(objs):
    '''
        encode objs info to bytes body for protocol
        2B x(LE) + 2B y(LE) + 2B w(LE) + 2B h(LE) + 2B idx + 4B score(float) ...
        Total: 14 bytes per object
    '''
    body = b''
    for obj in objs:
        body += struct.pack("<hhHHHf", obj.x, obj.y, obj.w, obj.h, obj.class_id, obj.score)
    return body

model_path = "model_317447.mud"
if not os.path.exists(model_path):
    model_path = "/root/models/maixhub/317447/model_317447.mud"
detector = nn.YOLOv5(model=model_path)

cam = camera.Camera(detector.input_width(), detector.input_height(), detector.input_format())
dis = display.Display()

while not app.need_exit():
    img = cam.read()
    objs = detector.detect(img, conf_th=0.5, iou_th=0.45)

    if len(objs) > 0 and report_on:
        body = encode_objs(objs)
        try:
            # Send the byte payload to the target microcontroller over I2C
            i2c_bus.writeto(TARGET_I2C_ADDR, body)
        except Exception as e:
            print(f"I2C Write Error: {e}")

    for obj in objs:
        img.draw_rect(obj.x, obj.y, obj.w, obj.h, color=image.COLOR_RED)
        msg = f'{detector.labels[obj.class_id]}: {obj.score:.2f}'
        img.draw_string(obj.x, obj.y, msg, color=image.COLOR_RED)

    dis.show(img)