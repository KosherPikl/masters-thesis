'''
Tuck Forbes main camera code version 2
8/22/25
'''
import sensor
import time
import math
from machine import UART
from machine import LED


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
clock = time.clock()

lens_mm = 2.8  # Standard Lens.
lens_to_camera_mm = 22  # Standard Lens.
sensor_w_mm = 4.592  # For OV5650 sensor
sensor_h_mm = 3.423  # For OV5650 sensor

x_res = 160  # QQVGA
y_res = 120  # QQVGA
f_x = (lens_mm / sensor_w_mm) * x_res
f_y = (lens_mm / sensor_h_mm) * y_res
c_x = x_res / 2
c_y = y_res / 2


def translation_to_mm(translation, tag_size, tag_id):
    # translation is in decimeters...
    if tag_id == 9:
        return ((translation * 100) * tag_size) / 380
    elif tag_id == 10:
        return ((translation * 100) * tag_size) / 40.5
    elif tag_id == 422:
        return ((translation * 100) * tag_size) / 122
    else:
        return 0



def degrees(radians):
    return (180 * radians) / math.pi

valid_tag_ids = {
    422: 87,
    9: 61,
    10: 20,
}

uart = UART(1, 19200, timeout_char=200)
led = LED("LED_BLUE")

p = pyb.Pin("P13",pyb.Pin.OUT_PP)

p.low()

while True:
    clock.tick()
    led.on()
    img = sensor.snapshot()
    tags = sorted(
        img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y),
        key=lambda x: x.w * x.h,
        reverse=True,
    )
    target_found = False



    if tags and (tags[0].id in valid_tag_ids):
        # print(tags[0])
        target_found = True
        tag_size = valid_tag_ids[tags[0].id]
        img.draw_rectangle(tags[0].rect, color=(255, 0, 0))
        img.draw_cross(tags[0].cx, tags[0].cy, color=(0, 255, 0))
        print_args = (
            translation_to_mm(tags[0].x_translation, tag_size, tags[0].id),
            translation_to_mm(tags[0].y_translation, tag_size, tags[0].id),
            translation_to_mm(tags[0].z_translation, tag_size, tags[0].id),
            degrees(tags[0].x_rotation),
            degrees(tags[0].y_rotation),
            degrees(tags[0].z_rotation),
            tags[0].id,
        )
        print("Tx: %f, Ty: %f, Tz: %f, Rx: %f, Ry: %f, Rz: %f, ID: %f" % print_args)
        p.high()
        uart.write("%f %f %f %f %f %f %f " % print_args)
        # Translation units are in mm. Rotation units are in degrees.
        p.low()
        time.sleep(5)
    else:
        print("FPS %f" % clock.fps())

