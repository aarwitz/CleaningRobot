
import subprocess
import time
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
import cv2
import numpy as np
import os


# Topics expected in DETECT mode (be permissive about exact clothes topic name)
EXPECTED_TOPICS = [
    '/camera/aligned_depth_to_color/image_raw',
    '/camera/color/image_raw',
    '/yolo/detections',
]
# Accept any of these clothes output topics (different nodes may publish different names)
POSSIBLE_CLOTHES_TOPICS = [
    '/clothes/detected',
    '/clothes/target_point_camera',
    '/clothes/target_point_map',
]
EXPECTED_RATES = {
    '/camera/aligned_depth_to_color/image_raw': (12, 18),  # 15 Hz ±3
    '/camera/color/image_raw': (25, 35),  # 15 Hz ±3
    '/yolo/detections': (25, 35),  # 10 Hz ±3
    '/clothes/target_point_camera': (5, 15),  # 10 Hz ±5
    # '/robot/state': (10, 20),  # 15 Hz ±5
}

DOCKER_CONTAINER = 'docker-vision-1'


def get_active_topics():
    cmd = f'ros2 topic list'
    try:
        output = subprocess.check_output(cmd, shell=True, stderr=subprocess.STDOUT, text=True)
        return output.strip().split('\n')
    except subprocess.CalledProcessError:
        return []

def get_active_nodes():
    cmd = f'ros2 node list'
    try:
        output = subprocess.check_output(cmd, shell=True, stderr=subprocess.STDOUT, text=True)
        return output.strip().split('\n')
    except subprocess.CalledProcessError:
        return []

def get_topic_rate(topic, timeout_s=6):
    proc = subprocess.Popen(
        ["stdbuf", "-oL", "ros2", "topic", "hz", topic],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1
    )

    start = time.time()

    try:
        while True:
            line = proc.stdout.readline()
            if not line:
                if time.time() - start > timeout_s:
                    break
                continue

            # DEBUG: uncomment to see lines
            # print("HZ:", line.rstrip())

            if "average rate:" in line:
                proc.terminate()
                return float(line.split(":")[1].strip())

    finally:
        proc.kill()

    return None


def capture_example_image(timeout_s=10):
    """Spin rclpy briefly to capture one RGB image and one YOLO detection,
    draw an overlay and save it to example_detection_1.png in CWD.
    """
    rclpy.init()

    class SaverNode(Node):
        def __init__(self):
            super().__init__('test_capture_node')
            self.bridge = CvBridge()
            self.latest_image = None
            self.latest_detection = None
            self.sub_img = self.create_subscription(
                Image, '/camera/color/image_raw', self.img_cb, 10)
            self.sub_det = self.create_subscription(
                Detection2DArray, '/yolo/detections', self.det_cb, 10)

        def img_cb(self, msg: Image):
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.latest_image = cv_img
            except Exception:
                self.get_logger().error('Failed to convert image')

        def det_cb(self, msg: Detection2DArray):
            if msg.detections:
                self.latest_detection = msg.detections[0]

    node = SaverNode()
    start = time.time()
    filename = None

    try:
        while time.time() - start < timeout_s:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.latest_image is not None and node.latest_detection is not None:
                img = node.latest_image.copy()
                det = node.latest_detection

                # Try to extract bbox center and size robustly
                def get_nested(obj, *attrs):
                    cur = obj
                    for a in attrs:
                        cur = getattr(cur, a, None)
                        if cur is None:
                            return None
                    return cur

                cx = get_nested(det, 'bbox', 'center', 'position', 'x')
                cy = get_nested(det, 'bbox', 'center', 'position', 'y')
                if cx is None or cy is None:
                    cx = get_nested(det, 'bbox', 'center', 'x')
                    cy = get_nested(det, 'bbox', 'center', 'y')

                sx = getattr(det.bbox, 'size_x', None) if hasattr(det, 'bbox') else None
                sy = getattr(det.bbox, 'size_y', None) if hasattr(det, 'bbox') else None

                h, w = img.shape[:2]

                drawn = False
                if cx is not None and cy is not None:
                    try:
                        ux = float(cx)
                        uy = float(cy)

                        # Heuristics for coordinate space:
                        # - normalized (0..1): cx<=1 and cy<=1
                        # - network-space (letterboxed, e.g. 640x640): values <= net_w/net_h
                        # - image-space: values <= image width/height
                        net_w = 640.0
                        net_h = 640.0

                        def network_to_image(u_net, v_net, img_w, img_h, net_w=net_w, net_h=net_h):
                            scale = min(net_w / img_w, net_h / img_h)
                            pad_x = (net_w - img_w * scale) / 2.0
                            pad_y = (net_h - img_h * scale) / 2.0
                            u_img = (u_net - pad_x) / scale
                            v_img = (v_net - pad_y) / scale
                            return u_img, v_img, scale

                        # Determine type
                        if ux <= 1.0 and uy <= 1.0:
                            # normalized coordinates [0..1]
                            ux_img = ux * w
                            uy_img = uy * h
                            size_x_img = float(sx) * w if sx is not None and sx <= 1.0 else None
                            size_y_img = float(sy) * h if sy is not None and sy <= 1.0 else None
                        elif ux <= net_w and uy <= net_h:
                            # network-space (letterboxed)
                            ux_img, uy_img, scale = network_to_image(ux, uy, w, h)
                            size_x_img = (float(sx) / scale) if sx is not None else None
                            size_y_img = (float(sy) / scale) if sy is not None else None
                        else:
                            # assume already image-space
                            ux_img = ux
                            uy_img = uy
                            size_x_img = float(sx) if sx is not None else None
                            size_y_img = float(sy) if sy is not None else None

                        # clamp
                        ux_img = max(0, min(ux_img, w - 1))
                        uy_img = max(0, min(uy_img, h - 1))

                        if size_x_img is not None and size_y_img is not None:
                            half_w = size_x_img / 2.0
                            half_h = size_y_img / 2.0
                            x1 = int(max(0, ux_img - half_w))
                            y1 = int(max(0, uy_img - half_h))
                            x2 = int(min(w - 1, ux_img + half_w))
                            y2 = int(min(h - 1, uy_img + half_h))
                            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            drawn = True
                        else:
                            cv2.circle(img, (int(ux_img), int(uy_img)), 10, (0, 255, 0), 2)
                            drawn = True
                    except Exception:
                        drawn = False

                if not drawn:
                    # fallback: annotate that detection was received
                    cv2.putText(img, 'Detection received', (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                1.0, (0, 255, 0), 2)

                # Save example
                out_dir = os.getcwd()
                filename = os.path.join(out_dir, 'example_detection_1.png')
                cv2.imwrite(filename, img)
                print(f'Saved example detection to {filename}')
                break

    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()

    if filename is None:
        print('No detection+image pair captured within timeout')


def main():
    print("Waiting 5 seconds for system to stabilize...")
    time.sleep(5)

    # We focus on topics/rates for functional verification. Node names can vary
    # between systems/launches (e.g., realsense node may be '/camera/camera').

    print("\nChecking active topics...")
    topics = get_active_topics()
    print("Topics:", topics)
    missing_topics = [t for t in EXPECTED_TOPICS if t not in topics]
    if missing_topics:
        print("WARNING: Missing some core topics:", missing_topics)
    else:
        print("Core topics are present.")

    # Check for any clothes output topic (be permissive)
    clothes_topic_found = None
    for ct in POSSIBLE_CLOTHES_TOPICS:
        if ct in topics:
            clothes_topic_found = ct
            break

    if clothes_topic_found:
        print(f'Found clothes topic: {clothes_topic_found}')
    else:
        print('WARNING: No clothes output topic found (checked common names)')

    # Try to capture one RGB image + YOLO detection and save an example overlay.
    capture_example_image(timeout_s=10)

    print("\nChecking topic publish rates...")
    for topic, (min_rate, max_rate) in EXPECTED_RATES.items():
        print(f"Checking rate for {topic}...")
        rate = get_topic_rate(topic)
        print("rate :", rate)
        if rate is None:
            print(f"ERROR: Could not determine rate for {topic}")
        elif not (min_rate <= rate <= max_rate):
            print(f"ERROR: {topic} publishing at {rate:.2f} Hz (expected {min_rate}-{max_rate} Hz)")
        else:
            print(f"{topic} publishing at {rate:.2f} Hz (OK)")


if __name__ == "__main__":
    main()
