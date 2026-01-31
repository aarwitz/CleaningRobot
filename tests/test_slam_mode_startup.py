import subprocess
import time
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import os

# SLAM-related topics to check
EXPECTED_TOPICS = [
    '/visual_slam/image_0',
    '/visual_slam/image_1',
    '/visual_slam/status',
    '/visual_slam/tracking/odometry',
]

EXPECTED_RATES = {
    '/visual_slam/image_0': (12, 30),
    '/visual_slam/image_1': (12, 30),
    '/visual_slam/tracking/odometry': (10, 40),
}


def get_active_topics():
    try:
        output = subprocess.check_output('ros2 topic list', shell=True, stderr=subprocess.STDOUT, text=True)
        return output.strip().split('\n')
    except subprocess.CalledProcessError:
        return []


def get_topic_rate(topic, timeout_s=6):
    proc = subprocess.Popen(
        ["stdbuf", "-oL", "ros2", "topic", "hz", topic],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    start = time.time()
    try:
        while True:
            line = proc.stdout.readline()
            if not line:
                if time.time() - start > timeout_s:
                    break
                continue
            if "average rate:" in line:
                proc.terminate()
                return float(line.split(":")[1].strip())
    finally:
        proc.kill()
    return None


def capture_slam_image(timeout_s=8):
    """Subscribe to `/visual_slam/image_0` and save first image seen to example_slam_image_1.png"""
    rclpy.init()

    class Saver(Node):
        def __init__(self):
            super().__init__('test_slam_capture')
            self.bridge = CvBridge()
            self.img = None
            self.sub = self.create_subscription(Image, '/visual_slam/image_0', self.cb, 10)

        def cb(self, msg: Image):
            try:
                # try color first, fall back to mono
                self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception:
                try:
                    mono = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                    self.img = cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
                except Exception:
                    self.get_logger().error('Failed to convert SLAM image')

    node = Saver()
    start = time.time()
    filename = None
    try:
        while time.time() - start < timeout_s:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.img is not None:
                filename = os.path.join(os.getcwd(), 'example_slam_image_1.png')
                cv2.imwrite(filename, node.img)
                print(f'Saved SLAM image to {filename}')
                break
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()

    if filename is None:
        print('No SLAM image captured within timeout')

    return filename


def main():
    print('Waiting 5s for system to stabilize...')
    time.sleep(5)

    topics = get_active_topics()
    print('Topics:', topics)

    missing = [t for t in EXPECTED_TOPICS if t not in topics]
    if missing:
        print('WARNING: Missing SLAM topics:', missing)
    else:
        print('SLAM topics present')

    # attempt to read status once with ros2 topic echo --once
    vo_state = None
    status_raw = None
    try:
        status_raw = subprocess.check_output('timeout 3 ros2 topic echo /visual_slam/status --once', shell=True, stderr=subprocess.STDOUT, text=True)
        print('SLAM status (sample):')
        print(status_raw.strip())
        # parse vo_state if available
        for line in status_raw.splitlines():
            if line.strip().startswith('vo_state:'):
                try:
                    vo_state = int(line.split(':', 1)[1].strip())
                except Exception:
                    vo_state = None
                break
    except subprocess.CalledProcessError:
        print('Could not read /visual_slam/status (may be unavailable or timing out)')

    if vo_state is not None:
        print(f'Parsed vo_state={vo_state}')

    # Save one SLAM image for visual inspection
    slam_img_path = capture_slam_image(timeout_s=8)

    # If we captured an image, overlay vo_state and measured odom rate (best-effort)
    if slam_img_path is not None and os.path.exists(slam_img_path):
        try:
            img = cv2.imread(slam_img_path)
            overlay_lines = []
            if vo_state is not None:
                overlay_lines.append(f'vo_state={vo_state}')

            # try to measure odom rate quickly
            odom_rate = get_topic_rate('/visual_slam/tracking/odometry', timeout_s=3)
            if odom_rate is not None:
                overlay_lines.append(f'odom_rate={odom_rate:.2f}Hz')

            # draw lines
            y = 30
            for ln in overlay_lines:
                cv2.putText(img, ln, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                y += 30

            cv2.imwrite(slam_img_path, img)
            print(f'Annotated SLAM image with status at {slam_img_path}')
        except Exception as e:
            print('Failed to annotate SLAM image:', e)

    # Check publish rates
    print('\nChecking topic publish rates...')
    for topic, (mi, ma) in EXPECTED_RATES.items():
        print(f'Checking rate for {topic}...')
        rate = get_topic_rate(topic)
        print('rate :', rate)
        if rate is None:
            print(f'ERROR: Could not determine rate for {topic}')
        elif not (mi <= rate <= ma):
            print(f'ERROR: {topic} publishing at {rate:.2f} Hz (expected {mi}-{ma} Hz)')
        else:
            print(f'{topic} publishing at {rate:.2f} Hz (OK)')

    # Optionally check for nvblx/nav2 via ros2 node list
    try:
        nodes = subprocess.check_output('ros2 node list', shell=True, stderr=subprocess.STDOUT, text=True).strip().split('\n')
        print('\nNodes:', nodes)
        nv_nodes = [n for n in nodes if 'nvblox' in n or 'nav2' in n.lower()]
        if nv_nodes:
            print('Found nvblx/nav2 related nodes:', nv_nodes)
        else:
            print('No nvblox/nav2 nodes detected in node list (may be disabled)')
    except Exception:
        print('Failed to list nodes')


if __name__ == '__main__':
    main()
