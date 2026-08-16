#!/usr/bin/env python3
"""
Pure Python TensorRT YOLOv8 inference node.

Completely bypasses Isaac ROS GXF/NITROS pipeline. Uses TensorRT Python API
with PyTorch CUDA tensors for GPU memory management.

Subscribes to camera RGB image, runs YOLOv8 inference, publishes
Detection2DArray on /yolo/detections in network-space coordinates (640x640).
The downstream clothes_perception node handles coordinate conversion.
"""

import os
import time

import cv2
import numpy as np
import tensorrt as trt
import torch

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

# COCO class names (80 classes) - used for mapping numeric IDs to human-readable labels
COCO_NAMES = [
    'person','bicycle','car','motorcycle','airplane','bus','train','truck','boat','traffic light',
    'fire hydrant','stop sign','parking meter','bench','bird','cat','dog','horse','sheep','cow',
    'elephant','bear','zebra','giraffe','backpack','umbrella','handbag','tie','suitcase','frisbee',
    'skis','snowboard','sports ball','kite','baseball bat','baseball glove','skateboard','surfboard','tennis racket','bottle',
    'wine glass','cup','fork','knife','spoon','bowl','banana','apple','sandwich','orange',
    'broccoli','carrot','hot dog','pizza','donut','cake','chair','couch','potted plant','bed',
    'dining table','toilet','tv','laptop','mouse','remote','keyboard','cell phone','microwave','oven',
    'toaster','sink','refrigerator','book','clock','vase','scissors','teddy bear','hair drier','toothbrush'
]


class YoloTrtNode(Node):
    def __init__(self):
        super().__init__('yolo_trt_node')

        # -- Parameters --
        self.declare_parameter('model_file_path', '/models/yolov8s.onnx')
        self.declare_parameter('engine_file_path', '/models/yolov8s.plan')
        self.declare_parameter('confidence_threshold', 0.65)
        self.declare_parameter('nms_threshold', 0.5)
        self.declare_parameter('num_classes', 80)
        self.declare_parameter('network_width', 640)
        self.declare_parameter('network_height', 640)
        self.declare_parameter('input_topic', '/camera/color/image_raw')
        self.declare_parameter('output_topic', '/yolo/detections')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('force_engine_update', False)
        # socks2.onnx was fine-tuned on BGR-ordered images (measured
        # 2026-08-16: BGR 0.54-0.81 vs RGB 0.05-0.67 on the same frames,
        # catastrophic on wrist views). Default False = standard YOLOv8 RGB.
        self.declare_parameter('bgr_input', False)

        self.onnx_path = str(self.get_parameter('model_file_path').value)
        self.engine_path = str(self.get_parameter('engine_file_path').value)
        self.conf_threshold = float(self.get_parameter('confidence_threshold').value)
        self.nms_threshold = float(self.get_parameter('nms_threshold').value)
        self.num_classes = int(self.get_parameter('num_classes').value)
        self.net_w = int(self.get_parameter('network_width').value)
        self.net_h = int(self.get_parameter('network_height').value)
        rate_hz = float(self.get_parameter('rate_hz').value)

        force_update = self.get_parameter('force_engine_update').value
        if isinstance(force_update, str):
            force_update = force_update.lower() in ('true', '1', 'yes')

        self.min_interval = 1.0 / rate_hz
        self.last_inference_time = 0.0

        # -- TensorRT init --
        self.trt_logger = trt.Logger(trt.Logger.INFO)
        self.engine = None
        self.trt_context = None
        self.stream = None
        self.input_name = None
        self.output_name = None

        if force_update and os.path.exists(self.engine_path):
            self.get_logger().info(f'Force engine update: removing {self.engine_path}')
            os.remove(self.engine_path)

        if not self._init_engine():
            self.get_logger().error('TensorRT engine init failed – node will not process images')
            # Still create pub/sub so the node is visible in the graph
            self._setup_ros()
            return

        self._setup_ros()

        # Load class names: prefer a file in /models/coco.names, otherwise use embedded list
        try:
            if os.path.exists('/models/coco.names'):
                with open('/models/coco.names', 'r') as f:
                    self.class_names = [l.strip() for l in f.readlines() if l.strip()]
            else:
                self.class_names = COCO_NAMES
        except Exception:
            self.class_names = COCO_NAMES

        self.get_logger().info(
            f'YoloTrtNode ready | {self.get_parameter("input_topic").value} -> '
            f'{self.get_parameter("output_topic").value} | '
            f'{self.net_w}x{self.net_h} | rate={rate_hz}Hz | '
            f'conf={self.conf_threshold} nms={self.nms_threshold}'
        )

    # ------------------------------------------------------------------ ROS
    def _setup_ros(self):
        output_topic = str(self.get_parameter('output_topic').value)
        input_topic = str(self.get_parameter('input_topic').value)
        self.pub = self.create_publisher(Detection2DArray, output_topic, 10)
        self.sub = self.create_subscription(Image, input_topic, self._image_cb, 5)

    # ----------------------------------------------------------- TensorRT
    def _init_engine(self) -> bool:
        if os.path.exists(self.engine_path):
            self.get_logger().info(f'Loading cached engine: {self.engine_path}')
            runtime = trt.Runtime(self.trt_logger)
            with open(self.engine_path, 'rb') as f:
                self.engine = runtime.deserialize_cuda_engine(f.read())
        else:
            self.get_logger().info(
                f'Building engine from {self.onnx_path} – this takes several minutes on first run …'
            )
            self.engine = self._build_engine()

        if self.engine is None:
            return False

        self.trt_context = self.engine.create_execution_context()

        self.input_name = self.engine.get_tensor_name(0)
        self.output_name = self.engine.get_tensor_name(1)

        input_shape = self.engine.get_tensor_shape(self.input_name)
        output_shape = self.engine.get_tensor_shape(self.output_name)

        # Auto-detect num_classes from engine output shape: [1, 4+num_classes, num_anchors]
        detected_classes = output_shape[1] - 4
        if detected_classes > 0 and detected_classes != self.num_classes:
            self.get_logger().warn(
                f'num_classes parameter ({self.num_classes}) does not match '
                f'engine output ({detected_classes} classes). Using {detected_classes}.'
            )
            self.num_classes = detected_classes

        self.get_logger().info(
            f'Engine: input={self.input_name}{list(input_shape)} '
            f'output={self.output_name}{list(output_shape)} '
            f'num_classes={self.num_classes}'
        )

        self.trt_context.set_input_shape(self.input_name, (1, 3, self.net_h, self.net_w))
        self.stream = torch.cuda.Stream()

        # Warm-up: run one dummy inference so CUDA kernels are compiled
        self._warmup()
        return True

    def _build_engine(self):
        builder = trt.Builder(self.trt_logger)
        network = builder.create_network(
            1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
        )
        parser = trt.OnnxParser(network, self.trt_logger)

        with open(self.onnx_path, 'rb') as f:
            if not parser.parse(f.read()):
                for i in range(parser.num_errors):
                    self.get_logger().error(f'ONNX parse error: {parser.get_error(i)}')
                return None

        config = builder.create_builder_config()
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)  # 1 GB

        # FP16 disabled: causes sigmoid score underflow to ~0 on Jetson for YOLOv8 custom models
        self.get_logger().info('Building with FP32 (FP16 disabled to prevent score underflow)')

        t0 = time.monotonic()
        serialized = builder.build_serialized_network(network, config)
        elapsed = time.monotonic() - t0

        if serialized is None:
            self.get_logger().error('Engine build failed')
            return None

        self.get_logger().info(f'Engine built in {elapsed:.1f}s – saving to {self.engine_path}')
        with open(self.engine_path, 'wb') as f:
            f.write(serialized)

        runtime = trt.Runtime(self.trt_logger)
        return runtime.deserialize_cuda_engine(serialized)

    def _warmup(self):
        dummy = torch.zeros((1, 3, self.net_h, self.net_w), dtype=torch.float32, device='cuda')
        out_shape = tuple(self.trt_context.get_tensor_shape(self.output_name))
        out = torch.empty(out_shape, dtype=torch.float32, device='cuda')
        self.trt_context.set_tensor_address(self.input_name, dummy.data_ptr())
        self.trt_context.set_tensor_address(self.output_name, out.data_ptr())
        self.trt_context.execute_async_v3(self.stream.cuda_stream)
        self.stream.synchronize()
        self.get_logger().info('Warm-up inference complete')

    # ---------------------------------------------------------- Preprocess
    def _preprocess(self, img: np.ndarray):
        """
        Letterbox-resize *img* (H, W, 3 uint8 RGB) to (net_h, net_w).
        Returns (blob [1,3,H,W] float32, scale, pad_x, pad_y).
        """
        h, w = img.shape[:2]
        scale = min(self.net_w / w, self.net_h / h)
        new_w, new_h = int(w * scale), int(h * scale)
        pad_x = (self.net_w - new_w) // 2
        pad_y = (self.net_h - new_h) // 2

        resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        # Grey (114) letterbox fill – standard YOLOv8 convention
        padded = np.full((self.net_h, self.net_w, 3), 114, dtype=np.uint8)
        padded[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized

        blob = padded.astype(np.float32) / 255.0
        blob = blob.transpose(2, 0, 1)[np.newaxis]  # [1, 3, H, W]
        # CRITICAL: transpose() makes the array NON-contiguous, and
        # torch.from_numpy().cuda() preserves those strides -- but TensorRT
        # reads data_ptr() as dense NCHW, so it saw channel-scrambled garbage
        # and scored ~0.000 on everything (found 2026-08-16; the model itself
        # scores 0.7+ once the buffer is contiguous).
        return np.ascontiguousarray(blob), scale, pad_x, pad_y

    # --------------------------------------------------------- Postprocess
    def _postprocess(self, output: np.ndarray):
        """
        YOLOv8 output: [1, 4+num_classes, N_anchors].
        Returns list of (cx, cy, w, h, score, class_id) in *network* coords.
        """
        # [1, 5, 8400] -> [8400, 5]
        preds = output[0].T

        boxes = preds[:, :4]                       # cx, cy, w, h
        scores = preds[:, 4:4 + self.num_classes]  # class scores

        class_ids = np.argmax(scores, axis=1)
        max_scores = scores[np.arange(len(scores)), class_ids]

        # Diagnostic: log top score before thresholding
        if len(max_scores) > 0:
            top_idx = np.argmax(max_scores)
            self.get_logger().info(
                f'Top raw score: {max_scores[top_idx]:.3f} (class {class_ids[top_idx]}), '
                f'threshold: {self.conf_threshold}, '
                f'above_thresh: {int(np.sum(max_scores > self.conf_threshold))}',
                throttle_duration_sec=2.0,
            )

        mask = max_scores > self.conf_threshold
        boxes = boxes[mask]
        max_scores = max_scores[mask]
        class_ids = class_ids[mask]

        if len(boxes) == 0:
            return []

        # Corner format for NMS
        x1 = boxes[:, 0] - boxes[:, 2] / 2
        y1 = boxes[:, 1] - boxes[:, 3] / 2
        x2 = boxes[:, 0] + boxes[:, 2] / 2
        y2 = boxes[:, 1] + boxes[:, 3] / 2

        keep = self._nms(x1, y1, x2, y2, max_scores, self.nms_threshold)

        results = []
        for i in keep:
            # Keep coordinates in network space (clothes_perception converts them)
            results.append((
                float(boxes[i, 0]),  # cx
                float(boxes[i, 1]),  # cy
                float(boxes[i, 2]),  # w
                float(boxes[i, 3]),  # h
                float(max_scores[i]),
                int(class_ids[i]),
            ))
        return results

    @staticmethod
    def _nms(x1, y1, x2, y2, scores, iou_threshold=0.5):
        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
            iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)
            order = order[np.where(iou <= iou_threshold)[0] + 1]
        return keep

    # ------------------------------------------------------------- Callback
    def _image_cb(self, msg: Image):
        # Rate-limit
        now = time.monotonic()
        if now - self.last_inference_time < self.min_interval:
            return
        self.last_inference_time = now

        if self.trt_context is None:
            return

        # Decode image
        self.get_logger().info(
            f'Image: {msg.width}x{msg.height} {msg.encoding}',
            throttle_duration_sec=10.0,
        )
        if msg.encoding == 'rgb8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        elif msg.encoding == 'bgr8':
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            img = raw[:, :, ::-1].copy()  # BGR -> RGB
        else:
            self.get_logger().warn(
                f'Unsupported encoding: {msg.encoding}', throttle_duration_sec=5.0
            )
            return
        if self.get_parameter('bgr_input').value:
            img = img[:, :, ::-1]   # model wants BGR; _preprocess handles views

        # Preprocess
        blob, _scale, _pad_x, _pad_y = self._preprocess(img)

        # Inference
        input_t = torch.from_numpy(blob).cuda()
        out_shape = tuple(self.trt_context.get_tensor_shape(self.output_name))
        output_t = torch.empty(out_shape, dtype=torch.float32, device='cuda')

        self.trt_context.set_tensor_address(self.input_name, input_t.data_ptr())
        self.trt_context.set_tensor_address(self.output_name, output_t.data_ptr())
        self.trt_context.execute_async_v3(self.stream.cuda_stream)
        self.stream.synchronize()

        output_np = output_t.cpu().numpy()

        # Postprocess – detections in network-space coordinates
        detections = self._postprocess(output_np)

        # Build Detection2DArray
        det_array = Detection2DArray()
        det_array.header = msg.header

        for cx, cy, w, h, score, class_id in detections:
            det = Detection2D()
            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.size_x = w
            det.bbox.size_y = h

            hyp = ObjectHypothesisWithPose()
            # Publish human-readable class name when possible; fall back to numeric id
            try:
                if isinstance(class_id, int) and class_id < len(self.class_names):
                    class_name = self.class_names[class_id]
                else:
                    class_name = str(class_id)
            except Exception:
                class_name = str(class_id)
            hyp.hypothesis.class_id = class_name
            hyp.hypothesis.score = score
            det.results.append(hyp)

            det_array.detections.append(det)

        self.pub.publish(det_array)

        if detections:
            self.get_logger().info(
                f'Published {len(detections)} detection(s), '
                f'best={detections[0][4]:.2f}',
                throttle_duration_sec=2.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = YoloTrtNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
