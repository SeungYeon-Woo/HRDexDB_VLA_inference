#!/usr/bin/env python3
import argparse
import time
from datetime import datetime, timezone
from threading import Event

import cv2

from paradex.io.camera_system.camera_reader import MultiCameraReader
from paradex.io.capture_pc.command_sender import CommandReceiver
from paradex.io.capture_pc.data_sender import DataPublisher


parser = argparse.ArgumentParser()
parser.add_argument("--width", type=int, default=640)
parser.add_argument("--height", type=int, default=480)
parser.add_argument("--jpeg-quality", type=int, default=90)
parser.add_argument("--port", type=int, default=1234)
parser.add_argument("--command-port", type=int, default=6890)
parser.add_argument("--camera-names", default="", help="Comma-separated camera names/serials to stream. Empty streams all local cameras.")
parser.add_argument("--fps", type=float, default=0.0, help="Optional max publish FPS. 0 means publish every new frame.")
args = parser.parse_args()

# Publishes HRDexDB-compatible live frames: 480x640 RGB-equivalent payload.
# CameraReader images are treated as OpenCV images and JPEG encoded. The robot-side
# bridge decodes with cv2.imdecode (BGR) and converts BGR->RGB before sending to RLDX.
dp = DataPublisher(port=args.port, name="camera_stream_hrdex")
exit_event = Event()
cr = CommandReceiver(event_dict={"exit": exit_event}, port=args.command_port)
camera_names = [x.strip() for x in args.camera_names.split(",") if x.strip()] or None
reader = MultiCameraReader(camera_names=camera_names)
last_frame_ids = {name: 0 for name in reader.camera_names}
last_publish_time = 0.0
last_heartbeat_time = 0.0
published_batches = 0
published_frames = 0
no_new_frame_loops = 0
print({
    "stream": "hrdex",
    "cameras": list(reader.camera_names),
    "target_shape": [args.height, args.width, 3],
    "jpeg_quality": args.jpeg_quality,
    "fps_limit": args.fps,
}, flush=True)

try:
    while not exit_event.is_set():
        if args.fps > 0:
            now = time.time()
            min_period = 1.0 / args.fps
            if now - last_publish_time < min_period:
                time.sleep(0.001)
                continue

        images_data = reader.get_images(copy=True)
        meta_data = []
        binary_data = []

        for camera_name, (image, frame_id) in images_data.items():
            if frame_id <= last_frame_ids[camera_name] or frame_id <= 0:
                continue

            resized = cv2.resize(image, (args.width, args.height), interpolation=cv2.INTER_AREA)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), int(args.jpeg_quality)]
            success, encoded_image = cv2.imencode(".jpg", resized, encode_param)
            if not success:
                continue

            published_at_unix = time.time()
            meta_data.append({
                "type": "image",
                "name": camera_name,
                "frame_id": int(frame_id),
                "shape": tuple(int(x) for x in resized.shape),
                "encoding": "jpeg_bgr_cv2",
                "target_shape": [args.height, args.width, 3],
                "timestamp": datetime.fromtimestamp(published_at_unix, tz=timezone.utc).isoformat(),
                "timestamp_unix": published_at_unix,
                "data_index": len(binary_data),
            })
            binary_data.append(encoded_image)
            last_frame_ids[camera_name] = frame_id

        if meta_data:
            dp.send_data(meta_data, binary_data)
            last_publish_time = time.time()
            published_batches += 1
            published_frames += len(meta_data)
            no_new_frame_loops = 0
        else:
            no_new_frame_loops += 1

        now = time.time()
        if now - last_heartbeat_time >= 2.0:
            print({
                "stream": "hrdex",
                "published_batches": published_batches,
                "published_frames": published_frames,
                "last_frame_ids": dict(last_frame_ids),
                "no_new_frame_loops": no_new_frame_loops,
            }, flush=True)
            last_heartbeat_time = now

        time.sleep(0.01)
finally:
    reader.close()
    dp.close()
    cr.end()
    print("HRDexDB camera streaming client stopped.")
