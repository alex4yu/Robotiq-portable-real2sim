#!/usr/bin/env python3
"""
Record RealSense infrared left (infra1), infrared right (infra2), and color streams
to a single .bag file (no depth). Intended for devices such as D405C.

The script runs until Ctrl+C and writes to ~/robotiq_ws/recordings/YYYY-MM-DD-HH.MM.SS.bag

Usage examples:
  python3 src/record_infra_and_color.py
  python3 src/record_infra_and_color.py --serial <DEVICE_SERIAL>
  python3 src/record_infra_and_color.py --width 640 --height 480 --fps 15
"""

import argparse
import os
import signal
import sys
import time
from datetime import datetime

import pyrealsense2 as rs


HOME = os.path.expanduser("~")
RECORD_DIR = os.path.join(HOME, "robotiq_ws", "recordings")


def ensure_record_dir() -> None:
    os.makedirs(RECORD_DIR, exist_ok=True)


def next_bag_path() -> str:
    fname = datetime.now().strftime("%Y-%m-%d-%H.%M.%S") + ".bag"
    return os.path.join(RECORD_DIR, fname)


class InfraColorRecorder:
    def __init__(self, width: int, height: int, fps: int, serial: str | None):
        self.width = width
        self.height = height
        self.fps = fps
        self.serial = serial
        self.pipe: rs.pipeline | None = None
        self.cfg: rs.config | None = None
        self.current_path: str | None = None
        self.is_recording: bool = False

    def start(self, bag_path: str) -> None:
        if self.is_recording:
            print("[INFO] Already recording")
            return

        self.pipe = rs.pipeline()
        self.cfg = rs.config()

        if self.serial:
            self.cfg.enable_device(self.serial)

        # Enable IR left (index 1) and IR right (index 2) as grayscale (y8)
        self.cfg.enable_stream(
            rs.stream.infrared, 1, self.width, self.height, rs.format.y8, self.fps
        )
        self.cfg.enable_stream(
            rs.stream.infrared, 2, self.width, self.height, rs.format.y8, self.fps
        )

        # Enable color RGB
        self.cfg.enable_stream(
            rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps
        )

        # Record to file (no depth enabled)
        self.cfg.enable_record_to_file(bag_path)

        print(f"[INFO] Starting recording to: {bag_path}")
        try:
            self.pipe.start(self.cfg)
        except Exception as e:
            print(f"[ERROR] Failed to start pipeline: {e}")
            # Clean up to leave object reusable
            self.pipe = None
            self.cfg = None
            raise

        self.current_path = bag_path
        self.is_recording = True

    def spin(self) -> None:
        if not self.pipe:
            return
        # Service SDK to push frames and keep file writing
        try:
            self.pipe.poll_for_frames()
        except Exception:
            pass
        time.sleep(0.02)

    def stop(self) -> None:
        if not self.is_recording:
            print("[INFO] Not recording")
            return
        print("[INFO] Stopping recording…")
        try:
            if self.pipe:
                self.pipe.stop()
        except Exception as e:
            print(f"[WARN] pipeline stop error: {e}")
        finally:
            self.pipe = None
            self.cfg = None
            self.is_recording = False
            if self.current_path:
                print(f"[OK] Saved: {self.current_path}")
            self.current_path = None


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Record infra1, infra2, and color to .bag (no depth)")
    p.add_argument("--serial", type=str, default=None, help="Specific camera serial number to use")
    p.add_argument("--width", type=int, default=640, help="Stream width")
    p.add_argument("--height", type=int, default=480, help="Stream height")
    p.add_argument("--fps", type=int, default=15, help="Stream FPS")
    p.add_argument("--out", type=str, default=None, help="Output .bag path (default: timestamped under recordings dir)")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    ensure_record_dir()

    bag_path = args.out if args.out else next_bag_path()
    rec = InfraColorRecorder(width=args.width, height=args.height, fps=args.fps, serial=args.serial)

    # Graceful shutdown on Ctrl+C
    def handle_sigint(sig, frame):
        print("\n[INFO] Ctrl+C received")
        if rec.is_recording:
            rec.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        rec.start(bag_path)
        print("[READY] Recording infra1, infra2, and color. Press Ctrl+C to stop.")
        print(f"[PATH ] {bag_path}")
        while True:
            rec.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rec.stop()


if __name__ == "__main__":
    main()


