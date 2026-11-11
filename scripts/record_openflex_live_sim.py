#!/usr/bin/env python3
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import time


def frame_state(last_sample, simulation_time, now, previous_time):
    if not math.isfinite(simulation_time) or last_sample <= 0 or now - last_sample > 0.5:
        return 'stale'
    if previous_time is not None and simulation_time <= previous_time:
        return 'clock_rollback' if simulation_time < previous_time else 'not_advanced'
    return 'fresh'


def main():
    parser = argparse.ArgumentParser(description='EGL recording of synchronized measured ROS state; no physics stepping')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=600)
    parser.add_argument('--fps', type=float, default=3)
    parser.add_argument('--width', type=int, default=960)
    parser.add_argument('--height', type=int, default=640)
    arguments = parser.parse_args()
    if not (math.isfinite(arguments.duration) and 0 < arguments.duration <= 1800
            and math.isfinite(arguments.fps) and 1 <= arguments.fps <= 10
            and 320 <= arguments.width <= 1920 and 240 <= arguments.height <= 1080
            and arguments.width % 2 == 0 and arguments.height % 2 == 0):
        parser.error('Bounded duration, 1-10 fps and even dimensions up to 1920x1080 required')
    if os.environ.get('ROS_DOMAIN_ID') != '83':
        raise RuntimeError('Recorder requires isolated ROS domain 83')
    os.environ['MUJOCO_GL'] = 'egl'
    import cv2
    import mujoco
    import rclpy
    from view_openflex_live_sim import LiveView

    arguments.output.mkdir(parents=True, exist_ok=False)
    manifest = {'scope': 'EGL_render_of_measured_ROS_mirror_NOT_desktop_NOT_PHYSICAL',
                'ros_domain': 83, 'physical_verification': False, 'physics_stepping': False,
                'fps': arguments.fps, 'started_at': datetime.now().astimezone().isoformat(),
                'valid_visual_evidence': False, 'task_success_evidence': False,
                'frames': 0, 'stale_intervals': 0, 'clock_rollback': False}
    interrupted = False

    def stop(signum, frame):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    rclpy.init()
    node = None
    writer = None
    try:
        node = LiveView()
        node.model.vis.global_.offwidth = max(node.model.vis.global_.offwidth, arguments.width)
        node.model.vis.global_.offheight = max(node.model.vis.global_.offheight, arguments.height)
        video = arguments.output / 'measured_state.mp4'
        writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*'mp4v'), arguments.fps,
                                 (arguments.width, arguments.height))
        if not writer.isOpened():
            raise RuntimeError('Video encoder could not open')
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(camera)
        camera.lookat[:] = [-0.4, -0.4, 0.7]
        camera.distance, camera.azimuth, camera.elevation = 3.6, 135, -25
        started = time.monotonic()
        next_frame = started
        previous_time = None
        with (arguments.output / 'frames.jsonl').open('w') as evidence:
            with mujoco.Renderer(node.model, height=arguments.height, width=arguments.width) as renderer:
                while rclpy.ok() and not interrupted and time.monotonic() - started < arguments.duration:
                    for callback_index in range(32):
                        rclpy.spin_once(node, timeout_sec=0.0)
                    node.update()
                    now = time.monotonic()
                    if now < next_frame:
                        time.sleep(min(0.01, next_frame - now))
                        continue
                    next_frame = now + 1 / arguments.fps
                    freshness = frame_state(node.last_sample, node.data.time, now, previous_time)
                    if freshness == 'clock_rollback':
                        manifest['clock_rollback'] = True
                        raise RuntimeError('Simulation clock restarted: new trial must have a separate video')
                    if freshness != 'fresh':
                        manifest['stale_intervals'] += 1
                        evidence.write(json.dumps({'wall_elapsed_sec': now - started, 'state': freshness}) + '\n')
                        evidence.flush()
                        continue
                    renderer.update_scene(node.data, camera=camera)
                    image = renderer.render()
                    bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                    cv2.putText(bgr, f'MEASURED ROS / SIM ONLY  t={node.data.time:.2f}s  wall={now-started:.1f}s',
                                (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (240, 240, 240), 1)
                    writer.write(bgr)
                    if manifest['frames'] == 0:
                        cv2.imwrite(str(arguments.output / 'first_frame.png'), bgr)
                    diagnostic = node.last_diagnostic
                    evidence.write(json.dumps({'frame': manifest['frames'], 'wall_elapsed_sec': now - started,
                        'simulation_time_sec': node.data.time, 'stamp': diagnostic['stamp'],
                        'object_pose': diagnostic['object_pose'], 'attached_arm': diagnostic.get('attached_arm'),
                        'table_support': diagnostic.get('delivery_support_contact'),
                        'safety_stop': diagnostic.get('safety_stop'), 'state': 'fresh'}) + '\n')
                    evidence.flush()
                    manifest['frames'] += 1
                    previous_time = node.data.time
        manifest['interrupted'] = interrupted
    except BaseException as error:
        manifest['error'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        if writer is not None:
            writer.release()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        video = arguments.output / 'measured_state.mp4'
        if video.exists():
            capture = cv2.VideoCapture(str(video))
            decoded = 0
            nonblank = 0
            while True:
                valid, image = capture.read()
                if not valid:
                    break
                decoded += 1
                nonblank += int(float(image.std()) > 5 and float(image.mean()) > 5)
            capture.release()
            manifest.update(decoded_frames=decoded, nonblank_frames=nonblank)
            manifest['valid_visual_evidence'] = bool(decoded >= 2 and decoded == manifest['frames']
                and nonblank == decoded and not manifest.get('error') and not manifest['clock_rollback'])
        manifest['playback_note'] = 'Only fresh frames encoded; gaps are recorded in frames.jsonl, not hidden as live frames'
        (arguments.output / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
        print(arguments.output / 'manifest.json')
    if not manifest['valid_visual_evidence']:
        raise RuntimeError('Recording not valid visual evidence; inspect manifest')


if __name__ == '__main__':
    main()
