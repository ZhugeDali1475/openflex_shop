#!/usr/bin/env python3
import argparse
import base64
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from io import BytesIO
import json
from pathlib import Path
import tkinter as tk
from urllib.request import ProxyHandler, Request, build_opener

from PIL import Image, ImageTk

from openflex_runtime import request_json, save


def describe_scene(run_dir):
    snapshot = request_json('http://127.0.0.1:18552/observation')
    camera = snapshot['sources']['camera']
    if snapshot['environment'] != 'mujoco_simulation' or camera['status'] != 'fresh':
        raise RuntimeError('没有可用的新鲜仿真相机帧')
    image = camera['data']
    jpeg = base64.b64decode(image['data'], validate=True)
    (run_dir / 'vision_input.jpg').write_bytes(jpeg)
    payload = {'model': 'robobrain2.5-4b-gguf', 'max_tokens': 180, 'temperature': 0,
               'messages': [{'role': 'user', 'content': [
                   {'type': 'text', 'text': '用简短中文描述相机图像里实际可见的货架和物体。只根据图像回答；形状或类别不确定就说明，不要猜测物体 ID 或三维坐标。'},
                   {'type': 'image_url', 'image_url': {'url': f'data:image/jpeg;base64,{image["data"]}'}}]}]}
    request = Request('http://127.0.0.1:4567/v1/chat/completions', data=json.dumps(payload).encode(),
                      headers={'Content-Type': 'application/json'})
    with build_opener(ProxyHandler({})).open(request, timeout=90) as response:
        result = json.load(response)
    choice = result['choices'][0]
    report = {'time': datetime.now().astimezone().isoformat(), 'camera_stamp_ns': camera['source_stamp_ns'],
              'scope': 'display_only_actual_model_output_not_robot_control', 'visual_identity_verified': False,
              'finish_reason': choice.get('finish_reason'), 'text': choice['message'].get('content') or '',
              'raw_response': result}
    save(run_dir / 'vision_result.json', report)
    return report


def read_json(path, default):
    try:
        return json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return default


def task_status(task_dir):
    lines = []
    model = read_json(task_dir / 'model_trace.json', [])
    if model:
        calls = model[-1].get('tool_calls', [])
        if calls:
            selected = calls[0]
            lines.append(f'模型第 {len(model)} 次选择：{selected["name"]} {selected["arguments"]}')
    for item in read_json(task_dir / 'tool_trace.json', []):
        feedback = item.get('terminal_result') or item.get('raw_result') or {}
        status = feedback.get('status', '有反馈')
        lines.append(f'  {item["selected_tool"]} → {status}')
    result = read_json(task_dir / 'result.json', {})
    if result:
        lines.append('任务：已独立验证交付成功' if result.get('passed') is True
                     else f'任务：未验证成功（{result.get("status", "failed")}）')
    return '\n'.join(lines) or '等待启动完整任务…'


def main():
    parser = argparse.ArgumentParser(description='Read-only OpenFlex model output and tool-feedback panel')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--task-dir', required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    root = tk.Tk()
    root.title('OpenFlex | RoboBrain 相机描述与执行反馈')
    root.geometry('600x780+2930+60')
    root.configure(padx=16, pady=12)
    tk.Label(root, text='RoboBrain · 相机图像与模型输出', font=('Sans', 17, 'bold')).pack(anchor='w')
    tk.Label(root, text='只读显示 · 隔离 MuJoCo · 不参与运动控制').pack(anchor='w')
    picture = tk.Label(root, text='等待采集相机画面…')
    picture.pack(pady=8)
    description = tk.Label(root, text='等待真实推理输出…', justify='left', wraplength=548)
    description.pack(anchor='w')
    tk.Label(root, text='视觉描述可能出错；抓取 ID 和位姿来自仿真真值。', fg='#8a4000').pack(anchor='w', pady=8)
    tk.Label(root, text='实际模型选技与后端反馈', font=('Sans', 13, 'bold')).pack(anchor='w')
    feedback = tk.Label(root, justify='left', anchor='nw', wraplength=548)
    feedback.pack(fill='both', expand=True)
    pool = ThreadPoolExecutor(max_workers=1)
    future = pool.submit(describe_scene, args.output)
    shown = False

    def tick():
        nonlocal shown
        path = args.output / 'vision_input.jpg'
        if path.exists() and picture.image is None:
            try:
                image = Image.open(BytesIO(path.read_bytes()))
                image.thumbnail((500, 340))
                picture.image = ImageTk.PhotoImage(image.resize((400, 300)))
                picture.configure(image=picture.image, text='')
            except OSError:
                pass
        if future.done() and not shown:
            shown = True
            try:
                result = future.result()
                description.configure(text='模型原文：' + (result['text'] or '未返回文字'))
            except Exception as error:
                description.configure(text=f'本次视觉推理失败：{error}')
                save(args.output / 'vision_error.json', {'error': str(error)})
        feedback.configure(text=task_status(args.task_dir))
        root.after(500, tick)

    picture.image = None
    root.after(0, tick)
    try:
        root.mainloop()
    finally:
        pool.shutdown(wait=False)


if __name__ == '__main__':
    main()
