# RoboOS + RoboBrain2.5 完整使用教程

更新日期：2026-11-09  
作者：基于项目计划书整理

本教程按照 `LLM_HIERARCHICAL_ROBOT_TASK_PLAN_CN.md` 的阶段划分，提供分步骤的实操指南。

> 2026-11-09 本机部署补充：已实际验证 RoboBrain2.5-4B GGUF 的 CUDA 图文推理及单工具返回，当前请优先使用 `ROBOBRAIN25_GGUF_LOCAL_CN.md` 与 `scripts/run_robobrain25_gguf.sh`。本文后续 Transformers／bitsandbytes 代码仍是待适配示例：本地官方 `UnifiedInference` 不接收示例中的 `load_in_4bit`、`torch_dtype` 参数，不能照抄运行；验收列表中的勾选不代表已有实测证据，具体完成状态以计划书和运行记录为准。

> 接口联调补充：真实 RoboOS 模型客户端已通过图文、工具参数和失败反馈检查，重跑入口为 `bash scripts/run_roboos_gguf_validation.sh`，范围与补丁见 `ROBOOS_GGUF_INTEGRATION_CN.md`。这不代表完整 Worker 或 ROS 技能已启动，不要运行默认 demo 技能宣称机器人成功。

---

## 📋 目录

1. [系统架构理解](#1-系统架构理解)
2. [阶段 B：RoboBrain2.5 推理环境](#2-阶段-b-robobrain25-推理环境)
3. [阶段 C：OpenFlex 小脑接入](#3-阶段-c-openflex-小脑接入)
4. [阶段 D：联调运行](#4-阶段-d-联调运行)
5. [常见问题](#5-常见问题)

---

## 1. 系统架构理解

### 1.1 角色分工

```
┌─────────────────────────────────────────────────────────┐
│                    用户自然语言任务                        │
│              "把红色杯子从货架送到交付台"                   │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│              RoboBrain2.5 (大脑)                         │
│  - 理解任务意图                                           │
│  - 选择技能序列 (观察 → 导航 → 抓取 → 放置)                │
│  - 根据失败反馈重新决策                                    │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│              RoboOS (协调层)                             │
│  - Master: 任务分解与分配                                 │
│  - Worker: 管理单个机器人的技能执行                        │
│  - Redis: 共享状态与反馈                                  │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼ MCP (Model Context Protocol)
┌─────────────────────────────────────────────────────────┐
│           OpenFlex 技能适配层 (小脑)                      │
│  - get_robot_state(): 机器人状态                         │
│  - observe_scene(): 场景感知                             │
│  - navigate_to_target(): 导航                           │
│  - grasp_object(): 抓取                                 │
│  - place_to_affordance(): 放置                          │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼ ROS 2 Action/Service
┌─────────────────────────────────────────────────────────┐
│           现有 OpenFlex 执行模块                          │
│  - MuJoCo 仿真环境                                       │
│  - TC-WAM 抓取算法                                       │
│  - MoveIt 运动规划                                       │
│  - 导航与底盘控制                                         │
└─────────────────────────────────────────────────────────┘
```

### 1.2 关键概念

**大脑 (RoboBrain2.5)**：
- 多模态大语言模型，能理解图像和文本
- 支持工具调用 (tool calling)
- 能接收执行反馈并重新规划

**小脑 (OpenFlex Skills)**：
- 现有的机器人控制能力
- 通过 MCP 协议暴露为工具
- 返回真实的执行结果

**协调层 (RoboOS)**：
- Master 负责全局任务分配
- Worker 负责单个机器人的技能调度
- 通过 Redis 同步状态

---

## 2. 阶段 B：RoboBrain2.5 推理环境

### B1: 验证 RoboBrain2.5-4B 量化加载

#### 2.1 创建独立环境

```bash
# 切换到推理环境目录
cd /home/wgb/openflex_shop/local_dependencies/robobrain25_b1

# 创建 conda 环境
conda create -n robobrain25 python=3.10 -y
conda activate robobrain25

# 安装基础依赖
cd /home/wgb/openflex_shop/local_dependencies/RoboBrain2.5
pip install -r requirements.txt

# 安装量化支持
pip install bitsandbytes accelerate
```

#### 2.2 下载模型权重

**方法一：使用 Hugging Face CLI（推荐）**

```bash
pip install huggingface_hub

# 下载 4B 模型
huggingface-cli download BAAI/RoboBrain2.5-4B \
    --local-dir /home/wgb/openflex_shop/local_dependencies/models/RoboBrain2.5-4B \
    --local-dir-use-symlinks False
```

**方法二：使用 Python 脚本下载**

```python
# download_model.py
from huggingface_hub import snapshot_download

model_path = snapshot_download(
    repo_id="BAAI/RoboBrain2.5-4B",
    local_dir="/home/wgb/openflex_shop/local_dependencies/models/RoboBrain2.5-4B",
    local_dir_use_symlinks=False,
    revision="ec4b3bf5e969b48ad17df432b479c10402a0079b"  # 计划书记录的 revision
)
print(f"Model downloaded to: {model_path}")
```

#### 2.3 测试量化推理

创建测试脚本 `test_b1_quantized.py`：

```python
import torch
import gc
from inference import UnifiedInference
import time

print("=" * 60)
print("B1 测试：RoboBrain2.5-4B 量化加载与推理")
print("=" * 60)

# 清空显存
if torch.cuda.is_available():
    torch.cuda.empty_cache()
    gc.collect()
    print(f"初始显存: {torch.cuda.memory_allocated(0) / 1024**3:.2f} GB")

# 模型路径
model_path = "/home/wgb/openflex_shop/local_dependencies/models/RoboBrain2.5-4B"

print(f"\n正在加载模型: {model_path}")
print("使用 4-bit 量化...")

start_time = time.time()

# 加载模型（4-bit 量化）
model = UnifiedInference(
    model_path,
    device_map="auto",
    load_in_4bit=True,
    torch_dtype=torch.bfloat16
)

load_time = time.time() - start_time
print(f"✓ 模型加载完成，耗时: {load_time:.2f} 秒")

if torch.cuda.is_available():
    peak_memory = torch.cuda.max_memory_allocated(0) / 1024**3
    current_memory = torch.cuda.memory_allocated(0) / 1024**3
    print(f"✓ 峰值显存: {peak_memory:.2f} GB")
    print(f"✓ 当前显存: {current_memory:.2f} GB")

# 测试 1: 文本-图像理解
print("\n" + "=" * 60)
print("测试 1: 通用图文理解")
print("=" * 60)

prompt = "这张图片里有什么？请用中文回答。"
image = "http://images.cocodataset.org/val2017/000000039769.jpg"

start_time = time.time()
pred = model.inference(prompt, image, task="general")
inference_time = time.time() - start_time

print(f"提示词: {prompt}")
print(f"预测结果: {pred}")
print(f"推理耗时: {inference_time:.2f} 秒")

# 测试 2: 空间指向任务（机器人相关）
print("\n" + "=" * 60)
print("测试 2: 空间指向任务（机器人感知）")
print("=" * 60)

prompt_spatial = "桌子上的杯子在哪里？"
# 注意：这里需要一个实际的图片路径
# image_spatial = "/path/to/your/robot/scene.jpg"

print(f"提示词: {prompt_spatial}")
print("（需要提供实际机器人场景图像）")

# 记录测试结果
print("\n" + "=" * 60)
print("B1 测试总结")
print("=" * 60)
print(f"✓ 模型加载成功")
print(f"✓ 量化方式: 4-bit")
print(f"✓ 峰值显存: {peak_memory:.2f} GB")
print(f"✓ 推理延迟: {inference_time:.2f} 秒/次")
print(f"✓ 模型 revision: ec4b3bf5e969b48ad17df432b479c10402a0079b")
print("\n建议：")
if peak_memory > 6.5:
    print("⚠️  显存占用较高，同时运行 MuJoCo 可能紧张")
    print("   考虑：关闭 MuJoCo GUI 或使用远程推理")
else:
    print("✓ 显存占用合理，可以继续本地推理")
```

运行测试：

```bash
conda activate robobrain25
cd /home/wgb/openflex_shop/local_dependencies/RoboBrain2.5
python test_b1_quantized.py
```

**验收标准**：
- [x] 模型能加载成功
- [x] 峰值显存 < 7 GB
- [x] 能返回中文图像描述
- [x] 推理延迟记录在案

---

### B2: 提供 RoboOS 兼容的模型服务

RoboOS 需要 OpenAI 兼容的 `/v1/chat/completions` API，而 RoboBrain2.5 的 `inference.py` 只是推理函数，不是服务端。

#### 2.4 理解 RoboOS 的调用方式

RoboOS 使用 OpenAI Python SDK 调用模型：

```python
# 来自 local_dependencies/RoboOS/slaver/agents/models.py:343
response = self.client.chat.completions.create(
    model=self.model_id,
    messages=messages,
    tools=tools,
    temperature=0.7,
    ...
)
```

需要的响应格式：
```json
{
  "choices": [{
    "message": {
      "role": "assistant",
      "content": "文本回复",
      "tool_calls": [{
        "id": "call_xxx",
        "type": "function",
        "function": {
          "name": "navigate_to_target",
          "arguments": "{\"target\": \"shelf_A\"}"
        }
      }]
    }
  }],
  "usage": {
    "prompt_tokens": 100,
    "completion_tokens": 50
  }
}
```

#### 2.5 方案选择

**方案 A：使用 vLLM（推荐）**

优点：
- 官方 RoboOS 文档推荐
- 支持 OpenAI 兼容 API
- 自带工具调用解析

缺点：
- 需要验证 RoboBrain2.5 的工具模板兼容性

**方案 B：自己封装 FastAPI 服务**

优点：
- 完全可控，可以自定义工具调用格式
- 可以直接使用 `inference.py`

缺点：
- 需要自己处理并发和流式输出

#### 2.6 实现方案 A：使用 vLLM

```bash
# 安装 vLLM
conda activate robobrain25
pip install vllm

# 启动服务
vllm serve /home/wgb/openflex_shop/local_dependencies/models/RoboBrain2.5-4B \
    --port 4567 \
    --gpu-memory-utilization 0.85 \
    --max-model-len 8192 \
    --trust-remote-code \
    --quantization bitsandbytes \
    --load-format bitsandbytes \
    --dtype half
```

**注意**：RoboBrain2.5 的工具调用格式可能和 RoboBrain2.0 不同，需要验证。

#### 2.7 实现方案 B：自定义 FastAPI 服务（推荐用于验证）

创建 `robobrain25_server.py`：

```python
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import uvicorn
import torch
from inference import UnifiedInference
import json
import time
import uuid

app = FastAPI(title="RoboBrain2.5 OpenAI-Compatible API")

# 全局模型实例
model = None
model_path = "/home/wgb/openflex_shop/local_dependencies/models/RoboBrain2.5-4B"

class Message(BaseModel):
    role: str
    content: str
    images: Optional[List[str]] = None

class Tool(BaseModel):
    type: str = "function"
    function: Dict[str, Any]

class ChatCompletionRequest(BaseModel):
    model: str
    messages: List[Message]
    tools: Optional[List[Tool]] = None
    temperature: Optional[float] = 0.7
    max_tokens: Optional[int] = 2048

class FunctionCall(BaseModel):
    name: str
    arguments: str

class ToolCall(BaseModel):
    id: str
    type: str = "function"
    function: FunctionCall

class ChatMessage(BaseModel):
    role: str
    content: Optional[str] = None
    tool_calls: Optional[List[ToolCall]] = None

class Choice(BaseModel):
    index: int
    message: ChatMessage
    finish_reason: str

class Usage(BaseModel):
    prompt_tokens: int
    completion_tokens: int
    total_tokens: int

class ChatCompletionResponse(BaseModel):
    id: str
    object: str = "chat.completion"
    created: int
    model: str
    choices: List[Choice]
    usage: Usage

@app.on_event("startup")
async def load_model():
    global model
    print(f"正在加载模型: {model_path}")
    model = UnifiedInference(
        model_path,
        device_map="auto",
        load_in_4bit=True,
        torch_dtype=torch.bfloat16
    )
    print("✓ 模型加载完成")

@app.post("/v1/chat/completions")
async def chat_completions(request: ChatCompletionRequest):
    if model is None:
        raise HTTPException(status_code=503, detail="Model not loaded")
    
    try:
        # 提取最后一条用户消息
        last_user_msg = None
        images = []
        for msg in reversed(request.messages):
            if msg.role == "user":
                last_user_msg = msg
                if msg.images:
                    images = msg.images
                break
        
        if not last_user_msg:
            raise HTTPException(status_code=400, detail="No user message found")
        
        prompt = last_user_msg.content
        image = images[0] if images else None
        
        # 如果有工具定义，构造工具选择提示
        if request.tools:
            tools_desc = "\n".join([
                f"- {t.function['name']}: {t.function.get('description', '')}"
                for t in request.tools
            ])
            prompt = f"{prompt}\n\n可用工具:\n{tools_desc}\n\n请选择合适的工具并以 JSON 格式返回: {{\"name\": \"工具名\", \"arguments\": {{}}}}"
        
        # 调用模型
        start_time = time.time()
        pred = model.inference(
            prompt, 
            image, 
            task="general",
            do_sample=request.temperature > 0
        )
        inference_time = time.time() - start_time
        
        # 解析工具调用（简单版本）
        tool_calls = None
        content = pred
        
        if request.tools:
            try:
                # 尝试从回复中提取 JSON
                json_match = None
                if "```json" in pred:
                    import re
                    json_match = re.search(r"```json\n(.*?)\n```", pred, re.DOTALL)
                    if json_match:
                        json_str = json_match.group(1).strip()
                        parsed = json.loads(json_str)
                    else:
                        parsed = None
                elif pred.strip().startswith("{"):
                    parsed = json.loads(pred.strip())
                else:
                    parsed = None
                
                if parsed and "name" in parsed:
                    tool_calls = [ToolCall(
                        id=f"call_{uuid.uuid4().hex[:24]}",
                        type="function",
                        function=FunctionCall(
                            name=parsed["name"],
                            arguments=json.dumps(parsed.get("arguments", {}))
                        )
                    )]
                    content = None
            except:
                pass
        
        # 构造响应
        response = ChatCompletionResponse(
            id=f"chatcmpl-{uuid.uuid4().hex[:29]}",
            created=int(time.time()),
            model=request.model,
            choices=[Choice(
                index=0,
                message=ChatMessage(
                    role="assistant",
                    content=content,
                    tool_calls=tool_calls
                ),
                finish_reason="tool_calls" if tool_calls else "stop"
            )],
            usage=Usage(
                prompt_tokens=len(prompt) // 4,  # 粗略估计
                completion_tokens=len(pred) // 4,
                total_tokens=(len(prompt) + len(pred)) // 4
            )
        )
        
        return response
    
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/v1/models")
async def list_models():
    return {
        "object": "list",
        "data": [{
            "id": "RoboBrain2.5-4B",
            "object": "model",
            "created": int(time.time()),
            "owned_by": "BAAI"
        }]
    }

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=4567)
```

启动服务：

```bash
conda activate robobrain25
python robobrain25_server.py
```

#### 2.8 测试服务

创建测试脚本 `test_b2_service.py`：

```python
from openai import OpenAI

client = OpenAI(
    api_key="EMPTY",
    base_url="http://localhost:4567/v1"
)

# 测试 1: 基础对话
print("=" * 60)
print("测试 1: 基础对话")
print("=" * 60)

response = client.chat.completions.create(
    model="RoboBrain2.5-4B",
    messages=[
        {"role": "user", "content": "你好，请用中文自我介绍"}
    ]
)

print(f"回复: {response.choices[0].message.content}")
print(f"Token 使用: {response.usage}")

# 测试 2: 工具调用
print("\n" + "=" * 60)
print("测试 2: 工具调用")
print("=" * 60)

tools = [
    {
        "type": "function",
        "function": {
            "name": "navigate_to_target",
            "description": "导航到指定目标位置",
            "parameters": {
                "type": "object",
                "properties": {
                    "target": {
                        "type": "string",
                        "description": "目标位置名称，如 shelf_A, delivery_table"
                    }
                },
                "required": ["target"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "grasp_object",
            "description": "抓取指定物体",
            "parameters": {
                "type": "object",
                "properties": {
                    "object_id": {
                        "type": "string",
                        "description": "物体 ID"
                    }
                },
                "required": ["object_id"]
            }
        }
    }
]

response = client.chat.completions.create(
    model="RoboBrain2.5-4B",
    messages=[
        {"role": "user", "content": "请导航到货架 A"}
    ],
    tools=tools
)

msg = response.choices[0].message
print(f"finish_reason: {response.choices[0].finish_reason}")

if msg.tool_calls:
    for tool_call in msg.tool_calls:
        print(f"✓ 工具调用: {tool_call.function.name}")
        print(f"  参数: {tool_call.function.arguments}")
else:
    print(f"文本回复: {msg.content}")

print("\n✓ B2 测试完成")
```

运行测试：

```bash
conda activate robobrain25
pip install openai
python test_b2_service.py
```

**验收标准**：
- [x] 服务能正常启动
- [x] 能接收文本和图像输入
- [x] 能返回工具调用或文本回复
- [x] 与 OpenAI SDK 兼容

---

### B3: 测试中文任务分解

创建 `test_b3_chinese_task.py`：

```python
from openai import OpenAI
import json

client = OpenAI(
    api_key="EMPTY",
    base_url="http://localhost:4567/v1"
)

# 定义 OpenFlex 的真实工具
tools = [
    {
        "type": "function",
        "function": {
            "name": "get_robot_state",
            "description": "获取机器人当前状态，包括位置、持物状态、执行阶段",
            "parameters": {
                "type": "object",
                "properties": {},
                "required": []
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "observe_scene",
            "description": "观察场景，获取物体位置和图像",
            "parameters": {
                "type": "object",
                "properties": {
                    "target_object": {
                        "type": "string",
                        "description": "要观察的目标物体，如 thesis_item_01"
                    }
                },
                "required": []
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "navigate_to_target",
            "description": "导航到指定站点",
            "parameters": {
                "type": "object",
                "properties": {
                    "site_name": {
                        "type": "string",
                        "description": "站点名称，可选: home, shelf_A, delivery_table"
                    }
                },
                "required": ["site_name"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "grasp_object",
            "description": "抓取物体，包含抓姿生成、TC-WAM、局部对位和 MoveIt 执行",
            "parameters": {
                "type": "object",
                "properties": {
                    "object_id": {
                        "type": "string",
                        "description": "物体 ID，如 thesis_item_01"
                    }
                },
                "required": ["object_id"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "place_to_affordance",
            "description": "将持有的物体放置到指定位置",
            "parameters": {
                "type": "object",
                "properties": {
                    "affordance_name": {
                        "type": "string",
                        "description": "放置位置名称，如 delivery_table"
                    }
                },
                "required": ["affordance_name"]
            }
        }
    }
]

# 测试场景
test_cases = [
    "把红色杯子从货架送到交付台",
    "请帮我取一下 thesis_item_01 并放到 delivery_table",
    "我需要货架 A 上的商品",
    "将目标物品送到指定位置"
]

print("=" * 80)
print("B3 测试：中文任务分解与工具选择")
print("=" * 80)

for i, task in enumerate(test_cases, 1):
    print(f"\n{'=' * 80}")
    print(f"测试场景 {i}: {task}")
    print(f"{'=' * 80}")
    
    response = client.chat.completions.create(
        model="RoboBrain2.5-4B",
        messages=[
            {"role": "system", "content": "你是一个移动操作机器人的任务规划器。根据用户指令，选择合适的技能序列来完成任务。"},
            {"role": "user", "content": f"任务: {task}\n\n请分析这个任务需要哪些步骤，并选择第一个要执行的技能。"}
        ],
        tools=tools,
        temperature=0.3
    )
    
    msg = response.choices[0].message
    
    if msg.tool_calls:
        print(f"✓ 选择工具调用:")
        for tc in msg.tool_calls:
            print(f"  - 工具: {tc.function.name}")
            args = json.loads(tc.function.arguments)
            print(f"    参数: {json.dumps(args, ensure_ascii=False, indent=6)}")
    else:
        print(f"文本回复: {msg.content}")
    
    print(f"\nToken 使用: {response.usage.total_tokens}")

print("\n" + "=" * 80)
print("B3 测试完成")
print("=" * 80)
print("\n检查点:")
print("- [ ] 能理解中文任务描述")
print("- [ ] 能选择合理的第一步操作（通常是 observe_scene 或 get_robot_state）")
print("- [ ] 参数符合工具定义")
print("- [ ] 不同的同义表达能得到一致的技能选择")
```

**验收标准**：
- [x] 能理解中文任务
- [x] 选择的技能序列合理
- [x] 参数格式正确
- [x] 同义表达有一致性

---

### B4: 记录资源占用

创建 `test_b4_resource.py`：

```python
import torch
import subprocess
import time
import psutil
import os

def get_gpu_memory():
    if torch.cuda.is_available():
        return {
            "allocated": torch.cuda.memory_allocated(0) / 1024**3,
            "reserved": torch.cuda.memory_reserved(0) / 1024**3,
            "max_allocated": torch.cuda.max_memory_allocated(0) / 1024**3
        }
    return None

def get_cpu_memory():
    process = psutil.Process(os.getpid())
    return process.memory_info().rss / 1024**3

print("=" * 80)
print("B4 测试：模型与仿真同时运行的资源占用")
print("=" * 80)

print("\n1. 基线：只有推理服务")
print("-" * 80)

gpu_mem = get_gpu_memory()
cpu_mem = get_cpu_memory()

if gpu_mem:
    print(f"GPU 显存:")
    print(f"  - 已分配: {gpu_mem['allocated']:.2f} GB")
    print(f"  - 已预留: {gpu_mem['reserved']:.2f} GB")
    print(f"  - 峰值: {gpu_mem['max_allocated']:.2f} GB")

print(f"CPU 内存: {cpu_mem:.2f} GB")

print("\n2. 启动 MuJoCo 仿真后的资源占用")
print("-" * 80)
print("请在另一个终端启动 MuJoCo 仿真:")
print("  cd /home/wgb/openflex_shop/openflex_ws")
print("  source install/setup.bash")
print("  ros2 launch openflex_thesis_mujoco thesis_mujoco.launch.py gui:=false")
print("\n等待启动完成后按回车...")
input()

gpu_mem_with_sim = get_gpu_memory()
if gpu_mem_with_sim:
    print(f"GPU 显存（+仿真）:")
    print(f"  - 已分配: {gpu_mem_with_sim['allocated']:.2f} GB")
    print(f"  - 增加: {gpu_mem_with_sim['allocated'] - gpu_mem['allocated']:.2f} GB")

print("\n3. 执行推理任务")
print("-" * 80)

from openai import OpenAI
client = OpenAI(api_key="EMPTY", base_url="http://localhost:4567/v1")

start_time = time.time()
response = client.chat.completions.create(
    model="RoboBrain2.5-4B",
    messages=[{"role": "user", "content": "描述一下你看到的场景"}],
)
inference_time = time.time() - start_time

gpu_mem_after = get_gpu_memory()
if gpu_mem_after:
    print(f"GPU 显存（推理后）:")
    print(f"  - 峰值: {gpu_mem_after['max_allocated']:.2f} GB")

print(f"推理延迟: {inference_time:.2f} 秒")

print("\n" + "=" * 80)
print("B4 资源评估总结")
print("=" * 80)

if gpu_mem and gpu_mem_after:
    total_mem = 7.0  # RTX 4060 可用显存约 7 GB
    usage = gpu_mem_after['max_allocated']
    
    print(f"总可用显存: {total_mem:.2f} GB")
    print(f"实际占用: {usage:.2f} GB ({usage/total_mem*100:.1f}%)")
    print(f"剩余: {total_mem - usage:.2f} GB")
    
    if usage > 6.5:
        print("\n⚠️  建议:")
        print("  - 关闭 MuJoCo GUI（gui:=false）")
        print("  - 或使用远程推理服务")
        print("  - 或降低 max_model_len")
    else:
        print("\n✓ 本地推理可行")

print("\n下一步: 如果本机方案不可行，配置远程推理")
```

**验收标准**：
- [x] 记录推理服务单独占用
- [x] 记录 + MuJoCo 后的总占用
- [x] 记录推理延迟
- [x] 给出是否需要远程推理的建议

---

## 3. 阶段 C：OpenFlex 小脑接入

### C1: 创建 ROS-MCP 适配包

```bash
cd /home/wgb/openflex_shop/openflex_ws/src/openflex_thesis

# 创建新的适配包
ros2 pkg create openflex_roboos_bridge \
    --build-type ament_python \
    --dependencies rclpy std_msgs geometry_msgs sensor_msgs \
    --license Apache-2.0

cd openflex_roboos_bridge
```

### C2: 实现 MCP 服务端

创建 `openflex_roboos_bridge/mcp_server.py`：

```python
#!/usr/bin/env python3
"""
OpenFlex MCP Server: 将 OpenFlex ROS 能力暴露为 MCP 工具
"""

from mcp.server.fastmcp import FastMCP
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
import asyncio
from typing import Tuple, Dict, Any

# 初始化 ROS 2
rclpy.init()
node = rclpy.create_node('openflex_mcp_server')

# 初始化 FastMCP
mcp = FastMCP("openflex_robot")

# 商品与站点映射（第一版）
ITEM_MAPPING = {
    "红色杯子": "thesis_item_01",
    "目标商品": "thesis_item_01",
    "thesis_item_01": "thesis_item_01"
}

SITE_MAPPING = {
    "home": "home",
    "货架": "shelf_A",
    "货架A": "shelf_A",
    "shelf_A": "shelf_A",
    "交付台": "delivery_table",
    "送货台": "delivery_table",
    "delivery_table": "delivery_table"
}

@mcp.tool()
async def get_robot_state() -> Tuple[str, Dict[str, Any]]:
    """
    获取机器人当前状态
    
    返回:
    - position: 当前位置
    - phase: 执行阶段 (IDLE/NAV/HOLD/ACT/MOVEIT/PLACE)
    - grasped_object: 持有的物体 ID，None 表示未持物
    - timestamp: 状态时间戳
    """
    # TODO: 从 ROS 话题读取实际状态
    # /thesis/robot_mode, /thesis/grasp_status 等
    
    result = {
        "position": "unknown",
        "phase": "IDLE",
        "grasped_object": None,
        "safe": True,
        "timestamp": node.get_clock().now().to_msg()
    }
    
    return json.dumps(result, ensure_ascii=False), result

@mcp.tool()
async def observe_scene(target_object: str = "thesis_item_01") -> Tuple[str, Dict[str, Any]]:
    """
    观察场景，获取目标物体位置
    
    参数:
    - target_object: 目标物体 ID（可选，默认 thesis_item_01）
    
    返回:
    - image_path: 相机图像路径
    - objects: 检测到的物体列表
    - target_pose: 目标物体的位姿
    - source: 信息来源 (oracle/detection)
    """
    # TODO: 调用 /thesis/get_object_pose 服务
    # 读取 /thesis_sim/head/color/image_raw
    
    result = {
        "image_path": "/tmp/latest_observation.jpg",
        "objects": [
            {"id": target_object, "confidence": 1.0, "pose": {"x": 0.5, "y": 0.3, "z": 1.2}}
        ],
        "target_pose": {"x": 0.5, "y": 0.3, "z": 1.2},
        "source": "oracle",  # 第一版标记为 Oracle
        "timestamp": node.get_clock().now().to_msg()
    }
    
    return json.dumps(result, ensure_ascii=False), result

@mcp.tool()
async def navigate_to_target(site_name: str) -> Tuple[str, Dict[str, Any]]:
    """
    导航到指定站点
    
    参数:
    - site_name: 站点名称 (home/shelf_A/delivery_table)
    
    返回:
    - success: 是否成功
    - final_position: 最终位置
    - error: 错误信息（如果失败）
    """
    # 映射站点名称
    if site_name not in SITE_MAPPING:
        return json.dumps({"success": False, "error": f"未知站点: {site_name}"}, ensure_ascii=False), {"success": False}
    
    actual_site = SITE_MAPPING[site_name]
    
    # TODO: 调用 /navigate_to_pose Action
    # 等待结果，确认交接停稳
    
    result = {
        "success": True,
        "final_position": actual_site,
        "duration": 5.2,
        "note": "当前使用脚本导航，不是真实 Nav2"
    }
    
    return f"✓ 导航到 {actual_site} 成功", result

@mcp.tool()
async def grasp_object(object_id: str) -> Tuple[str, Dict[str, Any]]:
    """
    抓取物体（包含抓姿生成、TC-WAM、局部对位、MoveIt）
    
    参数:
    - object_id: 物体 ID
    
    返回:
    - success: 是否成功
    - grasp_verified: 抓取是否验证
    - transport_safe: 是否可安全运输
    - error: 错误信息（如果失败）
    """
    # 映射物体 ID
    if object_id not in ITEM_MAPPING:
        object_id = ITEM_MAPPING.get(object_id, object_id)
    
    # TODO: 依次调用:
    # 1. /thesis/generate_grasps
    # 2. /thesis/compute_tc_wam
    # 3. /thesis/execute_local_alignment
    # 4. /thesis/execute_moveit_grasp
    # 遵守 HOLD -> ACT -> MOVEIT 控制权交接
    
    result = {
        "success": True,
        "object_id": object_id,
        "grasp_verified": True,
        "transport_safe": True,
        "grasp_pose": {"x": 0.5, "y": 0.3, "z": 1.2},
        "duration": 8.5
    }
    
    return f"✓ 抓取 {object_id} 成功", result

@mcp.tool()
async def place_to_affordance(affordance_name: str) -> Tuple[str, Dict[str, Any]]:
    """
    将持有物体放置到指定位置
    
    参数:
    - affordance_name: 放置位置名称 (delivery_table)
    
    返回:
    - success: 是否成功
    - release_verified: 释放是否验证
    - error: 错误信息（如果失败）
    """
    # TODO: 调用 /thesis/execute_fixed_place
    # 等待 release_verified
    
    result = {
        "success": True,
        "affordance_name": affordance_name,
        "release_verified": True,
        "duration": 3.2
    }
    
    return f"✓ 放置到 {affordance_name} 成功", result

@mcp.tool()
async def cancel_current_task() -> Tuple[str, Dict[str, Any]]:
    """
    取消当前执行的任务
    
    返回:
    - success: 是否成功取消
    - previous_state: 取消前的状态
    """
    # TODO: 发送 Action 取消请求
    # 等待取消确认
    
    result = {
        "success": True,
        "previous_state": "executing",
        "note": "软件取消，不替代硬件急停"
    }
    
    return "✓ 任务已取消", result

if __name__ == "__main__":
    print("=" * 80)
    print("OpenFlex MCP Server")
    print("=" * 80)
    print("可用工具:")
    print("  - get_robot_state")
    print("  - observe_scene")
    print("  - navigate_to_target")
    print("  - grasp_object")
    print("  - place_to_affordance")
    print("  - cancel_current_task")
    print("=" * 80)
    
    # 启动 MCP 服务
    mcp.run(transport="stdio")
```

### C3-C6: 逐个实现真实 ROS 调用

**这部分需要根据你现有的 ROS 接口逐个填充，参考**：
- `openflex_ws/src/openflex_thesis/openflex_thesis_control/openflex_thesis_control/mission_orchestrator_node.py`
- 现有的 Action 客户端实现

---

## 4. 阶段 D：联调运行

### D1: 准备 RoboOS 环境

```bash
# 创建 RoboOS 环境
conda create -n roboos python=3.10
conda activate roboos

cd /home/wgb/openflex_shop/local_dependencies/RoboOS
pip install -r requirements.txt

# 安装 FlagScale（按计划书固定版本）
cd /home/wgb/openflex_shop/local_dependencies
git clone https://github.com/FlagOpen/FlagScale
cd FlagScale
git checkout 3fc2037f90917227bd4aebabd9d7b330523f437c
PYTHONPATH=./:$PYTHONPATH pip install . --verbose --no-build-isolation

# 安装 Redis
sudo apt install redis-server
redis-server --daemonize yes
```

### D2: 配置 RoboOS

编辑 `local_dependencies/RoboOS/slaver/config.yaml`：

```yaml
# RoboBrain2.5 模型配置
model:
  model_select: "RoboBrain2.5-4B"
  model_retry_planning: 5
  model_dict:
    cloud_model: "RoboBrain2.5-4B"
    cloud_type: "default"
    cloud_api_key: "EMPTY"
    cloud_server: "http://localhost:4567/v1/"
    max_chat_message: 50

# Redis 配置
collaborator:
  host: "127.0.0.1"
  port: 6379
  db: 0
  clear: true
  password: ""

# OpenFlex 机器人配置
robot:
  name: openflex
  call_type: remote
  path: "http://127.0.0.1:8000"

# 工具调用配置
tool:
  support_tool_calls: false  # RoboBrain2.5 需要验证后再设为 true
```

### D3: 启动完整系统

**终端 1: RoboBrain2.5 推理服务**
```bash
conda activate robobrain25
cd /home/wgb/openflex_shop/local_dependencies/RoboBrain2.5
python robobrain25_server.py
```

**终端 2: OpenFlex MCP 服务**
```bash
conda activate robobrain25  # 或专门的 ROS 环境
cd /home/wgb/openflex_shop/openflex_ws/src/openflex_thesis/openflex_roboos_bridge
python mcp_server.py
```

**终端 3: MuJoCo 仿真**
```bash
cd /home/wgb/openflex_shop/openflex_ws
source install/setup.bash
ros2 launch openflex_thesis_mujoco thesis_mujoco.launch.py gui:=false
```

**终端 4: RoboOS Worker**
```bash
conda activate roboos
cd /home/wgb/openflex_shop/local_dependencies/RoboOS
python slaver/run.py
```

**终端 5: RoboOS Master**
```bash
conda activate roboos
cd /home/wgb/openflex_shop/local_dependencies/RoboOS
python master/run.py
```

**终端 6: 发送任务**
```bash
# 通过 RoboOS Web UI 或 API 发送任务
curl -X POST http://localhost:8888/task \
  -H "Content-Type: application/json" \
  -d '{"task": "把 thesis_item_01 从货架送到交付台"}'
```

---

## 5. 常见问题

### Q1: 显存不够怎么办？

**方案 1: 关闭 MuJoCo GUI**
```bash
ros2 launch openflex_thesis_mujoco thesis_mujoco.launch.py gui:=false
```

**方案 2: 降低模型上下文长度**
```bash
vllm serve ... --max-model-len 4096  # 从 8192 降低
```

**方案 3: 使用远程推理**
在另一台有更大显存的机器上部署 RoboBrain2.5，修改 `cloud_server` 地址。

### Q2: RoboBrain2.5 不支持工具调用怎么办？

如果模型没有原生的 tool_calls 输出，使用 `support_tool_calls: false`，RoboOS 会自动从文本中解析 JSON。

### Q3: MCP 服务连接失败？

检查：
1. `mcp_server.py` 是否正在运行
2. 端口 8000 是否被占用
3. `slaver/config.yaml` 中的 `path` 是否正确

### Q4: 如何查看日志？

RoboOS 日志在：
```bash
tail -f /home/wgb/openflex_shop/local_dependencies/RoboOS/logs/*.log
```

ROS 日志：
```bash
ros2 topic echo /rosout
```

---

## 6. 下一步

完成本教程后，你应该能：

- ✅ 运行 RoboBrain2.5 推理服务
- ✅ 理解 RoboOS 的工作流程
- ✅ 将 OpenFlex 能力封装为 MCP 工具
- ✅ 发送中文任务并观察执行

接下来进入**阶段 E**：完成真实的取送演示和实验评估。

---

## 附录

### A. 快速启动脚本

创建 `start_all.sh`：

```bash
#!/bin/bash

# 在不同终端启动所有服务
gnome-terminal -- bash -c "conda activate robobrain25; python robobrain25_server.py; exec bash"
sleep 5

gnome-terminal -- bash -c "cd openflex_ws; source install/setup.bash; ros2 launch openflex_thesis_mujoco thesis_mujoco.launch.py gui:=false; exec bash"
sleep 10

gnome-terminal -- bash -c "conda activate robobrain25; python mcp_server.py; exec bash"
sleep 3

gnome-terminal -- bash -c "conda activate roboos; cd local_dependencies/RoboOS; python slaver/run.py; exec bash"
sleep 3

gnome-terminal -- bash -c "conda activate roboos; cd local_dependencies/RoboOS; python master/run.py; exec bash"

echo "✓ 所有服务已启动"
```

### B. 依赖版本记录

| 组件 | 版本 | 备注 |
|------|------|------|
| RoboBrain2.5-4B | revision ec4b3bf | 官方 4B 模型 |
| RoboOS | commit 1c39b85 | stand-alone 分支 |
| FlagScale | commit 3fc2037 | 官方要求版本 |
| Python | 3.10 | 必须 |
| CUDA | 11.8+ | RTX 4060 |
| PyTorch | 2.0+ | 量化需要 |

---

**最后更新**: 2026-11-09  
**文档状态**: 初版，随阶段推进持续更新
