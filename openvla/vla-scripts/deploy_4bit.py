import json_numpy
json_numpy.patch()
import torch
import uvicorn
import numpy as np
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse, HTMLResponse
from PIL import Image
from transformers import AutoModelForVision2Seq, AutoProcessor, BitsAndBytesConfig
import traceback
import base64
from io import BytesIO

# 全局变量缓存最新图像（用于网页实时显示）
latest_image = None
latest_action = None

class OpenVLA4bitServer:
    def __init__(self, openvla_path="openvla/openvla-7b"):
        self.openvla_path = openvla_path

        # 4bit 量化配置
        self.quantization_config = BitsAndBytesConfig(
            load_in_4bit=True,
            bnb_4bit_quant_type="nf4",
            bnb_4bit_compute_dtype=torch.float16,
            bnb_4bit_use_double_quant=True,
        )

        self.processor = AutoProcessor.from_pretrained(
            self.openvla_path,
            trust_remote_code=True
        )

        print("Loading 4-bit OpenVLA model...")
        self.vla = AutoModelForVision2Seq.from_pretrained(
            self.openvla_path,
            quantization_config=self.quantization_config,
            low_cpu_mem_usage=True,
            trust_remote_code=True,
            torch_dtype=torch.float16,
            device_map="auto",
        )
        self.vla.tie_weights()
        print("✅ Model loaded successfully!")

    # ===================== 实时监控网页（带摄像头画面）=====================
    async def index_page(self):
        return HTMLResponse("""
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>OpenVLA 实时监控</title>
    <style>
        body { background: #1a1a1a; color: white; font-family: Arial; padding: 20px; }
        .container { max-width: 900px; margin: 0 auto; }
        .video-frame {
            width: 640px;
            height: 480px;
            border: 3px solid #4CAF50;
            background: #000;
            margin: 10px auto;
            display: block;
        }
        .action {
            background: #222; padding: 10px; border-radius: 8px; margin-top: 15px;
            font-family: monospace; font-size: 14px;
        }
        h2 { text-align: center; color: #4CAF50; }
    </style>
</head>
<body>
    <div class="container">
        <h2>🤖 OpenVLA 实时摄像头监控</h2>
        <img id="cameraFeed" class="video-frame" src="">
        <div class="action">当前动作: <span id="actionText">等待数据...</span></div>
    </div>

    <script>
        // 实时刷新画面
        function updateCamera() {
            fetch('/latest-image')
                .then(res => res.json())
                .then(data => {
                    if (data.image) {
                        document.getElementById('cameraFeed').src = 'data:image/jpeg;base64,' + data.image;
                    }
                    if (data.action) {
                        document.getElementById('actionText').innerText = data.action;
                    }
                })
                .catch(e => console.log(e));
            setTimeout(updateCamera, 100); // 100ms 刷新一次
        }
        updateCamera();
    </script>
</body>
</html>
        """)

    # ===================== 获取最新画面接口 =====================
    async def get_latest_image(self):
        global latest_image, latest_action
        img_b64 = ""
        if latest_image is not None:
            im = Image.fromarray(latest_image)
            buf = BytesIO()
            im.save(buf, format='JPEG')
            img_b64 = base64.b64encode(buf.getvalue()).decode('utf-8')
        return JSONResponse({"image": img_b64, "action": str(latest_action)})

    # ===================== 核心预测接口 =====================
    async def predict_action(self, request: Request):
        global latest_image, latest_action
        try:
            payload = await request.json()
            image = np.array(payload["image"], dtype=np.uint8)
            instruction = payload["instruction"]
            unnorm_key = "bridge_orig"

            # 保存最新图像（给网页显示）
            latest_image = image.copy()

            # 模型推理
            img = Image.fromarray(image).convert("RGB")
            prompt = f"In: What action should the robot take to {instruction.lower()}?\nOut:"
            inputs = self.processor(prompt, img, return_tensors="pt")
            inputs = inputs.to(device=self.vla.device, dtype=self.vla.dtype)

            with torch.no_grad():
                action = self.vla.predict_action(**inputs, unnorm_key=unnorm_key, do_sample=False)
            
            latest_action = action.tolist()
            return JSONResponse({"status": "success", "action": latest_action})

        except Exception as e:
            traceback.print_exc()
            return JSONResponse({"status": "error", "message": str(e)})

    def run(self, host="0.0.0.0", port=8000):
        self.app = FastAPI()
        self.app.get("/")(self.index_page)                  # 监控主页
        self.app.get("/latest-image")(self.get_latest_image)# 实时图像
        self.app.post("/act")(self.predict_action)          # 机械臂控制
        uvicorn.run(self.app, host=host, port=port)

if __name__ == "__main__":
    server = OpenVLA4bitServer()
    server.run(host="127.0.0.1", port=8000)