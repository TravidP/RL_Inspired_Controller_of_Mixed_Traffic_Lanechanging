import torch
import numpy as np
from flow.controllers.ut import *
# 加载模型（假设你已经定义了 FFN 类并导入）
model = FFN()
model.load_state_dict(torch.load("/home/spei/flow/flow/controllers/model-80.pth", map_location='cpu'))
model.eval()

# 创建一个假输入 observation（根据训练时 obs 的 shape）
dummy_obs = torch.randn(1, obs_dim)  # obs_dim 是你训练时观测向量的维度

# 获取模型输出
with torch.no_grad():
    action = model(dummy_obs)

print("模型输出：", action)
