import torch

model_path = "/home/spei/flow/flow/controllers/model_simple.pth"  # 替换为你的路径
state = torch.load(model_path, map_location=torch.device('cpu'))


# 查看这个字典中有哪些键
print("模型文件包含的键：", state.keys())

for k, v in state['net'].items():
    print(k, v.shape)

