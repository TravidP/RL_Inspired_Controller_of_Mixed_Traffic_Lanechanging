import torch

# 原始模型文件路径
model_path = "/home/spei/flow/flow/controllers/model-80.pth"
save_path = "/home/spei/flow/flow/controllers/model_simple.pth"

# 载入原始模型
state_dict = torch.load(model_path, map_location="cpu")
if "net" in state_dict:
    state_dict = state_dict["net"]  # 只取网络部分

# 创建新的 state_dict 映射
new_state_dict = {}
for key, value in state_dict.items():
    if key.startswith("p_head."):
        new_key = key.replace("p_head.", "net.")
        new_state_dict[new_key] = value
    else:
        print(f"跳过未使用 key: {key}")

# 保存为新文件
torch.save(new_state_dict, save_path)
print(f"保存新模型到: {save_path}")
