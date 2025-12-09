import torch
import torch.nn as nn
import numpy as np

class SimpleFFN(nn.Module):
    def __init__(self, input_dim=11, output_dim=4):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, 64),
            nn.Tanh(),
            nn.Linear(64, 64),
            nn.Tanh(),
            nn.Linear(64, output_dim)
        )

    def forward(self, x):
        return self.net(x)
        # output=self.net(x)
        # mu, log_std = output[0], output[1]
        # std = torch.exp(log_std)
        # accel = torch.normal(mu, std).item()
        # lane_logits = output[2:4]
        # dist = torch.distributions.Categorical(logits=lane_logits)
        # lane_change = dist.sample().item()    # 返回 0 或 1
        # return {'accel': accel, 'lane_change': lane_change}
