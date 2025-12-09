import torch
import torch.nn as nn
from types import SimpleNamespace
 # Import setdefaults
#from flow.controllers.utils import setdefaults
def args_from(layers, default):
    # 仅返回整数层大小，排除激活函数字符串
    return [default] + [x for x in layers if isinstance(x, int)]
class FFN(nn.Module):
    def __init__(self, c):
        super().__init__()
        self.c = setdefaults(
            c,
            layers=[64, 'tanh', 64, 'tanh'],
            weight_scale='default',
            weight_init='orthogonal'
        )
        s_sizes = [c.observation_space.shape[0]] + c.layers[::2]
        self.shared = build_fc(*args_from(c.layers, s_sizes[0]))
        self.p_head = build_fc(s_sizes[-1], *args_from(c.layers, c.model_output_size))
        if c.use_critic:
            self.v_head = build_fc(s_sizes[-1], *args_from(c.layers, 1))
    def forward(self, inp, value=False, policy=False, argmax=None):
        s = self.shared(inp)
        pred = SimpleNamespace()
        if value and self.c.use_critic:
            pred.value = self.v_head(s)
        if policy or argmax is not None:
            pred.policy = self.p_head(s)
            if argmax is not None:
                dist = self.c.dist_class(pred.policy)
                pred.action = dist.argmax() if argmax else dist.sample()
        return pred

def build_fc(*args):
    layers = []
    for i, o in zip(args[:-1], args[1:]):
        if isinstance(o, int):
            layers.append(nn.Linear(i, o))
        else:
            layers.append(dict(relu=nn.ReLU, tanh=nn.Tanh)[o]())
    return nn.Sequential(*layers)