import numpy as np
v_mean = 50
v_std = 5
print(float(np.clip(np.random.normal(v_mean, v_std), 10, 50)))