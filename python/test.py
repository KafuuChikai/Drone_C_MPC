import numpy as np
from drone_model_simple import DroneModel
# xs=np.array([1, 1, -5])
# print(xs[3:9])
# xs_between = np.concatenate((xs[0:3], xs[3:9], np.zeros(3)))
# print(xs_between)

total = [0,0,-5,0,0,0,1,0,0,0]
t=0.01
total = total + t * ca.vcat(model.f(total,[-4.9,0.0005,0,0]))
total[6:10] = total[6:10]/np.linalg.norm(total[6:10])