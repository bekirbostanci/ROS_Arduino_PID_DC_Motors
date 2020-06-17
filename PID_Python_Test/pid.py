import matplotlib.pyplot as plt
import random

#   m/s
#   |           slope = 1
#   |           /
# 10|          /
#   |         /
#   |________/_________ pwm
#           50   60



kp = 0.3
ki = 1
kd = 0.0

integral_ = 0
prev_error_ = 0
error = 0
list=[]
list_time=[]
list_noise=[]


setpoint = 10
measured_value = 0
for i in range(100):
    error = setpoint - measured_value
    integral_ += error
    derivative_ = error - prev_error_

    if (setpoint == 0 and error == 0):
        integral_ = 0

    pid = (kp * error) + (ki * integral_) + (kd * derivative_)+setpoint

    noise = 0
    if i%10 ==0 :
        noise = random.uniform(-1,1)

    if pid<= 50:
        measured_value = 0
    else:
        measured_value = pid - 50 + (noise)

    prev_error_ = error

    list.append(pid)
    list_time.append(i)
    list_noise.append(noise)



fig, ax = plt.subplots()
ax.plot( list_time,list)
ax.plot( list_time,list_noise)

plt.grid()
plt.xlim(0, 110)
plt.ylim(-5, 100)
ax.set(xlabel='time (s)', ylabel='voltage (mV)',
       title='pid controller')


fig.savefig("test.png")
plt.show()

