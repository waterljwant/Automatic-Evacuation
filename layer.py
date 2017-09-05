'''
R为机器人的转弯半径r1+自身外切圆半径r2
d为激光雷达安装位置距离机器人前端的距离
w为机器人宽度
layerNumber 划分层的数目
speed 最大速度
angular_speed 角速度
(speed / angular_speed) 作为调和因子
Fi = (speed / angular_speed) * R * sin(ODF) / sin(OFD)
'''
#!/usr/bin/python
import math


d = 0.3 	#unit: m, 激光雷达安装位置距离机器人前端的距离
w = 0.48		#unit: m, 小车宽度
length = 0.72	#unit: m, 小车的长度

r1 = 0.0 	#unit: m, 小车的转弯半径
r2 = 0.5*math.sqrt(w * w + length * length) 	#unit: m, 小车的外切圆半径
R = (r1+r2)
speed = 3.5 	#unit: m/s
angular_speed = 1/4.5 * 2 * math.pi #unit: rad/s, 4.5s转一圈
print(r2)
print('\n')

layerNumber = 5 #划分为5等分，共四个距离

theta_MON = math.asin(d/R)  #unit: rad
#print('OMN',theta_MON*180/math.pi)

F = [0 for x in range(0, layerNumber-1)]

for i in range(0,layerNumber-1):
	theta_MOD = theta_MON/layerNumber * (i+1)
	#theta_ODF = math.pi - (0.5*math.pi - theta_MON) # in old method
	theta_ODF = math.pi - (0.5*math.pi - theta_MON - (0.5*math.pi-theta_MON)/layerNumber*(i+1) + theta_MON/layerNumber*(i+1))#in new method
	theta_OFD = math.pi - theta_ODF - theta_MOD # theta_FOD = theta_MOD
	F[i] = (speed/angular_speed) * R * math.sin(theta_ODF) / math.sin(theta_OFD)
	print('ODF',theta_ODF*180/math.pi)
	print('OFD',theta_OFD*180/math.pi)

print(F)