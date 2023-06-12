from pycarmaker import CarMaker, Quantity
import time 
import math

# 1 - Open CarMaker with option -cmdport
'''
    For example: on a Windows system with CarMaker 8.0.2 installed on the default
    folder send the command C:\IPG\carmaker\win64-11.1.2\bin\CM.exe -cmdport 16660
'''
# 3 - Initialize pyCarMaker
IP_ADDRESS = "localhost"
PORT = 16660
cm = CarMaker(IP_ADDRESS, PORT)

# 4 - Connect to CarMaker
cm.connect()

# 5 - Subscribe to vehicle speed
# Create a Quantity instance for vehicle speed (vehicle speed is a float type variable)
segments_num = 200
segments_x_quan = []
segments_y_quan = []
segments_type_quan = []

UAQ_name = 'Sensor.FSpace.Front.Segm.'
camera_name = 'Sensor.Camera.VehSensor_0.Obj.'
for i in range(segments_num):
    x_name = UAQ_name + str(i) + '.ds.x'
    y_name = UAQ_name + str(i) + '.ds.y'
    segments_x_quan.append(Quantity(x_name,Quantity.FLOAT))
    segments_y_quan.append(Quantity(y_name,Quantity.FLOAT))

for i in range(5):
    segments_type_quan.append(Quantity(camera_name+str(i)+'.Type',Quantity.FLOAT))
    

time_quan = Quantity('Time',Quantity.FLOAT)
ego_car_x = Quantity('Vhcl.Fr1.x',Quantity.FLOAT)
ego_car_y = Quantity('Vhcl.Fr1.y',Quantity.FLOAT)
ego_car_z = Quantity('Vhcl.Fr1.z',Quantity.FLOAT)
ego_car_v_x = Quantity('Car.Aero.vres_1.x',Quantity.FLOAT)

# Initialize with negative speed to indicate that value was not read
for i in range(segments_num):
    segments_x_quan[i].data = -1
    segments_y_quan[i].data = -1

for i in range(5):
    segments_type_quan[i].data = -1

time_quan.data = -1
ego_car_x.data = -1
ego_car_y.data = -1
ego_car_z.data = -1
ego_car_v_x.data = -1

# Subscribe (TCP socket need to be connected)

#从carmaker中读取参数包括freespace生成的200的点的x，y；自车x，y，z；时间戳
for i in range(segments_num):
    cm.subscribe(segments_x_quan[i])
    cm.subscribe(segments_y_quan[i])

for i in range(5):
    cm.subscribe(segments_type_quan[i])

cm.subscribe(time_quan)
cm.subscribe(ego_car_x)
cm.subscribe(ego_car_y)
cm.subscribe(ego_car_z)
cm.subscribe(ego_car_v_x)

# 6 - Read all subscribed quantities. In this example, vehicle speed and simulation status
# For some reason, the first two reads will be incomplete and must be ignored
# You will see 2 log errors like this: [ ERROR]   CarMaker: Wrong read
cm.read()
cm.read()
time.sleep(0.1)

time_gap = 0.2
sample_time = 0.2
c = sample_time/time_gap

#从传感器读取数据中计算出障碍物的宽度，长度，以及障碍物中心点距离自车的x，y
obs_width = 0
obs_width_list = []
obs_length = 0
obs_length_list = []
obs2car_x = 0
obs2car_y = 0
obs_class = ''
obs_length_init = 5

while( 1 ):
    #c = c - 1
    # Read data from carmaker
    cm.read()
    segments_x_data = []
    segments_y_data = []

    for i in range(segments_num):
        if segments_x_quan[i].data != 0:
            segments_x_data.append(segments_x_quan[i].data)
        if segments_y_quan[i].data != 0:
            segments_y_data.append(segments_y_quan[i].data)

    #检测物体种类
    for i in range(5):
        if segments_type_quan[i].data == 0 :
            obs_class = 'car'
            obs_length_init = 5
        elif segments_type_quan[i].data == 1:
            obs_class = 'truck'
            obs_length_init = 10


    #未检测到物体时直接跳出循环
    if len(segments_x_data) == 0 and len(segments_y_data) == 0:
        print(time_quan.data)
        print('x=',ego_car_x.data)
        print('y=',ego_car_y.data)
        print('z=',ego_car_z.data)
        print("未检测到障碍物")
        time.sleep(time_gap)
        continue

    #判断是否检测到障碍物侧面
    if max(segments_x_data) - min(segments_x_data) < 0.1:
        #计算障碍物横向偏移
        obs2car_y = (max(segments_y_data) + min(segments_y_data))/2
        #计算障碍物宽度
        obs_width = abs(max(segments_y_data) - min(segments_y_data))
        #障碍物长度无法计算得到，设为预设长度
        obs_length = obs_length_init
        #计算障碍物纵向偏移
        obs2car_x = (max(segments_x_data) + min(segments_x_data))/2 + obs_length/2
    
    #判断是否检测到障碍物正面和侧面
    if max(segments_x_data) - min(segments_x_data) >1 and abs(max(segments_y_data)-min(segments_y_data)) > 1:
        #计算障碍物纵向偏移
        obs2car_x = (max(segments_x_data) + min(segments_x_data))/2
        #计算障碍物长度
        obs_length = max(segments_x_data) - min(segments_x_data)
        obs_length_list.append(obs_length)
        #计算障碍物横向偏移
        obs2car_y = (max(segments_y_data) + min(segments_y_data))/2
        #计算障碍物宽度
        obs_width = abs(max(segments_y_data) - min(segments_y_data))
        obs_width_list.append(obs_width)

    #判断是否检测到障碍物正面
    if abs(max(segments_y_data) - min(segments_y_data)) < 0.1 :
        #检测不到障碍物正面表明自车正在绕过障碍物，此时认为障碍物长度、宽度即为相应列表中存储的最大值
        #障碍物长度
        obs_length = max(obs_length_list)
        #障碍物宽度
        obs_width = max(obs_width_list)
        #计算障碍物纵向偏移
        obs2car_x = max(segments_x_data)  - obs_length/2
        #计算障碍物横向偏移
        obs2car_y = sum(segments_y_data)/len(segments_y_data)
        obs2car_y = obs2car_y + math.copysign(1,obs2car_y) * obs_width/2

    #纵向偏移是指freespace安装位置距离障碍物横对称线的距离
    #横向偏移是指freespace安装位置距离障碍物竖对称线的距离

    read_freespace = []
    read_freespace.append(time_quan.data)       #时间戳
    read_freespace.append(ego_car_x.data)       #自车x坐标
    read_freespace.append(ego_car_y.data)       #自车y坐标
    read_freespace.append(ego_car_z.data)       #自车z坐标
    read_freespace.append(ego_car_v_x.data)     #自车x速度
    read_freespace.append(obs_length)           #障碍物长度
    read_freespace.append(obs_width)            #障碍物宽度
    read_freespace.append(obs2car_y)            #障碍物横向偏移
    read_freespace.append(obs2car_x)            #障碍物纵向偏移

    print()
    print(time_quan.data)
    print('x=',ego_car_x.data)
    print('y=',ego_car_y.data)
    print('z=',ego_car_z.data)
    print('v=',ego_car_v_x.data)
    print('障碍物长度：',obs_length)
    print('障碍物宽度：',obs_width)
    print('障碍物横向偏移：',obs2car_y)
    print('障碍物纵向偏移：',obs2car_x)
    
    time.sleep(time_gap)
