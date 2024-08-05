----
### 编译

依赖：
[Eclipse Paho MQTT C++ Client Library](https://github.com/eclipse/paho.mqtt.cpp)
[Eigen3](https://eigen.tuxfamily.org/)
ubuntu 安装依赖
> sudo apt install libpaho-mqttpp-dev
>  (ubuntu18.04及以下可能需要源码编译，20.04以上可以apt安装，windows可以源码编译或者用vcpkg)

> sudo apt install libeigen3-dev

> paho-mqttpp依赖eclipse-paho-mqtt-c和openssl,一般会自动安装


如果要测试vda,添加test选项编译，编译车辆端模拟的代码和一些测试代码,如果使用虚拟车辆则无须编译test


```bash
mkdir build
cd build
mkdir install
cmake -DTEST=ON -DCMAKE_INSTALL_PREFIX=./install .. 
make install -j4
```


### 配置MQTT代理服务器(如果使用vda车辆)
我用的 **Eclipse Mosquitto**
安装后配置端口和ip
默认配置文件路径在 **/etc/mosquitto/mosquitto.conf**
添加或修改对应配置
```
listener 1883 0.0.0.0
allow_anonymous true
```
默认的日志路径 **/var/log/mosquitto/mosquitto.log**
用MQTTX测试一下确定连接通畅
### 运行服务端
```
cd ./install/RobotSchedulingSystem
ls
```
确保当前路径下存在config目录
运行 rss文件
```
./rss
```
正常的输出
```
2024-04-30 11:30:24,808 INFO [rss] read param success
[I 240430 11:30:24.8092 httpsrv.cc:295] listen 0.0.0.0:8080
[I 240430 11:30:25.0094 main.cc:228] service start, press 'Ctrl + C' to exit.
```
此时需要上传模型，否则其他请求都会返回失败


### 使用网页查看
用浏览器打开config中的view.html
确认ip和端口

初始界面如下


![view](imgs/init.png)

##### 如果已经上传模型，或者调度系统启动时配置了自动加载某个文件，可以直接点击确定，此时会显示车辆列表和地图

> <big>**<font color=#b04f61> 在地图上滚轮可以放大缩小地图，右键按住拖动可以地图,鼠标停在Point 或 Location 或 Vehicle上面可以大字号显示</font>**</big>

粗圆环表示停靠点；双线圆环表示充电点；被禁用的道路和操作点会显示灰色
带颜色的路径和点表示不同的block区域；正方形方框表示周边操作点
![view](imgs/map.png)

### 如果未上传模型文件，或者需要重新上传更改后的模型文件，点击选择模型文件
直接选择config中的map.xml
不同的tcs:preferredAdapterClass对应不同的车辆模型
使用虚拟车辆注意修改初始点和停靠点
```xml
    <vehicle name="Vehicle-0005" length="1000" energyLevelCritical="30" energyLevelGood="90"
        energyLevelFullyRecharged="30" energyLevelSufficientlyRecharged="90" maxVelocity="1000"
        maxReverseVelocity="1000">
        <property name="tcs:preferredAdapterClass"
            value="org.opentcs.virtualvehicle.LoopbackCommunicationAdapterFactory" />
        <property name="loopback:initialPosition" value="P7" />
        <property name="tcs:preferredParkingPosition" value="P7" />
        <vehicleLayout color="#FF00FF" />
    </vehicle>
```
使用vda车辆注意修改车辆属性中的MQTT代理的ip(暂定)
```xml
 <vehicle name="Vehicle-0004" length="1000" energyLevelCritical="30" energyLevelGood="90"
        energyLevelFullyRecharged="30" energyLevelSufficientlyRecharged="90" maxVelocity="1000"
        maxReverseVelocity="1000">
        <property name="tcs:preferredAdapterClass"
            value="org.opentcs.virtualvehicle.LoopbackCommunicationAdapterFactory" />
        <property name="vda5050:interfaceName" value="uagv" />
        <property name="vda5050:manufacturer" value="rw" />
        <property name="vda5050:minVisualizationInterval" value="1000" />
        <property name="vda5050:orderQueueSize" value="2" />
        <property name="vda5050:serialNumber" value="tx4" />
        <property name="vda5050:version" value="2.0" />
        <property name="vda5050:ip" value="192.168.0.39" />
        <property name="vda5050:port" value="1883" />
        <vehicleLayout color="#FFFF00" />
    </vehicle>
```

![view](imgs/upload.png)

#### 也可以使用api工具发送http请求
config中有api接口定义文件
> RSS.openapi.json

发送上传模型请求：
路由 /plantModel
参数 type 设置为xml，如果要上传json文件，设置type为json或不设置该参数
body config中的map.xml内容



操作成功服务器会输出
```
[I 240430 11:33:47.4365 rss.cc:627] init orderpool ok
[I 240430 11:33:47.4365 rss.cc:632] init scheduler ok
[I 240430 11:33:47.4365 rss.cc:622] init dispatcher ok
[I 240430 11:33:47.4369 rss.cc:84] init point size 93
[I 240430 11:33:47.4371 rss.cc:152] init loc_type size 3
[I 240430 11:33:47.4373 rss.cc:236] init location size 91
[I 240430 11:33:47.4486 rss.cc:336] init path size 98
[I 240430 11:33:47.4490 rss.cc:500] init vehicle size 4
[I 240430 11:33:47.4490 rss.cc:504] init resource ok
[I 240430 11:33:47.4495 timer.hpp:25] generate map use time: 0(s) 0(ms) 487(us)
[I 240430 11:33:47.4495 rss.cc:640] init planner ok
[I 240430 11:33:47.4495 rss.cc:523] run all ...
[I 240430 11:33:47.4508 dispatch.cc:279] Dispatcher run....
[I 240430 11:33:47.4608 schedule.cc:10] Scheduler run....
[I 240430 11:33:47.4609 rss.cc:525] run all ok
[I 240430 11:33:47.5507 master.cc:47] mqtt_serial_number:tx2 master ONLINE
[I 240430 11:33:47.5508 master.cc:47] mqtt_serial_number:tx1 master ONLINE
[I 240430 11:33:47.5508 master.cc:47] mqtt_serial_number:tx3 master ONLINE
[I 240430 11:33:47.5508 master.cc:47] mqtt_serial_number:tx4 master ONLINE
[W 240430 11:33:47.5911 vehicle.cc:666] Vehicle-0002 tx2 CONNECTIONBROKEN
[W 240430 11:33:47.5912 vehicle.cc:666] Vehicle-0001 tx1 CONNECTIONBROKEN
[W 240430 11:33:47.5950 vehicle.cc:666] Vehicle-0003 tx3 CONNECTIONBROKEN
[W 240430 11:33:47.5951 vehicle.cc:666] Vehicle-0004 tx4 CONNECTIONBROKEN
```
表示解析模型成功，服务资源初始化成功，服务器连接MQTT代理成功，由于接收到车辆上次的遗嘱消息或者状态未更新,
车辆状态为broken,然后等待车辆连接

可以多次重新上传模型，资源会清理然后重新创建

上传模型文件内容格式示例：
- Point 路径点
```xml
 <point name="Point-0091" xPosition="77250" yPosition="-70950" zPosition="0"
        vehicleOrientationAngle="NaN" type="HALT_POSITION">
        <outgoingPath name="Point-0091 --- Point-0005" />
        <pointLayout xPosition="77250" yPosition="-70950" xLabelOffset="-10" yLabelOffset="-20"
            layerId="0" />
 </point>
```
- Path 路径点组成的路线
```xml
 <path name="Point-0027 --- Point-0036" sourcePoint="Point-0027" destinationPoint="Point-0036"
        length="10351" maxVelocity="1000" maxReverseVelocity="1000" locked="false">
        <pathLayout connectionType="DIRECT" layerId="0" />
        <property name="vda5050:action.01" value="Door" />
        <property name="vda5050:action.01.blockingType" value="SOFT" />
        <property name="vda5050:action.01.parameter.script" value="robot_pose" />
        <property name="vda5050:action.01.when" value="ORDER_START" />
        <property name="vda5050:action.02" value="Door" />
        <property name="vda5050:action.02.blockingType" value="NONE" />
        <property name="vda5050:action.02.parameter.script" value="robot_pose" />
        <property name="vda5050:action.02.when" value="ORDER_END" />
    </path>
     <path name="Point-0090 --- Point-0004" sourcePoint="Point-0090" destinationPoint="Point-0004"
        length="7250" maxVelocity="1000" maxReverseVelocity="1000" locked="false">
        <pathLayout connectionType="DIRECT" layerId="0" />
        <peripheralOperation completionRequired="true" executionTrigger="AFTER_ALLOCATION"
            locationName="Location-0001" name="up" />
        <peripheralOperation completionRequired="false" executionTrigger="AFTER_MOVEMENT"
            locationName="Location-0001" name="down" />
    </path>
```
- Location 操作点,停靠点
```xml
 <location name="Location-0001" xPosition="32600" yPosition="-57700" zPosition="0" locked="false"
        type="LIFT">
        <link point="Point-0038" />
        <property name="tcs:defaultLocationSymbol" value="DEFAULT" />
        <locationLayout xPosition="32600" yPosition="-57700" xLabelOffset="-10" yLabelOffset="-20"
            locationRepresentation="DEFAULT" layerId="0" />
    </location>
```
- Vehicle 车辆
```xml
    <vehicle name="Vehicle-0003" length="1000" energyLevelCritical="30" energyLevelGood="90"
        energyLevelFullyRecharged="30" energyLevelSufficientlyRecharged="90" maxVelocity="1000"
        maxReverseVelocity="1000" envelope="3A">
        <property name="tcs:preferredAdapterClass"
            value="org.opentcs.virtualvehicle.LoopbackCommunicationAdapterFactory" />
        <property name="vda5050:interfaceName" value="uagv" />
        <property name="vda5050:manufacturer" value="rw" />
        <property name="vda5050:minVisualizationInterval" value="1000" />
        <property name="vda5050:orderQueueSize" value="2" />
        <property name="vda5050:serialNumber" value="tx3" />
        <property name="vda5050:version" value="2.0" />
        <property name="vda5050:ip" value="192.168.0.39" />
        <property name="vda5050:port" value="1883" />
        <vehicleLayout color="#00FFFF" />
    </vehicle>
```
- Block 规则区域
```xml
 <block name="Block-0006" type="SINGLE_VEHICLE_ONLY">
        <member name="Point-0079" />
        <member name="Point-0079 --- Point-0080" />
        <member name="Point-0080" />
        <member name="Point-0085" />
        <member name="Point-0085 --- Point-0079" />
        <blockLayout color="#9900CC" />
    </block>
```
- LocationType 操作点类型
```xml
 <locationType name="LIFT">
        <allowedPeripheralOperation name="up" />
        <allowedPeripheralOperation name="down" />
        <property name="tcs:defaultLocationTypeSymbol" value="NONE" />
        <locationTypeLayout locationRepresentation="NONE" />
    </locationType>
    <locationType name="Transfer">
        <allowedOperation name="NOP" />
        <allowedOperation name="pick" />
        <allowedOperation name="drop" />
        <property name="tcs:defaultLocationTypeSymbol" value="LOAD_TRANSFER_ALT_2" />
        <property name="vda5050:destinationAction.drop.parameter.angle" value="1.57" />
        <property name="vda5050:destinationAction.drop.parameter.script" value="robot_rotate" />
        <property name="vda5050:destinationAction.pick.parameter.angle" value="1.57" />
        <property name="vda5050:destinationAction.pick.parameter.script" value="robot_rotate" />
        <locationTypeLayout locationRepresentation="LOAD_TRANSFER_ALT_2" />
    </locationType>
```

如果操作成功会显示地图


![view](imgs/map.png)


### 打开车辆客户端

#### 虚拟车辆
 通过配置模型文件车辆“intendedVehicle”属性
 上传模型后自动加载虚拟车辆，不需要另外开启客户端
 虚拟车辆可以下单

#### vda模拟车辆 
需要另外开启测试车辆客户端
单辆车
```
./sim_agv_test 127.0.0.1
```
如果模拟测试多辆车
```
./sim_agv_test 127.0.0.1 false
```
成功连接到代理后显示
```
2024-04-30 13:36:23,770 INFO [default] tx1  ONLINE
```
服务端显示
```
[I 240430 13:36:23.7704 vehicle.cc:657] Vehicle-0001 tx1 ONLINE
```
表示已经和车辆在正常通信了
网页可以看到车辆id和电量
![view](imgs/normal.png)
如果电量不足会先自动去最近的充电点充电，充电结束后自动去最近的停靠点

车辆端执行任务会打印一些输出
```
2024-04-30 14:45:36,170 INFO [default] 接收到的mqtt消息内容....
2024-04-30 14:45:36,170 INFO [default] move from Point-0031 to Point-0030
2024-04-30 14:45:36,170 INFO [default] Vx -0.000218557 Vy -5000
2024-04-30 14:45:37,573 INFO [default] move from Point-0030 to Point-0029
2024-04-30 14:45:37,573 INFO [default] Vx -0.000218557 Vy -5000
```

车辆端退出，服务端会打印
```
[W 240430 15:25:08.6232 vehicle.cc:666] Vehicle-0001 tx1 CONNECTIONBROKEN
```
如果该车辆当前有任务，该任务随之失败


### 新建运输订单
#### 快速下单
> <font color=#b04f61>简单测试可以用鼠标在车辆位置按下左键，然后拖动到Point或Location点口释放可以快捷下单，对应的动作为MOVE或NOP，pick或drop
> 界面右上角显示最新的几条订单状态</font>
#### 也使用api工具发送请求：
路由 /transportOrders/{name}
参数 name 订单编号，唯一
body 订单具体内容

示例： 发送pick订单
![pick](imgs/pick.png)

![pick](imgs/pick_server.png)

车辆按规划路径移动，完后任务后如果空闲会去充电或着停靠点
去停靠点或者充电的路径上（根据电量情况）新建订单会立即取消当前任务去执行新订单
当前默认服务端会一次发送两个path(3个node)指令到车辆客户端，然后根据执行结果继续发送后续节点路径




### VDA动作
由机器人执行的动作，服务器发送指令，机器人执行，并反馈结果


当前
```
      NOP,     // 啥也不干
      pick,    // 去某location load
      drop,  // 去某location unload
      MOVE,    // 去某point,并停在那里，未完成
      Charge,  // 充电
      CLOSE, //关门
      OPEN, //开门
```
其中close和open是路径属性中的定义的动作自动下发，其他的可以由运输订单来下发
如果路径点上定义了相应的动作，根据触发时机会发送动作到车辆
服务端会显示动作状态
例如：Point-0027 --- point-0036路径定义了两个门的动作，当前是模拟只打印动作名称表示执行完成

![action](imgs/path_action.png)

```
[I 240430 13:51:41.9991 vehicle.cc:791] Vehicle-0001 01 Door
[I 240430 13:51:41.9992 vehicle.cc:791] Vehicle-0001 02 Door
```
等待动作完成（如果需要等待结果）
```
[I 240430 13:51:55.9562 vehicle.cc:1058] Vehicle-0001 action [pick] ok
```
所有需要等待结果的动作都完成后继续执行后续操作
```
[I 240430 13:51:55.9563 vehicle.cc:1075] Vehicle-0001 all actions ok
```

### 外围操作
电梯等其他外设，由服务器直接执行的动作，具体操作目前留空，用打印来模拟
如果路径点上定义了相应的动作，根据触发时机会去调用
例如 point-0090 --- point-0004上的定义了电梯操作
操作点要存在且该操作要在allowedPeripheralOperation中


![path_perop](imgs/path_perop.png)

刚刚执行完订单后，车辆当前在P12停靠，下一个去location-75的订单，车辆会经过此路径
在移动前up
```
[I 240430 14:36:04.6749 vehicle.cc:778] do Location-0001[up]
```
在移动后down
```
[I 240430 14:36:06.7941 vehicle.cc:895] do Location-0001[down]
```

### 禁用道路或操作点

使用api工具发送请求
>可以鼠标双击路径或操作点来启用或禁用

例如请求禁用Point-0022 --- Point-0023
请求成功后该道路变为不可用，同时网页显示浅灰色
![ban_path](imgs/ban_path.png)

机器人此时在P2点，发送一个NOP指令
规划的路径会避开该path
![ban_path](imgs/ban_path2.png)

禁用操作点后，如果订单的终点是操作点，订单会直接失败

发送取消禁用请求后道路变为可用，网页会重新显示

#### 其他api操作
使用API工具自行测试