
# HW/EXAM 1 : เต่าหรรษา+
เต่าหรรษา+ การบ้านสำหรับวิชา FRA501_2567 Robotics Dev โดยเพื่อศึกษาการทำงานของ ROS2 ได้อย่างถูกต้อง

## Systemp Overview
เนื่องจากทางกลุ่มของผมได้ทำการซื้อ System architecture และ .yaml มาเราจะขอใช้รูปเพื่อประกอบการอธิบาย

![image](https://cdn.discordapp.com/attachments/1284584015382183968/1284584239727251608/System_Overview.png?ex=66e729cd&is=66e5d84d&hm=0e8c5eee9b68681ee3c724c8908762dcf6b561f6e51bbc299fff54daf842aa36&)
[รูป system overview]
## Teleop_turtle Descriptions

![image](https://cdn.discordapp.com/attachments/1284584015382183968/1284684417981223016/Screencastfrom09-15-2024081320AM-ezgif.com-video-to-gif-converter.gif?ex=66e78719&is=66e63599&hm=7c37e3fef18470cc18f62482fa55cd4f94d912890e4776c29cbf2cd6ab7628b6&)

## Teleop.py
![imgae](https://cdn.discordapp.com/attachments/1284584015382183968/1284685291830902896/Screenshot_from_2024-09-15_08-20-26.png?ex=66e787e9&is=66e63669&hm=f79ba08cfb6f211328f0b48a31378afdd679d5c5a9a82e6fc9df79140d57b9e4&)

Node Teleop_key.py 
มีหน้าที่ในการส่งค่า /custom_key/cmd_vel ไปยัง teleop_scheduler โดยการใช้ keyboard ของเราในการสั่งงาน

นอกจากนี้ยังมี path ต่างๆที่มีหน้าที่ต่างกันโดยจะเริ่มต้นที่

**1. `/save_path`**:
 ที่จะคอยเซฟตำเเหน่งใหม่ไปยังไฟล์ .ymal ทั้ง 4 ค่า (ใช้ไม่ได้)

**2. `/clear_path `**:จะมีหน้าที่คอยกินพิซซ่าที่ไม่ได้ถูกเซฟในไฟล์ .ymal  (ใช้ไม่ได้)

**3. `/make_path`** จะมีหน้าที่ในการส่งทิ้ง Pizza ตรงไหน (ใช้ได้)

### teleop_scheduler:

รับค่าความเร็วจาก /custom_key/cmd_vel
เเละยังมีSrvice จาก save_path , clear_path , make_path
/run เป็น service ในการสั่งให้เต่าทำงาน
/reach_notify บ่งบอก state ของการทำงาน
/target ที่คอยส่งเป้าหมายไปยัง Controller
/pose ส่งตำเเหน่งไปยัง Controller
cmd_velส่งว่าควรมีความเร็วเท่าไหร่ไปให้ turtlesim
pizza_count นับพิซซ่าที่ถูกกิน
/spawn_pizza เป็น service ที่คอยปล่อยพิซซ่าไปตามตำเเหน่งเต่าที่เดิน

### Controller:
รับ target มาจาก Scheduler
มีการใช้service notify 
publish ค่าความเร็ว cmd_velไปยัง turtlesim
เเละรับตำเเหน่งของเต่าผ่าน pose

### turtlesim+:
ส่งตำเเหน่งของเต่าไปยัง controller เพื่อที่จะได้รับ
รับความเร็ว cmd_vel จากcontroller 
มีการส่งค่า pizza_count ไปยัง teleop_scheduler
เเละใช้service spawn_pizza ในการวางพิซซ่าตามตำเเหน่งที่เต่าวิ่ง

### Pizzapath.ymal :
เป็นไฟล์ส่งค่า x , y ไปยัง copy turtle

## Copy_turtle Node Descriptions
### 1. `copy_schedule.py`
Node copy_schedule.py มีหน้าที่หลักๆ:
- อ่านค่าตำแหน่งจาก .yaml
- ส่งค่าตำแหน่งไปให้ Controller 
- กำหนดหน้าที่ (state) ให้กับตัวหุ่นเต่าแต่ละตัว

#### Key Components:
- **Parameters**:
  - `turtltsim_namespace`: Namespace ของ windows ที่เราสร้าง
  - `turtle_name`: ชื่อเต่าแต่ละตัว
  - `my_copy_index`: Index ของเต่าแต่ละตัว

- **Publisher**:
  - `target_pub_client`: ส่งค่า `Goaltarget` ตำแหน่งที่เต่าจะไปทิ้งพิซซ่า

- **Services**:
  - `/run`: สั่งงานการเริ่มทำงานของเต่า Copy ทั้งหมด 4 ตัว
  - `/reach_notiffy`: กำหนด State ของเต่าแต่ละตัวว่าอยู่สถานะไหน
  - `/empty_notiffy`: มีหน้าที่ในการสั่งงานตอนสุดท้ายเมื่อเต่าทั้งหมดได้วางพิซซ่าครบหมดแล้ว

### 2. `controller_node.py`

controller_node มีหน้าที่ :
- สั่งงานให้เต่าเดินไปตามทางเดิม
- รับส่งค่าสถานะจาก `copy_schedule`

#### Key Components:
- **Parameters**:
  - `turtltsim_namespace`: Namespace ของ windows ที่เราสร้าง
  - `turtle_name`: ชื่อเต่าแต่ละตัว

- **Subscribers**:
  - `pose_callback`: รับค่าตำแหน่งของเต่า
  - `Target_callback`: รับค่าตำแหน่งของพิซซ่าที่เรา

- **Publisher**:
  - `cmdvel_pub`: ส่งค่า `Twist` เพื่อควบคุมเต่า

- **Services**:
  - `spawn_pizza`: สร้างพิซซ่าที่ตำแหน่งนั้น ๆ
  - `client_reach_notify_pub`: ส่งค่าสถานะไปถาม `copy_schedule` 

ในส่วนของ Copy_turtle นั้นจะเป็นการสร้างเต่า 4 ตัวโดยใช้ Launch file ขื้นมาหลังจากที่เราได้ทำการกด Save ทั้งหมด 4 ครั้งจาก Teleop_turtle จากนั้นเต่าทั้ง 4 ตัวจะเริ่มทำงาน 

![image](https://cdn.discordapp.com/attachments/1284584015382183968/1284584945590734969/image.png?ex=66e72a75&is=66e5d8f5&hm=ec96833c5a108e556e7153d3a3b26e761f8a27904e133e6cbc0312de0a6171ad&)
[รูป system overview ของ Copy turtle]


เต่า Copy จะเริ่มทำงานต่อเมื่อได้รับ service /run ขึ้นมา ถึงจะเริ่มการทำงาน

![image](https://cdn.discordapp.com/attachments/1284584015382183968/1284587702469132289/Screencastfrom09-15-2024014950AM-ezgif.com-video-to-gif-converter.gif?ex=66e72d06&is=66e5db86&hm=df12b723d84f9a43c76000b37ea0e4e082adf84749f0f6db4579997c4a6b7f4a&)
[ตัวอย่างการทำงานของ Copy turtle]

### 3.`turtlesim_launch.py`

สร้าง Node ทั้งหมดที่จำเป็นต่อการใช้ Copy_turtle ได้แก่

#### Components:
- **Turtlesim Node**: สร้างเต่าแต่ละตัว
- **Copy Schedule Node**: สร้างตัวจัดการงาน `copy_schedule.py` เพื่อควบคุมเต่า
- **Controller Node**: สร้างตัวควบคุม `controller_node.py` เพื่อควบคุมเต่า
- **Service Calls**:
  - `remove_turtle`: ลบเต่าที่สร้างขึ้นมาทิ้งทั้งหมด
  - `spawn_turtle`: สร้างเต่าใหม่ให้ชื่อตรงกับที่เรากำหนด

## Usage

#### 1.Clone Project from github
```bash
git clone https://github.com/ThepokKung/homework1_ws
cd homework1_ws
```


#### 2.Build Project
```bash
colcon build --parallel-workers 2 # --parallel-workers 2 
```
(--parallel-workers เพื่อเพิ่ม CPU ในการประมวลผล ตอนรันครั้งแรกจะประมวลไม่ทันแล้วคอมค้าง)

#### 3.Source
```bash
source install/setup.bash 
```
#### 4.Run file
#### * 4.1 teleop_turtle.launch.py
รันเต่า teleop ที่ควบคุมด้วย keyboard ได้
```bash
ros2 launch turtle_bringup_plus teleop_turtle.launch.py
```
เนื่องจากเต่า teleop ไม่สามารถบังคับได้ด้วยตัวเอง เลยต้องใช้ keyboard ในการควบคุมน้อง
```bash 
ros2 run turtle_bringup_plus teleop_key.py 
```
#### * 4.2 copy_turtle.launch.py
```bash
ros2 launch turtle_bringup_plus copy_turtle.launch.py
```
เนื่องจากว่าจริงๆ น้องต้องรับค่าจากเต่า teleop ทำให้น้องยังไม่สามารถเริ่มการทำงานด้วยตัวเองได้ เราต้องสั่งน้องทำงานโดยการใช้ service call หรือจะผ่าน rqt ก็ได้
#### ผ่าน terminal
```bash
ros2 service call /run package_teleop_interfaces/srv/Run "command: 1" 
```
#### ผ่าน rqt 
![image](https://cdn.discordapp.com/attachments/1284584015382183968/1284688309901332561/Screenshot_from_2024-09-15_08-32-16.png?ex=66e78ab9&is=66e63939&hm=9a78a25ef5912684d84318a603802b478752e071a82a62ecd17c058b7470108a&)

### ผู้จัดทำ
65340500001 กันตพัฒน์ เลาหะพันธ์ุ

65340500004 ไกรวิชญ์ วิชาโคตร
