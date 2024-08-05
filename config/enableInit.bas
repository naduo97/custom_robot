 
 



''''''''''''''''''''''''初始化准备

RAPIDSTOP(2)  '总线启动。清空运动缓冲区，停止当前运动


WAIT IDLE

FOR i=0 to 20                                            '取消原来的总线轴设置

        ATYPE(i)=0   
		?"111111"

NEXT

''''''''''''''''''''''''EtherCAT总线初始化


SLOT_SCAN(0)                                             '开始扫描
'SLOT_SCAN (slot)
'slot：控制器 EtherCAT 槽位号或 RTEX 槽位号， 0-缺省

IF RETURN THEN 

        ?"总线扫描成功","连接设备数："NODE_COUNT(0)

        ?

        ?"开始映射轴号"
		AXIS_ADDRESS(0)=0+1
        AXIS_ADDRESS(1)=0+2             '映射轴号
		AXIS_ADDRESS(2)=0+3 
		AXIS_ADDRESS(3)=0+8
		AXIS_ADDRESS(4)=0+9
		
		AXIS_ADDRESS(12)=0+4
		AXIS_ADDRESS(13)=0+5
		AXIS_ADDRESS(14)=0+6
		AXIS_ADDRESS(15)=0+7
		
		
		'AXIS_ADDRESS (0)=(0<<16)+0+1 '第一个 ECAT 驱动器，驱动器编号 0，绑定为轴 0
		'AXIS_ADDRESS (2)=(0<<16)+1+1 '第二个 ECAT 驱动器，驱动器编号 1，绑定为轴 2
		'AXIS_ADDRESS (1)=(0<<16)+2+1 '第三个 ECAT 驱动器，驱动器编号 2，绑定为轴 1
		
		'012为大臂的三轴，4为小臂传送带，5为推杆，6为大臂传送带，12为俯仰
		
	
		
		
        ATYPE(0)=65                                 'EtherCAT类型
		ATYPE(1)=65                                    '65位置控制，66速度控制，67力矩控制
		ATYPE(2)=65									'这里的关节轴是0-4,5个轴，6-11是虚拟轴，对应xyzrpy,5也是虚拟轴，为了让关节轴数量和虚拟轴对应
		ATYPE(3)=65
		ATYPE(4)=65
		ATYPE(5)=0
		ATYPE(6)=0
		ATYPE(7)=0
		ATYPE(8)=0
		ATYPE(9)=0
		ATYPE(10)=0
		ATYPE(11)=0
		ATYPE(12)=65
		ATYPE(13)=65
		ATYPE(14)=65
		ATYPE(15)=65
		
		'DPOS(1)=0                                   
        'DPOS(2)=0                                    
		'DPOS(3)=0
		'DPOS(4)=0
		'DPOS(5)=0
		'DPOS(6)=0
		
		
		DRIVE_PROFILE(0)= -1
        DRIVE_PROFILE(1)= -1              '伺服PDO功能  控制器和从站之间数据传输方式有两种，一种是按指定时间周期性交换数据，称之为 PDO(Process Data Object)
		DRIVE_PROFILE(2)= -1
		DRIVE_PROFILE(3)= -1
		DRIVE_PROFILE(4)= -1
		DRIVE_PROFILE(12)= -1
		DRIVE_PROFILE(13)= -1
		DRIVE_PROFILE(14)= -1
		DRIVE_PROFILE(15)= -1
		'DRIVE_PROFILE(6)= -1
                                             'ATYPE=66时设置DRIVE_PROFILE=20

                                             'ATYPE=67时设置DRIVE_PROFILE=30
		
		'DRIVE_PROFILE -- 驱动器 PDO 设置									 
		'每个轴的发送 pdo 接收 pdo 的配置选择。
		'必须设置正确的 ATYPE（设置为 65/66/67）以后才能操作这个参数。									 
		DISABLE_GROUP(0)
        DISABLE_GROUP(1)                           '每轴单独分组  ,
		DISABLE_GROUP(2)
		DISABLE_GROUP(3)
		DISABLE_GROUP(4)
		DISABLE_GROUP(12)
		DISABLE_GROUP(13)
		DISABLE_GROUP(14)
		DISABLE_GROUP(15)
		'DISABLE_GROUP(6)
		
		
		'一停多，把几个轴设置为一组，驱动器告警后会关闭组内的所有使能(ECAT 产品支持)，脉冲轴设置无意义。
		
        ?"轴号映射完成"

        DELAY (100)

 

        SLOT_START(0)              '总线开启
		'SLOT_START(0),总线启动。
		'通过 RETURN 返回成功与否。返回-1 启动成功， 0 启动失败。

        IF RETURN THEN 

                ?"总线开启成功"

                ?"开始清除驱动器错误(根据驱动器数据字典设置)"
				
				DRIVE_CONTROLWORD(0)=0
				DRIVE_CONTROLWORD(1)=0
                DRIVE_CONTROLWORD(2)=0        '配合伺服清除错误
				DRIVE_CONTROLWORD(3)=0
				DRIVE_CONTROLWORD(4)=0
				DRIVE_CONTROLWORD(12)=0
				DRIVE_CONTROLWORD(13)=0
				DRIVE_CONTROLWORD(14)=0
				DRIVE_CONTROLWORD(15)=0
				'DRIVE_CONTROLWORD(5)=0
				'DRIVE_CONTROLWORD(6)=0
                DELAY (10)

				DRIVE_CONTROLWORD(0)=128
				DRIVE_CONTROLWORD(1)=128
                DRIVE_CONTROLWORD(2)=128       'bit7=1强制伺服清除错误
				DRIVE_CONTROLWORD(3)=128
				DRIVE_CONTROLWORD(4)=128
				DRIVE_CONTROLWORD(12)=128
				DRIVE_CONTROLWORD(13)=128
				DRIVE_CONTROLWORD(14)=128
				DRIVE_CONTROLWORD(15)=128
				'DRIVE_CONTROLWORD(5)=128
				'DRIVE_CONTROLWORD(6)=128
				
                DELAY (10)
				
				DRIVE_CONTROLWORD(0)=0  
				DRIVE_CONTROLWORD(1)=0
                DRIVE_CONTROLWORD(2)=0         '配合伺服清除错误
				DRIVE_CONTROLWORD(3)=0
				DRIVE_CONTROLWORD(4)=0
				DRIVE_CONTROLWORD(12)=0
				DRIVE_CONTROLWORD(13)=0
				DRIVE_CONTROLWORD(14)=0
				DRIVE_CONTROLWORD(15)=0
				'DRIVE_CONTROLWORD(5)=0
				'DRIVE_CONTROLWORD(6)=0  
				
                DELAY (10)

                DATUM(0)                                                      '单轴找原点运动。ZMC 控制器为 0 触发有效，输入为 OFF 状态时，表示到达原点/限位

                DELAY (100)         

                ?"轴使能准备"
                '使能总开关

				AXIS_ENABLE(0)=1  
				AXIS_ENABLE(1)=1
                AXIS_ENABLE(2)=1                   '轴使能
				AXIS_ENABLE(3)=1   
				AXIS_ENABLE(4)=1 
				AXIS_ENABLE(12)=1 
				AXIS_ENABLE(13)=1
				AXIS_ENABLE(14)=1
				AXIS_ENABLE(15)=1
                WDOG=1                                                        '使能总开关

                ?"轴使能完成" 

ELSE

                ?"总线开启失败"

        ENDIF

ELSE

        ?"总线扫描失败"

ENDIF


DELAY(5000)







'''''电机、机械手参数定义




DIM baseL,L1,L2,L3,L4,L5,L6,L7,L8,L9,L10,L11,L12,crank(3),dok(4),tcp(6)
baseL = 358
'五连杆
L1 = 850
L2 = 1190
L3 = 800
L4 = 500
L5 = 568.25
dok(0) = -ATAN2(122,555) +  PI/2  'double atan2(double y, double x),返回以弧度表示的 y/x 的反正切。y 和 x 的值的符号决定了正确的象限
dok(1) =  ATAN2(0,1190)   '
dok(2) = -ATAN2(138.6, 787.9)- ATAN2(122,555)  + PI/2 'CDX
dok(3) = -ATAN2(122,555) - ATAN2(59.9,496.4)  '
'五连杆的

'曲柄

'dok(0) =l1与x轴的夹角，dok(1)=l2与x轴夹角，dok(2)=l3与x轴夹角，dok(3)=l4与x轴夹角
L6 = 300
L7 = 500
L8 = 557.31
L9 = 462.55
crank(0) = ATAN2(112.09,278.27) + ATAN2(107,450) + PI/2   
crank(1) = ATAN2(164.73,472.09) + ATAN2(107,450) 'CBX
crank(2) = ATAN2(107,450) + ATAN2(90,550) +  PI/2 '
'大臂长，小臂长
L10 = 253 
L11 = 4228
L12 = 0

'tcp(0, 1184 , 0, 139.25, 0, 0, 0)
'tcp(0, 1207 , 0, 114.78, 0, 0, 0)
'tcp(0, 1211 , 0, 102, 0, 0, 0)
tcp(0, 1216.96 , -2, 96, 0, 0, 0)'20230725标定TCP

dim u_j1,u_j2,u_j3,u_j4,u_j5,u_j6                '关节电机1-6实际一圈脉冲数
u_j1=8388608  '编码器123轴23位，45轴19位
u_j2=8388608
u_j3=8388608
u_j4=524288
u_j5=524288
u_j6=524288

dim i_1,i_2,i_3,i_4,i_5,i_6                '关节1-6传动比    123轴按照电机实际参数设置，45轴是零差云控关节模组，输出负载端编码器的位置反馈，是除以了减速比以后的，所以减速比为1  
i_1=205
i_2=200
i_3=200
'i_1=1
'i_2=1
'i_3=1
i_4=1
i_5=1
i_6=1
''这个控制器端的配置，因为控制器获取的关节输出端位置是经过减速比后的位置反馈
dim Pules1OneCircle, Pules2OneCircle, Pules3OneCircle, Pules4OneCircle, Pules5OneCircle, Pules6OneCircle, PulesROneCircle '每个关节转动一圈的实际脉冲数
Pules1OneCircle=u_j1*i_1
Pules2OneCircle=u_j2*i_2
Pules3OneCircle=u_j3*i_3
Pules4OneCircle=u_j4*i_4
Pules5OneCircle=u_j5*i_5
Pules6OneCircle=u_j6*i_6


PulesROneCircle=360*1000

'''''关节轴设置
BASE(0,1,2,3,4,5)                                '选择关节轴号0、1、2、3、4、5(其5代表关节5，不能动，atype务必 = 0)
'BASE(1,2,3,4,5,6)
'FS_LIMIT = 360,180,180,180,360,360        '这里的上下限位是角位移么？ 
'RS_LIMIT = -360,-180,-180,-180,-360,-360
 
'atype=0,0,0,0,0,0                               '轴类型，0为虚拟轴
'atype=65,65,65,65,65,0                             '轴类型设为EtherCAT周期位置模式 
'FS_LIMIT = 15,38,35.5,18,15,15        '这里的上下限位是角位移么？ 
'RS_LIMIT = -15,-35,-61.5,-18,-15,-15


FS_LIMIT = 15,38,35.5,10,15,15        '这里的上下限位是角位移么？ 
RS_LIMIT = -15,-35,-61.5,-26,-15,-15

UNITS = Pules1OneCircle/360,Pules2OneCircle/360,Pules3OneCircle/360,Pules4OneCircle/360,Pules5OneCircle/360,Pules6OneCircle/360   '把units设成每°脉冲数
'DPOS=0,0,0,0,0,0

'MPOS= 0,0,0,0,0,0                             '设置关节轴的位置，此处要根据实际情况来修改。



'20230906更新零点标定
vr(0) = -133805639
vr(1) = 219048662
vr(2) = 12240320
vr(3) = 191923  '数字减小 小臂向上抬
vr(4) = 288154
vr(13)= -19656000



for i = 0 to 4
	
	dpos(i) = (encoder(i) - vr(i))/units(i)
	mpos(i) = dpos(i)
next
?"dpos初始化完成"
Print "DPOS: " DPOS(0),DPOS(1),DPOS(2),DPOS(3),DPOS(4)
dpos(13)= (encoder(13) -vr(13))/534306
MPOS(13)= DPOS(13)





speed=5,5,5,5,5,5                        '速度参数设置,5已经是最小速度
accel=10,10,10,10,10,10   				'设置加速度和减速度，此处从50改成10，加速到5用时0.5s，加速度减速度降低以后可以显著改善运动控制过程中末端的大幅摆动
decel=100,100,100,100,100,100

'accel=50,50,50,50,50,50
'decel=50,50,50,50,50,50
FASTDEC=100,100,100,100,100,100
CLUTCH_RATE=1000,1000,1000,1000,1000,1000                'CONNECT 连接的速度，用于定义连结率从 0 到设置倍率的改变时间， ratio/秒

'CONNECT,同步运动指令,将当前轴的目标位置与 driving_axis 轴的测量位置通过电子齿轮连接。

BASE(0,1,2)
FWD_IN=0,1,2  '分别设置正向限位开关,输入 IN0 作为轴 0 的正向限位信号
INVERT_IN(0,ON) '反转信号
INVERT_IN(1,ON)  'ZMC 系列控制器输入 OFF 时认为有信号输入
INVERT_IN(2,ON)

BASE(0,1,2)
REV_IN=3,4,5  '分别设置负向限位开关,输入 IN3 作为轴 0 的负向限位信号
INVERT_IN(3,ON) '反转信号
INVERT_IN(4,ON)  'ZMC 系列控制器输入 OFF 时认为有信号输入
INVERT_IN(5,ON)





merge=on                                         '开启连续插补  不开插补的话两段运动之间不连续，开插补后，运动轨迹更加平滑
'merge=off
corner_mode = 2                                '启动拐角减速 自动拐角减速,按 ACCEL,DECEL 加减速度,此参数是在 MOVE 函数调用前生效

decel_angle = 15 * (PI/180)                '开始减速的角度 15度   运动15度以下是不减速，15-45度是等比减速，45度以上是完全减速
'完全减速是按照decel的减速度减速吗？等比减速是怎样的？  手册里说等比减速以FORCE_SPEED为参考值，FORCE_SPEED在等比减速时起作用
stop_angle = 45 * (PI/180)                '降到最低速度的角度45度

'?"关节轴DPOS："DPOS

'''''''''''''''''''''虚拟轴设置'''''''''''''''''''''
BASE(6,7,8,9,10,11)
'BASE(7,8,9,10,11,12)
FS_LIMIT = 200000,200000,20000,20000,200000
RS_LIMIT = -200000,-200000,-20000,-20000,-200000



speed=230,230,230,230,230,230                        '速度参数设置  100uits/s
accel=250,250,250,250,250,250
decel=250,250,250,250,250,250
FASTDEC=1000,1000,1000,1000,1000,1000
'decel=1000,1000,1000,1000,1000,1000
'ATYPE=0,0,0,0,0,0                                            '设置为虚拟轴,前面总线初始化那里有设置
DPOS=0,0,0,0,0,0     
TABLE(10,baseL,L1,L2,L3,L4,L5,L6,L7,L8,L9,L10,L11,L12,crank(0),crank(1),crank(2),dok(0),dok(1),dok(2),dok(3),Pules1OneCircle, Pules2OneCircle, Pules3OneCircle, Pules4OneCircle, Pules5OneCircle , PulesROneCircle,tcp(0),tcp(1),tcp(2),tcp(3),tcp(4),tcp(5))

'比较点坐标设置   参数存储在 TABLE10 开始的位置  这个TABLE是什么还是不太理解，手册里也有说是数据存储的位置



UNITS=1000,1000,1000,1000,1000,1000         '运动精度，要提前设置，中途不能变化   UNITS脉冲当量




BASE(20,21,22,23,24,25)  'frame 123 虚拟轴
FS_LIMIT = 200000,200000,20000,20000,200000
RS_LIMIT = -200000,-200000,-20000,-20000,-200000
speed=100,100,100,100,100,100                        '速度参数设置  100uits/s
accel=1000,1000,1000,1000,1000,1000
decel=1000,1000,1000,1000,1000,1000
'DPOS=0,0,0,0,0,0  
UNITS=1000,1000,1000,1000,1000,1000         '运动精度，要提前设置，中途不能变化   UNITS脉冲当量
ATYPE= 0,0,0,0,0,0

TABLE(50,baseL,L1,L2,L3,L4,L5,L6,L7,L8,L9,L10,L11,L12,crank(0),crank(1),crank(2),dok(0),dok(1),dok(2),dok(3),Pules1OneCircle, Pules2OneCircle, Pules3OneCircle, Pules4OneCircle, Pules5OneCircle , PulesROneCircle,0,0,0,0) 
BASE(26,27,28,29,30,31)  'frame 123 关节轴
FS_LIMIT = 200000,200000,20000,20000,200000
RS_LIMIT = -200000,-200000,-20000,-20000,-200000
speed=100,100,100,100,100,100                        '速度参数设置  100uits/s
accel=1000,1000,1000,1000,1000,1000
decel=1000,1000,1000,1000,1000,1000
'DPOS=0,0,0,0,0,0  
UNITS=Pules1OneCircle/360,Pules2OneCircle/360,Pules3OneCircle/360,Pules4OneCircle/360,Pules5OneCircle/360,Pules6OneCircle/360          '运动精度，要提前设置，中途不能变化   UNITS脉冲当量
ATYPE= 0,0,0,0,0,0

'CONNECT(1,0) AXIS(26)  '电子齿轮连接frame123的关节轴30和实际的关节轴0
'CONNECT(1,1) AXIS(27)
'CONNECT(1,2) AXIS(28)
'CONNECT(1,3) AXIS(29)
'CONNECT(1,4) AXIS(30)

ADDAX(0)  AXIS(26) '运动叠加，叠加的是脉冲个数
ADDAX(1)  AXIS(27) 
ADDAX(2)  AXIS(28) 
ADDAX(3)  AXIS(29) 
ADDAX(4)  AXIS(30) 




'		
BASE(13)   '选择轴号2

'电机编码器23位，分辨率8388608，一个编码器分辨率总数电机转一圈，脉冲当量=8388608/360=23301  需要考虑减速比吗？

UNITS(13)=534306   '脉冲当量552731   脉冲当量一般设为电机走 1mm 或 1°的所需脉冲数。脉冲当量p= e*k/L  e编码器分辨率8388608，L为同步带周长，k为减速比
SPEED(13)=450  '速度设置为1000
CREEP(13)=150   ' 找原点反向爬行速度，速度为700，这里是归零？
ACCEL(13)=2000  '加速度
DECEL(13)=2000  '减速度
REP_DIST = 100000000   '设置坐标循环范围
'RS_LIMIT = -5 '负向软限位，必须大于-REP_DIST 才会生效
'FS_LIMIT = 580 '正向软限位，必须小于 REP_DIST 才会生效
''DPOS(13)=0
''MPOS(13)=0   '编码器反馈位置
'MERGE = ON   '启动连续插补
'corner_mode = 2 
'SRAMP=100    '加减速变化率，使得速度曲线平滑，在机械启停或加减速时减少抖动，范围在0-250毫秒之间
'FASTDEC(13)=300   '快减减速度，达到限位或异常停止时自动采用。
'IF IN(9) > 0 THEN
'	?"已在原点！"
'ELSE
'	DATUM_IN=9 '输入 IN9 作为原点开关     
'	INVERT_IN(9,OFF) '反转 IN9 电平信号，常开信号进行反转(ZMC 控制器)     ON是反转 OFF是不反转，这条指令应该可以不要？                
'	'通用输入口设置为原点开关信号， 1 无效。
'	'设置了原点开关后， ZMC 控制器输入 OFF 时认为有信号输入，要相反效果可以用
'	'INVERT_IN 反转电平。 (ECI 系列控制器除外)。
'	TRIGGER   '自动触发示波器
'
'	DATUM(3)'模式3下找原点'   DATUM单轴找原点运动  以speed的速度
'	?"正在复位！"
'	WAITIDLE
'	?"复位成功！原点已回零！"
'ENDIF
'DPOS(13)=0 '轴13推杆的位置已清零'
'MPOS(13)=0
'?"推杆位置已复位清零！"

DELAY(1000)



'RUNTASK 1, conveyer_control() '传送带控制的子线程，包括小臂和大臂的传送带
'RUNTASK 2, pressure_control()  '传送带控制的子线程，包括小臂和大臂的传送带
'RUNTASK 3, end_serv_control()
RUNTASK 4, identification()    '相机手眼标定子线程，用来求解前三轴正解
'RUNTASK 5, lidar_scan()        '雷达避障子线程，用来保护机械臂上下左右不发生碰撞
'RUNTASK 6, end_effector_rotate()  '装货时让末端发生偏转的子线程


'''初始位置，零位




DIM Z1,Z2,Z3,Z4,Z5,Z6
'
'
''''原点''''
'Z1=5400
'Z2=800
'Z3=1800
'Z4=0   ' rx  翻滚
'Z5=0   ' ry  俯仰
'Z6=0   ' rz  偏航
'manipulator_control(Z1,Z2,Z3,Z4,Z5,Z6)
'WAITIDLE
'MOVE_DELAY(2000)

'end_effector_control()

'
'
'CONNREFRAME_CONTROL()

'Z1=5412
'Z2=0
'Z3=800
'Z4=0    rx  '翻滚
'Z5=0    ry ' 俯仰
'Z6=0    rz  '偏航
'manipulator_control(Z1,Z2,Z3,Z4,Z5,Z6)
'WAITIDLE
'MOVE_DELAY(2000)
'''''原点''''
'
''
'Z1=5412
'Z2=-800
'Z3=1400
'Z4=0   ' rx  翻滚
'Z5=0   ' ry  俯仰
'Z6=0   ' rz  偏航
'manipulator_control(Z1,Z2,Z3,Z4,Z5,Z6)
'WAITIDLE
'MOVE_DELAY(2000)



'Z1=5212
'Z2=-700
'Z3=1300
'Z4=0   ' rx  翻滚
'Z5=0   ' ry  俯仰
'Z6=0   ' rz  偏航
'manipulator_control(Z1,Z2,Z3,Z4,Z5,Z6)
'WAITIDLE
'MOVE_DELAY(2000)
''
''
'DELAY(2000)

DIM a,b,c

a=AIN(0) '读取通道 1 的 AD
a=AIN(0) *10 /4095 '通道 1 的电压值     ''实际电压0.278V，距离410mm
PRINT "当前电压：" a
b = a*1000*0.099 + 98.586  '对应的距离
c = b*0.7985
'

PRINT "测距传感器检测到的距离" b 

PRINT "直线距离" c


'
''
'
''
'''
'''第一个箱子
''
''
'
'Z1=5470
'Z2=440
'Z3=968
'Z4=0
'Z5=0
'Z6=0 
''
'manipulator_control(Z1,Z2,Z3,Z4,Z5,Z6)
'WAITIDLE
'MOVE_DELAY(2000)
'''
'end_effector_control()
''''
'''''
''''''第二个箱子
'''''
'Z1=5490
'Z2=-70
'Z3=1128
'Z4=0
'Z5=0
'Z6=0 
''
'manipulator_control(Z1,Z2,Z3,Z4,Z5,Z6)
''WAITIDLE
''MOVE_DELAY(2000)
'''
'''

''''''
'''''''第三个箱子
'Z1=5490
'Z2=-460
'Z3=1128
'Z4=0
'Z5=0
'Z6=0 
'''
'manipulator_control(Z1,Z2,Z3,Z4,Z5,Z6)
''WAITIDLE
''MOVE_DELAY(2000)
'
''
'IF SCAN_EVENT(IN(12)) > 0   THEN  ' IN12 IN13上升沿触发
'	PRINT "货箱经过尾部"
'	end_effector_control()
'ENDIF
''''''
''''''
'''''''第四个箱子
'Z1=5490
'Z2=-460
'Z3=950
'Z4=0
'Z5=0
'Z6=0 
'
'manipulator_control(Z1,Z2,Z3,Z4,Z5,Z6)
''WAITIDLE
''MOVE_DELAY(2000)
'''
'''
'IF SCAN_EVENT(IN(12)) > 0 AND SCAN_EVENT(IN(13)) > 0 THEN  ' IN12 IN13上升沿触发
'	end_effector_control()
'ENDIF
''''''
'''''''第五个箱子
''''''
'Z1=5490
'Z2=-70
'Z3=950
'Z4=0
'Z5=0
'Z6=0 
'
'manipulator_control(Z1,Z2,Z3,Z4,Z5,Z6)
''WAITIDLE
''MOVE_DELAY(2000)
'''
'''
'IF SCAN_EVENT(IN(12)) > 0 AND SCAN_EVENT(IN(13)) > 0 THEN  ' IN12 IN13上升沿触发
'	end_effector_control()
'ENDIF
''''''
''''''
'''''''第六个箱子
'Z1=5490
'Z2=320
'Z3=950
'Z4=0
'Z5=0
'Z6=0 
'
'manipulator_control(Z1,Z2,Z3,Z4,Z5,Z6)
''WAITIDLE
''''MOVE_DELAY(2000)
''
'IF SCAN_EVENT(IN(12)) > 0 AND SCAN_EVENT(IN(13)) > 0 THEN  ' IN12 IN13上升沿触发
'	end_effector_control()
'ENDIF
''''''
'''''''第七个箱子
'Z1=5490
'Z2=320
'Z3=770
'Z4=0
'Z5=0
'Z6=0 
'
'manipulator_control(Z1,Z2,Z3,Z4,Z5,Z6)
''WAITIDLE
'''MOVE_DELAY(2000)
'''
'end_effector_control()
'''''''
'''''''第八个箱子
'Z1=5490
'Z2=-70
'Z3=770
'Z4=0
'Z5=0
'Z6=0 
'''
'manipulator_control(Z1,Z2,Z3,Z4,Z5,Z6)
''WAITIDLE
'''MOVE_DELAY(2000)
''
'end_effector_control()
''''''
'''''''第九个箱子
'Z1=5490
'Z2=-460
'Z3=770
'Z4=0
'Z5=0
'Z6=0 
'
'manipulator_control(Z1,Z2,Z3,Z4,Z5,Z6)
'
''WAITIDLE
''MOVE_DELAY(2000)
'
'
'end_effector_control()
''
'Z1=5417.428
'Z2=0
'Z3=1458
'Z4=0   ' rx  翻滚
'Z5=0   ' ry  俯仰
'Z6=0   ' rz  偏航
'manipulator_control(Z1,Z2,Z3,Z4,Z5,Z6)
'WAITIDLE
'MOVE_DELAY(2000)



END

SUB manipulator_control(t1,t2,t3,t4,t5,t6)

	BASE(0,1,2,3,4,5)  '关联关节轴
	CONNFRAME(121,10,6,7,8,9,10,11)'逆解模式
	WAIT LOADED

	'底盘高度170mm

	PRINT "frame状态：" MTYPE(0)
	IF MTYPE(0)=33 THEN   '判断是否处于逆解状态  正解是 mtype(6) = 34， 逆解状态判断 Mtype(0) = 33
		BASE(6,7,8,9,10,11)
		PRINT "t1-t6:"t1,t2,t3,t4,t5,t6

		table(100,t1,t2,t3,t4,t5,t6)
		frame_trans2(100,200,0)
		?"逆解返回："frame_trans2(100,200,0)'-1有解0无解

		?"DPOS(0)=",TABLE(200)
		?"DPOS(1)=",TABLE(201)
		?"DPOS(2)=",TABLE(202)
		?"DPOS(3)=",TABLE(203) 
		?"DPOS(4)=",TABLE(204)
		?"DPOS(5)=",TABLE(205)




		IF TABLE(200) > -15 AND TABLE(200)<15 AND TABLE(201) > -35 AND TABLE(201)<38 AND TABLE(202) > -61.5 AND TABLE(202)<35.5 AND TABLE(203) > -18 AND TABLE(203)<18 AND TABLE(204) > -15 AND TABLE(204)<15	THEN
			CONNREFRAME(121,10,0,1,2,3,4,5)
			BASE(0,1,2,3,4)
			errswitch = 4  
			MOVEABS(TABLE(200),TABLE(201),TABLE(202),TABLE(203),TABLE(204))  'moveabs，绝对运动指令，MOVE_PT，单位时间距离
			?"errswitch"
			TRACE "DPOS(0) =" DPOS(0)
			TRACE "DPOS(1) =" DPOS(1)
			TRACE "DPOS(2) =" DPOS(2)
			TRACE "DPOS(3) =" DPOS(3)
			TRACE "DPOS(4) =" DPOS(4)
			

			?"位置可达"
		ELSE
			?"超过机械限定范围"
			
		ENDIF
		
	ELSE
		?"没有进入逆解状态，检查轴报错"

	ENDIF


	''CONNREFRAME(121,10,0,1,2,3,4,5)'正解模式
	''BASE(0,1,2,3,4,5)
	''MOVEABS(0,0,0,0,0,0)  'MOVEABS,关节轴绝对运动
	''MOVER_LABS(0,0,0,0,5,0)
	PRINT"11111"
ENDSUB

SUB CONNREFRAME_CONTROL()
	BASE(6,7,8,9,10,11)
	CONNREFRAME(121,10,0,1,2,3,4,5)
	IF MTYPE(6)=34 THEN
		BASE(0,1,2,3,4)
		MOVEABS(0.55,-20.6945,-36.05,-17.11,-13)
	ENDIF
ENDSUB

SUB end_effector_control()



	'''''''大臂传推杆'''''''




	BASE(13)
	IF REMAIN_BUFFER(13)>0 THEN   '返回可以剩余使用的缓冲个数,当返回 0 时表示当前轴的缓冲空间满，此时如果继续调用当前轴的运动指令会阻塞任务直到缓冲有空位。
		?"推杆运动"
		
		FORCE_SPEED(13)=80   ' 第一段速度
		'STARTMOVE_SPEED(13)=100   '第一段起始速度
		ENDMOVE_SPEED(13)=500   '第一段结束速度
		MOVESP(40) AXIS(13)  'SP 速度应用于插补运动指令带 SP 后缀之后的指令（例如 MOVESP、 MOVECICRSP），此时运动速度'采用 FORCE_SPEED 参数，而不是 SPEED 参数。
		
		FORCE_SPEED(13)=500   ' 第二段速度
		'STARTMOVE_SPEED(13)=500   '第二段起始速度
		ENDMOVE_SPEED(13)=500   '第二段结束速度
		MOVESP(500) AXIS(13)  'SP 速度应用于插补运动指令带 SP 后缀之后的指令（例如 MOVESP、 MOVECICRSP），此时运动速度'采用 FORCE_SPEED 参数，而不是 SPEED 参数。
	

		
		FORCE_SPEED(13)=100    ' 第四段速度
		'STARTMOVE_SPEED(13)=100   '第四段起始速度
		ENDMOVE_SPEED(13)=10
		MOVESP(90) AXIS(13)
		?"正运动"
		MOVE_DELAY(500)'等待吸盘抓取
		
		
		
'负向运动分段速度控制部分
'		FORCE_SPEED(13)=120 
'		ENDMOVE_SPEED(13)=500
'		MOVESP(-40) AXIS(13)
'		
'		FORCE_SPEED(13)=500
'		ENDMOVE_SPEED(13)=1100
'		MOVESP(-50) AXIS(13)
'		
'		FORCE_SPEED(13)=1100
'		ENDMOVE_SPEED(13)=400
'		MOVESP(-450) AXIS(13)
'		
'
'		
'		CREEP=200
'		DATUM_IN=9 '输入 IN9 作为原点开关     
'		INVERT_IN(9,OFF) '反转 IN9 电平信号，常开信号进行反转(ZMC 控制器)     ON是反转 OFF是不反转，这条指令应该可以不要？                
'		'通用输入口设置为原点开关信号， 1 无效。
'		'设置了原点开关后， ZMC 控制器输入 OFF 时认为有信号输入，要相反效果可以用
'		'INVERT_IN() '反转电平。 (ECI 系列控制器除外)。
'		TRIGGER   '自动触发示波器
'
'		DATUM(3)'模式3下找原点'   DATUM单轴找原点运动  以speed的速度
'		
'		WAITIDLE
'		?"负运动"
		
		
		
'		FORCE_SPEED(13)=50 
'		ENDMOVE_SPEED(13)=50
'		MOVESP(-40) AXIS(13)
'		
'		FORCE_SPEED(13)=100
'		ENDMOVE_SPEED(13)=100
'		MOVESP(-50) AXIS(13)
'		
'		FORCE_SPEED(13)=100
'		ENDMOVE_SPEED(13)=100
'		MOVESP(-450) AXIS(13)
'		
'
'		
'		CREEP=50
'		DATUM_IN=9 '输入 IN9 作为原点开关     
'		INVERT_IN(9,OFF) '反转 IN9 电平信号，常开信号进行反转(ZMC 控制器)     ON是反转 OFF是不反转，这条指令应该可以不要？                
'		'通用输入口设置为原点开关信号， 1 无效。
'		'设置了原点开关后， ZMC 控制器输入 OFF 时认为有信号输入，要相反效果可以用
'		'INVERT_IN() '反转电平。 (ECI 系列控制器除外)。
'		TRIGGER   '自动触发示波器
'
'		DATUM(3)'模式3下找原点'   DATUM单轴找原点运动  以speed的速度
'		
'		WAITIDLE
'		?"负运动"
		
		
		
		
		'MOVE_DELAY(1000)   '完成一个节拍，等待下一次抓取

	ENDIF

'批量前移，shift+tab

ENDSUB		
	
SUB conveyer_control()
	
	'b=IN(9)
	'PRINT "IN9:" IN(9)
	
	BASE(14)  '大臂传送带
	REP_DIST = 100000000   '设置坐标循环范围
	RS_LIMIT = -200000000 '负向软限位，必须大于-REP_DIST 才会生效
	FS_LIMIT = 200000000 '正向软限位，必须小于 REP_DIST 才会生效
	DPOS(14)=0
	MPOS(14)=0   '编码器反馈位置
	UNITS(14)=552731
	SPEED=200         '启动轴3,实际走了约117毫米/秒
	ACCEL(13)=2000  '加速度
	DECEL(13)=2000  '减速度
	VMOVE(1) AXIS(14)
	
	
'	
'	
'	''''''''小臂传送带'''''''
	BASE(12)
	REP_DIST = 100000000   '设置坐标循环范围
	RS_LIMIT = -200000000'负向软限位，必须大于-REP_DIST 才会生效
	FS_LIMIT = 200000000 '正向软限位，必须小于 REP_DIST 才会生效
	DPOS(12)=0
	MPOS(12)=0   '编码器反馈位置
	UNITS(12)=552731   '轴直径58mm
	ACCEL(13)=2000  '加速度
	DECEL(13)=2000  '减速度
	
	
	BASE(15)
	REP_DIST = 100000000   '设置坐标循环范围
	RS_LIMIT = -200000000'负向软限位，必须大于-REP_DIST 才会生效
	FS_LIMIT = 200000000 '正向软限位，必须小于 REP_DIST 才会生效
	DPOS(15)=0
	MPOS(15)=0   '编码器反馈位置
	UNITS(15)=125424   '轴直径106.5mm 周长334.41，
	ACCEL(15)=2000  '加速度
	DECEL(15)=2000  '减速度
	SPEED=400 
	VMOVE(1) AXIS(15)
'	
'	
'	WHILE 1
'		IF IN(9) > 0 THEN
'			
'			SPEED = 500
'			
'			VMOVE(1) AXIS(12)
'			
'			'?"速度为700"
'			
'		ELSE 
'			SPEED = 150
'			VMOVE(1) AXIS(12)
'			'?"速度为100"
'		ENDIF
'		
'
'	WEND
ENDSUB

'SUB pressure_control()
'	WHILE 1
'		IF DPOS(13) > 500 THEN
'			OP(1,OFF)
'			OP(0,ON)
''		ELSEIF DPOS(13) > 400 AND DPOS(13)  < 500 THEN
''			OP(1,ON)
''			OP(0,OFF)
'		ELSEIF DPOS(13)< 200 THEN 
'			OP(1,OFF)
'			OP(0,OFF)
''		ENDIF
'		ELSEIF IN(7) > 0 THEN
'			'DELAY (150)
'			OP(1,ON) 	
'			OP(0,OFF)
'		ENDIF
'			
'	WEND
'ENDSUB



SUB pressure_control()
	WHILE 1
		IF DPOS(13) > 500 THEN
			OP(1,OFF)
			OP(0,ON)
'		ELSEIF DPOS(13) > 400 AND DPOS(13)  < 500 THEN
'			OP(1,ON)
'			OP(0,OFF)
		ELSEIF DPOS(13)< 200 THEN 
			OP(1,OFF)
			OP(0,OFF)
'		ENDIF
		ELSEIF IN(11) > 0 AND DPOS(13)<380  THEN
			'DELAY (50)
			OP(1,ON) 	
			OP(0,OFF)
		ENDIF
			
	WEND
ENDSUB



SUB end_serv_control()
	WHILE 1
		PRINT "后右侧货箱检测光电传感器：" IN(13)
		IF IN(12)>0 AND IN(13) > 0  THEN  ' IN12 下降沿触发,两个上升沿同时满足比较难，写成一个
			PRINT "后右侧货箱检测光电传感器：" IN(13)
		
			PRINT "货箱到达尾部"
		
			PRINT "货箱经过尾部"
			end_effector_control()
		ENDIF
	WEND
ENDSUB




SUB identification()


		
	BASE(20,21,22,23,24,25) 				'选择虚拟轴号
	'CONNREFRAME(121,10,0,1,2,3,4,5) 		'启动正解连接。
	CONNREFRAME(123,50,26,27,28,29,30,31) 		'启动正解连接。
	
	WAIT LOADED
	PRINT "frame状态：" MTYPE(20)
	
	
	
	
	
	?"零位编码器值："VR(0),VR(1),VR(2),VR(3),VR(4)
	for i = 0 to 4
	
		dpos(i+26) = (encoder(i) - vr(i))/units(i)
		mpos(i+26) = dpos(i+26)
	next
	
	
'	DPOS(26) = (encoder(0) - 6860899)/units(0)
'	DPOS(27) = (encoder(1) - 1407606)/units(1)
'	DPOS(28) = (encoder(2) - 6241935)/units(2)
'	DPOS(29) = (encoder(3) - 242706)/units(3)
'	DPOS(30) = (encoder(4) - 272616)/units(4)
	
	
	?"DPOS(26)=",DPOS(26)
	?"DPOS(27)=",DPOS(27)
	?"DPOS(28)=",DPOS(28)
	?"DPOS(29)=",DPOS(29)
	?"DPOS(30)=",DPOS(30)

	WHILE 1	
		
		IF MTYPE(20)=34 THEN   '判断是否处于逆解状态  正解是 mtype(6) = 34， 逆解状态判断 Mtype(0) = 33
			BASE(26,27,28,29,30,31)
			

			table(350,DPOS(26),DPOS(27),DPOS(28),DPOS(29),DPOS(30),0)
			'table(350,0,25,0,0,0,0)
			'BASE(20,21,22,23,24,25)
			frame_trans2(350,450,1)
			?"正解返回："frame_trans2(350,450,1)'-1有解0无解
		 
			?"x=",TABLE(450)
			?"y=",TABLE(451)
			?"z=",TABLE(452)
			?"rx=",TABLE(453) 
			?"ry=",TABLE(454)
			?"rz=",TABLE(455)

			
		ENDIF

	WEND

	
endsub


SUB lidar_scan()
	
	
	WHILE 1
		
		'IF SCAN_EVENT(IN(6)) > 0   OR   SCAN_EVENT(IN(7))  > 0 THEN
		'''''IN(6) 左右机械臂避障   IN(7)  下方机械臂避障   IN(14)  急停信号
		
		'PRINT "IN14: "IN(14)
		IF IN(15) > 0 THEN
		
			IF  IN(6) > 0 OR  IN(7)  > 0  OR IN(16) > 0 THEN
				PRINT "SAFE ISSUE"
				BASE(0,1,2,3,4,12,13,14)
				RAPIDSTOP (2)
				PRINT "emergency stop"
				'HALT
				
				
			ENDIF
		ENDIF
	
	WEND
endsub
'WAIT UNTIL DPOS(0)>1000 '等待轴 0 运行到 1000



SUB end_effector_rotate()
	WHILE 1
		'WAIT UNTIL IN(11) > 0
		'WAIT UNTIL IN(11) < 0 
		if IN(11) > 0 THEN
			BASE(4)
			MOVEABS(-8)
		ENDIF
	
	
	WEND
ENDSUB


'INVERT_IN(5,ON) '反转 IN5 电平信号，常开信号进行反转(ZMC 控制器)
