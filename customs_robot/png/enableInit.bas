RAPIDSTOP(2)  '总线启动。


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
		AXIS_ADDRESS(3)=0+7
		AXIS_ADDRESS(4)=0+8
		
		AXIS_ADDRESS(12)=0+4
		AXIS_ADDRESS(13)=0+5
		AXIS_ADDRESS(14)=0+6
		
		
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
				'DRIVE_CONTROLWORD(5)=0
				'DRIVE_CONTROLWORD(6)=0  
				
                DELAY (10)

                DATUM(0)                                                      '单轴找原点运动。ZMC 控制器为 0 触发有效，输入为 OFF 状态时，表示到达原点/限位

                DELAY (100)         

                ?"轴使能准备"
				AXIS_ENABLE(0)=1  
				AXIS_ENABLE(1)=1
                AXIS_ENABLE(2)=1                   '轴使能
				AXIS_ENABLE(3)=1   
				AXIS_ENABLE(4)=1 
				AXIS_ENABLE(12)=1 
				AXIS_ENABLE(13)=1
				AXIS_ENABLE(14)=1
				'AXIS_ENABLE(5)=1  
				'AXIS_ENABLE(6)=1  
                WDOG=1                                                        '使能总开关

                ?"轴使能完成"

ELSE

                ?"总线开启失败"

        ENDIF

ELSE

        ?"总线扫描失败"

ENDIF


DELAY(5000)


'BASE(14)
'UNITS(14)=552730
'SPEED=50             '启动轴3
'VMOVE(-1) AXIS(14)     '-1 负向运动， 1 正向运动

'5417，0，1458
'5418，450，1160
'5418，60，1160
'5418，-330，1160
'
'5418，-330，980
'5418，60，980
'5418，450，980
'
'5418，450，800
'5418，60，800
'5418，-330，800



'''''电机、机械手参数定义




DIM baseL,L1,L2,L3,L4,L5,L6,L7,L8,L9,L10,L11,L12,crank(3),dok(4)
baseL = 355
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
L12 = 1189.44 

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
FS_LIMIT = 15,38,35.5,18,15,15        '这里的上下限位是角位移么？ 
RS_LIMIT = -15,-35,-61.5,-18,-15,-15
'DPOS=0,0,0,0,0,0
'MPOS=0,0,0,0,0,0



UNITS = Pules1OneCircle/360,Pules2OneCircle/360,Pules3OneCircle/360,Pules4OneCircle/360,Pules5OneCircle/360,Pules6OneCircle/360   '把units设成每°脉冲数
'DPOS=0,0,0,0,0,0
'MPOS= 0,0,0,0,0,0                             '设置关节轴的位置，此处要根据实际情况来修改。



speed=5,5,5,5,5,5                        '速度参数设置,5已经是最小速度
accel=50,50,50,50,50,50
decel=50,50,50,50,50,50
FASTDEC=1000,1000,1000,1000,1000,1000
CLUTCH_RATE=1000,1000,1000,1000,1000,1000                'CONNECT 连接的速度，用于定义连结率从 0 到设置倍率的改变时间， ratio/秒

'CONNECT,同步运动指令,将当前轴的目标位置与 driving_axis 轴的测量位置通过电子齿轮连接。
BASE(3,4)
'DPOS=0,0
'MPOS=0,0




BASE(12)
'DPOS=0,0
'MPOS=0,0
UNITS(12) = 552370 
ACCEL(12) = 1000
DECEL(12) = 1000

RS_LIMIT = -20000000 '负向软限位，必须大于-REP_DIST 才会生效
FS_LIMIT = 200000000

BASE(13)   '选择轴号2

'电机编码器23位，分辨率8388608，一个编码器分辨率总数电机转一圈，脉冲当量=8388608/360=23301  需要考虑减速比吗？

UNITS(13)=534306   '脉冲当量552731   脉冲当量一般设为电机走 1mm 或 1°的所需脉冲数。脉冲当量p= e*k/L  e编码器分辨率8388608，L为同步带周长，k为减速比
SPEED(13)=450  '速度设置为1000
CREEP(13)=50   ' 找原点反向爬行速度，速度为700，这里是归零？
ACCEL(13)=2000  '加速度
DECEL(13)=2000  '减速度
REP_DIST = 100000000   '设置坐标循环范围
RS_LIMIT = 0 '负向软限位，必须大于-REP_DIST 才会生效
FS_LIMIT = 580 '正向软限位，必须小于 REP_DIST 才会生效
'DPOS(13)=0
'MPOS(13)=0   '编码器反馈位置
MERGE = ON   '启动连续插补
SRAMP=200    '加减速变化率，使得速度曲线平滑，在机械启停或加减速时减少抖动，范围在0-250毫秒之间
FASTDEC(13)=300   '快减减速度，达到限位或异常停止时自动采用。
IF IN(9) > 0 THEN
	?"已在原点！"
ELSE
	DATUM_IN=9 '输入 IN9 作为原点开关     
	INVERT_IN(9,OFF) '反转 IN9 电平信号，常开信号进行反转(ZMC 控制器)     ON是反转 OFF是不反转，这条指令应该可以不要？                
	'通用输入口设置为原点开关信号， 1 无效。
	'设置了原点开关后， ZMC 控制器输入 OFF 时认为有信号输入，要相反效果可以用
	'INVERT_IN 反转电平。 (ECI 系列控制器除外)。
	TRIGGER   '自动触发示波器

	DATUM(3)'模式3下找原点'   DATUM单轴找原点运动  以speed的速度
	?"正在复位！"
	WAITIDLE
	?"复位成功！原点已回零！"
ENDIF
DPOS(13)=0 '轴13推杆的位置已清零'
MPOS(13)=0


''''''大臂传送带'''''''
BASE(14)
UNITS(14) = 552731
ACCEL(14) = 1000
DECEL(14) = 1000
REP_DIST = 100000000   '设置坐标循环范围
RS_LIMIT = -200000000 '负向软限位，必须大于-REP_DIST 才会生效
FS_LIMIT = 200000000 '正向软限位，必须小于 REP_DIST 才会生效
DPOS(14)= 0
MPOS(14) = 0   '编码器反馈位置



BASE(0,1,2)
FWD_IN=0,1,2  '分别设置正向限位开关,输入 IN0 作为轴 0 的正向限位信号
INVERT_IN(0,ON) '反转信号
INVERT_IN(1,ON)  'ZMC 系列控制器输入 ON 时认为有信号输入
INVERT_IN(2,ON)

BASE(0,1,2)
REV_IN=3,4,5  '分别设置正向限位开关,输入 IN3 作为轴 0 的负向限位信号
INVERT_IN(3,ON) '反转信号
INVERT_IN(4,ON)  'ZMC 系列控制器输入 ON 时认为有信号输入
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
speed=200,100,100,100,100,100                        '速度参数设置  100uits/s
accel=350,1000,1000,1000,1000,1000
decel=350,1000,1000,1000,1000,1000
'decel=1000,1000,1000,1000,1000,1000
'ATYPE=0,0,0,0,0,0                                            '设置为虚拟轴,前面总线初始化那里有设置
'DPOS=0,0,0,0,0,0     
TABLE(10,baseL,L1,L2,L3,L4,L5,L6,L7,L8,L9,L10,L11,L12,crank(0),crank(1),crank(2),dok(0),dok(1),dok(2),dok(3),Pules1OneCircle, Pules2OneCircle, Pules3OneCircle, Pules4OneCircle, Pules5OneCircle , PulesROneCircle,0,0,0,0) '校正的末端参数
'比较点坐标设置   参数存储在 TABLE10 开始的位置  这个TABLE是什么还是不太理解，手册里也有说是数据存储的位置



UNITS=1000,1000,1000,1000,1000,1000         '运动精度，要提前设置，中途不能变化   UNITS脉冲当量



BASE(0,1,2,3,4,5)  '关联关节轴
CONNFRAME(121,10,6,7,8,9,10,11)'逆解模式
WAIT LOADED

'BASE(6,7,8,9,10,11)
'MOVEABS(5417,0,1458,0,0,0)

'''  求解逆解 '''''
'底盘高度170mm

PRINT "frame状态：" MTYPE(0)
IF MTYPE(0)=33 THEN   '判断是否处于逆解状态  正解是 mtype(6) = 34， 逆解状态判断 Mtype(0) = 33
	BASE(6,7,8,9,10,11)
	DIM Z1,Z2,Z3,Z4,Z5,Z6
	Z1=5417.428
	Z2=0
	Z3=1458
	Z4=0  'rx  翻滚
	Z5=0  'ry  俯仰
	Z6=0  'rz  偏航
	table(100,Z1,Z2,Z3,Z4,Z5,Z6)
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
		
		  
		'MOVEABS(TABLE(200),TABLE(201),TABLE(202),TABLE(203),TABLE(204))  'moveabs，绝对运动指令，MOVE_PT，单位时间距离
		
		'FORCE_SPEED = 1,1,1,1,1,1
		'MOVEABSSP(TABLE(200),TABLE(201),TABLE(202),TABLE(203),TABLE(204),0)
		
'		BASE(6,7,8,9,10,11)
'		MOVEABS(Z1,Z2,Z3,Z4,Z5,Z6)
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

END