 
 



''''''''''''''''''''''''��ʼ��׼��

RAPIDSTOP(2)  '��������������˶���������ֹͣ��ǰ�˶�


WAIT IDLE

FOR i=0 to 20                                            'ȡ��ԭ��������������

        ATYPE(i)=0   
		?"111111"

NEXT

''''''''''''''''''''''''EtherCAT���߳�ʼ��


SLOT_SCAN(0)                                             '��ʼɨ��
'SLOT_SCAN (slot)
'slot�������� EtherCAT ��λ�Ż� RTEX ��λ�ţ� 0-ȱʡ

IF RETURN THEN 

        ?"����ɨ��ɹ�","�����豸����"NODE_COUNT(0)

        ?

        ?"��ʼӳ�����"
		AXIS_ADDRESS(0)=0+1
        AXIS_ADDRESS(1)=0+2             'ӳ�����
		AXIS_ADDRESS(2)=0+3 
		AXIS_ADDRESS(3)=0+8
		AXIS_ADDRESS(4)=0+9
		
		AXIS_ADDRESS(12)=0+4
		AXIS_ADDRESS(13)=0+5
		AXIS_ADDRESS(14)=0+6
		AXIS_ADDRESS(15)=0+7
		
		
		'AXIS_ADDRESS (0)=(0<<16)+0+1 '��һ�� ECAT ����������������� 0����Ϊ�� 0
		'AXIS_ADDRESS (2)=(0<<16)+1+1 '�ڶ��� ECAT ����������������� 1����Ϊ�� 2
		'AXIS_ADDRESS (1)=(0<<16)+2+1 '������ ECAT ����������������� 2����Ϊ�� 1
		
		'012Ϊ��۵����ᣬ4ΪС�۴��ʹ���5Ϊ�Ƹˣ�6Ϊ��۴��ʹ���12Ϊ����
		
	
		
		
        ATYPE(0)=65                                 'EtherCAT����
		ATYPE(1)=65                                    '65λ�ÿ��ƣ�66�ٶȿ��ƣ�67���ؿ���
		ATYPE(2)=65									'����Ĺؽ�����0-4,5���ᣬ6-11�������ᣬ��Ӧxyzrpy,5Ҳ�������ᣬΪ���ùؽ����������������Ӧ
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
        DRIVE_PROFILE(1)= -1              '�ŷ�PDO����  �������ʹ�վ֮�����ݴ��䷽ʽ�����֣�һ���ǰ�ָ��ʱ�������Խ������ݣ���֮Ϊ PDO(Process Data Object)
		DRIVE_PROFILE(2)= -1
		DRIVE_PROFILE(3)= -1
		DRIVE_PROFILE(4)= -1
		DRIVE_PROFILE(12)= -1
		DRIVE_PROFILE(13)= -1
		DRIVE_PROFILE(14)= -1
		DRIVE_PROFILE(15)= -1
		'DRIVE_PROFILE(6)= -1
                                             'ATYPE=66ʱ����DRIVE_PROFILE=20

                                             'ATYPE=67ʱ����DRIVE_PROFILE=30
		
		'DRIVE_PROFILE -- ������ PDO ����									 
		'ÿ����ķ��� pdo ���� pdo ������ѡ��
		'����������ȷ�� ATYPE������Ϊ 65/66/67���Ժ���ܲ������������									 
		DISABLE_GROUP(0)
        DISABLE_GROUP(1)                           'ÿ�ᵥ������  ,
		DISABLE_GROUP(2)
		DISABLE_GROUP(3)
		DISABLE_GROUP(4)
		DISABLE_GROUP(12)
		DISABLE_GROUP(13)
		DISABLE_GROUP(14)
		DISABLE_GROUP(15)
		'DISABLE_GROUP(6)
		
		
		'һͣ�࣬�Ѽ���������Ϊһ�飬�������澯���ر����ڵ�����ʹ��(ECAT ��Ʒ֧��)�����������������塣
		
        ?"���ӳ�����"

        DELAY (100)

 

        SLOT_START(0)              '���߿���
		'SLOT_START(0),����������
		'ͨ�� RETURN ���سɹ���񡣷���-1 �����ɹ��� 0 ����ʧ�ܡ�

        IF RETURN THEN 

                ?"���߿����ɹ�"

                ?"��ʼ�������������(���������������ֵ�����)"
				
				DRIVE_CONTROLWORD(0)=0
				DRIVE_CONTROLWORD(1)=0
                DRIVE_CONTROLWORD(2)=0        '����ŷ��������
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
                DRIVE_CONTROLWORD(2)=128       'bit7=1ǿ���ŷ��������
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
                DRIVE_CONTROLWORD(2)=0         '����ŷ��������
				DRIVE_CONTROLWORD(3)=0
				DRIVE_CONTROLWORD(4)=0
				DRIVE_CONTROLWORD(12)=0
				DRIVE_CONTROLWORD(13)=0
				DRIVE_CONTROLWORD(14)=0
				DRIVE_CONTROLWORD(15)=0
				'DRIVE_CONTROLWORD(5)=0
				'DRIVE_CONTROLWORD(6)=0  
				
                DELAY (10)

                DATUM(0)                                                      '������ԭ���˶���ZMC ������Ϊ 0 ������Ч������Ϊ OFF ״̬ʱ����ʾ����ԭ��/��λ

                DELAY (100)         

                ?"��ʹ��׼��"
                'ʹ���ܿ���

				AXIS_ENABLE(0)=1  
				AXIS_ENABLE(1)=1
                AXIS_ENABLE(2)=1                   '��ʹ��
				AXIS_ENABLE(3)=1   
				AXIS_ENABLE(4)=1 
				AXIS_ENABLE(12)=1 
				AXIS_ENABLE(13)=1
				AXIS_ENABLE(14)=1
				AXIS_ENABLE(15)=1
                WDOG=1                                                        'ʹ���ܿ���

                ?"��ʹ�����" 

ELSE

                ?"���߿���ʧ��"

        ENDIF

ELSE

        ?"����ɨ��ʧ��"

ENDIF


DELAY(5000)







'''''�������е�ֲ�������




DIM baseL,L1,L2,L3,L4,L5,L6,L7,L8,L9,L10,L11,L12,crank(3),dok(4),tcp(6)
baseL = 358
'������
L1 = 850
L2 = 1190
L3 = 800
L4 = 500
L5 = 568.25
dok(0) = -ATAN2(122,555) +  PI/2  'double atan2(double y, double x),�����Ի��ȱ�ʾ�� y/x �ķ����С�y �� x ��ֵ�ķ��ž�������ȷ������
dok(1) =  ATAN2(0,1190)   '
dok(2) = -ATAN2(138.6, 787.9)- ATAN2(122,555)  + PI/2 'CDX
dok(3) = -ATAN2(122,555) - ATAN2(59.9,496.4)  '
'�����˵�

'����

'dok(0) =l1��x��ļнǣ�dok(1)=l2��x��нǣ�dok(2)=l3��x��нǣ�dok(3)=l4��x��н�
L6 = 300
L7 = 500
L8 = 557.31
L9 = 462.55
crank(0) = ATAN2(112.09,278.27) + ATAN2(107,450) + PI/2   
crank(1) = ATAN2(164.73,472.09) + ATAN2(107,450) 'CBX
crank(2) = ATAN2(107,450) + ATAN2(90,550) +  PI/2 '
'��۳���С�۳�
L10 = 253 
L11 = 4228
L12 = 0

'tcp(0, 1184 , 0, 139.25, 0, 0, 0)
'tcp(0, 1207 , 0, 114.78, 0, 0, 0)
'tcp(0, 1211 , 0, 102, 0, 0, 0)
tcp(0, 1216.96 , -2, 96, 0, 0, 0)'20230725�궨TCP

dim u_j1,u_j2,u_j3,u_j4,u_j5,u_j6                '�ؽڵ��1-6ʵ��һȦ������
u_j1=8388608  '������123��23λ��45��19λ
u_j2=8388608
u_j3=8388608
u_j4=524288
u_j5=524288
u_j6=524288

dim i_1,i_2,i_3,i_4,i_5,i_6                '�ؽ�1-6������    123�ᰴ�յ��ʵ�ʲ������ã�45��������ƿعؽ�ģ�飬������ض˱�������λ�÷������ǳ����˼��ٱ��Ժ�ģ����Լ��ٱ�Ϊ1  
i_1=205
i_2=200
i_3=200
'i_1=1
'i_2=1
'i_3=1
i_4=1
i_5=1
i_6=1
''����������˵����ã���Ϊ��������ȡ�Ĺؽ������λ���Ǿ������ٱȺ��λ�÷���
dim Pules1OneCircle, Pules2OneCircle, Pules3OneCircle, Pules4OneCircle, Pules5OneCircle, Pules6OneCircle, PulesROneCircle 'ÿ���ؽ�ת��һȦ��ʵ��������
Pules1OneCircle=u_j1*i_1
Pules2OneCircle=u_j2*i_2
Pules3OneCircle=u_j3*i_3
Pules4OneCircle=u_j4*i_4
Pules5OneCircle=u_j5*i_5
Pules6OneCircle=u_j6*i_6


PulesROneCircle=360*1000

'''''�ؽ�������
BASE(0,1,2,3,4,5)                                'ѡ��ؽ����0��1��2��3��4��5(��5����ؽ�5�����ܶ���atype��� = 0)
'BASE(1,2,3,4,5,6)
'FS_LIMIT = 360,180,180,180,360,360        '�����������λ�ǽ�λ��ô�� 
'RS_LIMIT = -360,-180,-180,-180,-360,-360
 
'atype=0,0,0,0,0,0                               '�����ͣ�0Ϊ������
'atype=65,65,65,65,65,0                             '��������ΪEtherCAT����λ��ģʽ 
'FS_LIMIT = 15,38,35.5,18,15,15        '�����������λ�ǽ�λ��ô�� 
'RS_LIMIT = -15,-35,-61.5,-18,-15,-15


FS_LIMIT = 15,38,35.5,10,15,15        '�����������λ�ǽ�λ��ô�� 
RS_LIMIT = -15,-35,-61.5,-26,-15,-15

UNITS = Pules1OneCircle/360,Pules2OneCircle/360,Pules3OneCircle/360,Pules4OneCircle/360,Pules5OneCircle/360,Pules6OneCircle/360   '��units���ÿ��������
'DPOS=0,0,0,0,0,0

'MPOS= 0,0,0,0,0,0                             '���ùؽ����λ�ã��˴�Ҫ����ʵ��������޸ġ�



'20230906�������궨
vr(0) = -133805639
vr(1) = 219048662
vr(2) = 12240320
vr(3) = 191923  '���ּ�С С������̧
vr(4) = 288154
vr(13)= -19656000



for i = 0 to 4
	
	dpos(i) = (encoder(i) - vr(i))/units(i)
	mpos(i) = dpos(i)
next
?"dpos��ʼ�����"
Print "DPOS: " DPOS(0),DPOS(1),DPOS(2),DPOS(3),DPOS(4)
dpos(13)= (encoder(13) -vr(13))/534306
MPOS(13)= DPOS(13)





speed=5,5,5,5,5,5                        '�ٶȲ�������,5�Ѿ�����С�ٶ�
accel=10,10,10,10,10,10   				'���ü��ٶȺͼ��ٶȣ��˴���50�ĳ�10�����ٵ�5��ʱ0.5s�����ٶȼ��ٶȽ����Ժ�������������˶����ƹ�����ĩ�˵Ĵ���ڶ�
decel=100,100,100,100,100,100

'accel=50,50,50,50,50,50
'decel=50,50,50,50,50,50
FASTDEC=100,100,100,100,100,100
CLUTCH_RATE=1000,1000,1000,1000,1000,1000                'CONNECT ���ӵ��ٶȣ����ڶ��������ʴ� 0 �����ñ��ʵĸı�ʱ�䣬 ratio/��

'CONNECT,ͬ���˶�ָ��,����ǰ���Ŀ��λ���� driving_axis ��Ĳ���λ��ͨ�����ӳ������ӡ�

BASE(0,1,2)
FWD_IN=0,1,2  '�ֱ�����������λ����,���� IN0 ��Ϊ�� 0 ��������λ�ź�
INVERT_IN(0,ON) '��ת�ź�
INVERT_IN(1,ON)  'ZMC ϵ�п��������� OFF ʱ��Ϊ���ź�����
INVERT_IN(2,ON)

BASE(0,1,2)
REV_IN=3,4,5  '�ֱ����ø�����λ����,���� IN3 ��Ϊ�� 0 �ĸ�����λ�ź�
INVERT_IN(3,ON) '��ת�ź�
INVERT_IN(4,ON)  'ZMC ϵ�п��������� OFF ʱ��Ϊ���ź�����
INVERT_IN(5,ON)





merge=on                                         '���������岹  �����岹�Ļ������˶�֮�䲻���������岹���˶��켣����ƽ��
'merge=off
corner_mode = 2                                '�����սǼ��� �Զ��սǼ���,�� ACCEL,DECEL �Ӽ��ٶ�,�˲������� MOVE ��������ǰ��Ч

decel_angle = 15 * (PI/180)                '��ʼ���ٵĽǶ� 15��   �˶�15�������ǲ����٣�15-45���ǵȱȼ��٣�45����������ȫ����
'��ȫ�����ǰ���decel�ļ��ٶȼ����𣿵ȱȼ����������ģ�  �ֲ���˵�ȱȼ�����FORCE_SPEEDΪ�ο�ֵ��FORCE_SPEED�ڵȱȼ���ʱ������
stop_angle = 45 * (PI/180)                '��������ٶȵĽǶ�45��

'?"�ؽ���DPOS��"DPOS

'''''''''''''''''''''����������'''''''''''''''''''''
BASE(6,7,8,9,10,11)
'BASE(7,8,9,10,11,12)
FS_LIMIT = 200000,200000,20000,20000,200000
RS_LIMIT = -200000,-200000,-20000,-20000,-200000



speed=230,230,230,230,230,230                        '�ٶȲ�������  100uits/s
accel=250,250,250,250,250,250
decel=250,250,250,250,250,250
FASTDEC=1000,1000,1000,1000,1000,1000
'decel=1000,1000,1000,1000,1000,1000
'ATYPE=0,0,0,0,0,0                                            '����Ϊ������,ǰ�����߳�ʼ������������
DPOS=0,0,0,0,0,0     
TABLE(10,baseL,L1,L2,L3,L4,L5,L6,L7,L8,L9,L10,L11,L12,crank(0),crank(1),crank(2),dok(0),dok(1),dok(2),dok(3),Pules1OneCircle, Pules2OneCircle, Pules3OneCircle, Pules4OneCircle, Pules5OneCircle , PulesROneCircle,tcp(0),tcp(1),tcp(2),tcp(3),tcp(4),tcp(5))

'�Ƚϵ���������   �����洢�� TABLE10 ��ʼ��λ��  ���TABLE��ʲô���ǲ�̫��⣬�ֲ���Ҳ��˵�����ݴ洢��λ��



UNITS=1000,1000,1000,1000,1000,1000         '�˶����ȣ�Ҫ��ǰ���ã���;���ܱ仯   UNITS���嵱��




BASE(20,21,22,23,24,25)  'frame 123 ������
FS_LIMIT = 200000,200000,20000,20000,200000
RS_LIMIT = -200000,-200000,-20000,-20000,-200000
speed=100,100,100,100,100,100                        '�ٶȲ�������  100uits/s
accel=1000,1000,1000,1000,1000,1000
decel=1000,1000,1000,1000,1000,1000
'DPOS=0,0,0,0,0,0  
UNITS=1000,1000,1000,1000,1000,1000         '�˶����ȣ�Ҫ��ǰ���ã���;���ܱ仯   UNITS���嵱��
ATYPE= 0,0,0,0,0,0

TABLE(50,baseL,L1,L2,L3,L4,L5,L6,L7,L8,L9,L10,L11,L12,crank(0),crank(1),crank(2),dok(0),dok(1),dok(2),dok(3),Pules1OneCircle, Pules2OneCircle, Pules3OneCircle, Pules4OneCircle, Pules5OneCircle , PulesROneCircle,0,0,0,0) 
BASE(26,27,28,29,30,31)  'frame 123 �ؽ���
FS_LIMIT = 200000,200000,20000,20000,200000
RS_LIMIT = -200000,-200000,-20000,-20000,-200000
speed=100,100,100,100,100,100                        '�ٶȲ�������  100uits/s
accel=1000,1000,1000,1000,1000,1000
decel=1000,1000,1000,1000,1000,1000
'DPOS=0,0,0,0,0,0  
UNITS=Pules1OneCircle/360,Pules2OneCircle/360,Pules3OneCircle/360,Pules4OneCircle/360,Pules5OneCircle/360,Pules6OneCircle/360          '�˶����ȣ�Ҫ��ǰ���ã���;���ܱ仯   UNITS���嵱��
ATYPE= 0,0,0,0,0,0

'CONNECT(1,0) AXIS(26)  '���ӳ�������frame123�Ĺؽ���30��ʵ�ʵĹؽ���0
'CONNECT(1,1) AXIS(27)
'CONNECT(1,2) AXIS(28)
'CONNECT(1,3) AXIS(29)
'CONNECT(1,4) AXIS(30)

ADDAX(0)  AXIS(26) '�˶����ӣ����ӵ����������
ADDAX(1)  AXIS(27) 
ADDAX(2)  AXIS(28) 
ADDAX(3)  AXIS(29) 
ADDAX(4)  AXIS(30) 




'		
BASE(13)   'ѡ�����2

'���������23λ���ֱ���8388608��һ���������ֱ����������תһȦ�����嵱��=8388608/360=23301  ��Ҫ���Ǽ��ٱ���

UNITS(13)=534306   '���嵱��552731   ���嵱��һ����Ϊ����� 1mm �� 1������������������嵱��p= e*k/L  e�������ֱ���8388608��LΪͬ�����ܳ���kΪ���ٱ�
SPEED(13)=450  '�ٶ�����Ϊ1000
CREEP(13)=150   ' ��ԭ�㷴�������ٶȣ��ٶ�Ϊ700�������ǹ��㣿
ACCEL(13)=2000  '���ٶ�
DECEL(13)=2000  '���ٶ�
REP_DIST = 100000000   '��������ѭ����Χ
'RS_LIMIT = -5 '��������λ���������-REP_DIST �Ż���Ч
'FS_LIMIT = 580 '��������λ������С�� REP_DIST �Ż���Ч
''DPOS(13)=0
''MPOS(13)=0   '����������λ��
'MERGE = ON   '���������岹
'corner_mode = 2 
'SRAMP=100    '�Ӽ��ٱ仯�ʣ�ʹ���ٶ�����ƽ�����ڻ�е��ͣ��Ӽ���ʱ���ٶ�������Χ��0-250����֮��
'FASTDEC(13)=300   '������ٶȣ��ﵽ��λ���쳣ֹͣʱ�Զ����á�
'IF IN(9) > 0 THEN
'	?"����ԭ�㣡"
'ELSE
'	DATUM_IN=9 '���� IN9 ��Ϊԭ�㿪��     
'	INVERT_IN(9,OFF) '��ת IN9 ��ƽ�źţ������źŽ��з�ת(ZMC ������)     ON�Ƿ�ת OFF�ǲ���ת������ָ��Ӧ�ÿ��Բ�Ҫ��                
'	'ͨ�����������Ϊԭ�㿪���źţ� 1 ��Ч��
'	'������ԭ�㿪�غ� ZMC ���������� OFF ʱ��Ϊ���ź����룬Ҫ�෴Ч��������
'	'INVERT_IN ��ת��ƽ�� (ECI ϵ�п���������)��
'	TRIGGER   '�Զ�����ʾ����
'
'	DATUM(3)'ģʽ3����ԭ��'   DATUM������ԭ���˶�  ��speed���ٶ�
'	?"���ڸ�λ��"
'	WAITIDLE
'	?"��λ�ɹ���ԭ���ѻ��㣡"
'ENDIF
'DPOS(13)=0 '��13�Ƹ˵�λ��������'
'MPOS(13)=0
'?"�Ƹ�λ���Ѹ�λ���㣡"

DELAY(1000)



'RUNTASK 1, conveyer_control() '���ʹ����Ƶ����̣߳�����С�ۺʹ�۵Ĵ��ʹ�
'RUNTASK 2, pressure_control()  '���ʹ����Ƶ����̣߳�����С�ۺʹ�۵Ĵ��ʹ�
'RUNTASK 3, end_serv_control()
RUNTASK 4, identification()    '������۱궨���̣߳��������ǰ��������
'RUNTASK 5, lidar_scan()        '�״�������̣߳�����������е���������Ҳ�������ײ
'RUNTASK 6, end_effector_rotate()  'װ��ʱ��ĩ�˷���ƫת�����߳�


'''��ʼλ�ã���λ




DIM Z1,Z2,Z3,Z4,Z5,Z6
'
'
''''ԭ��''''
'Z1=5400
'Z2=800
'Z3=1800
'Z4=0   ' rx  ����
'Z5=0   ' ry  ����
'Z6=0   ' rz  ƫ��
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
'Z4=0    rx  '����
'Z5=0    ry ' ����
'Z6=0    rz  'ƫ��
'manipulator_control(Z1,Z2,Z3,Z4,Z5,Z6)
'WAITIDLE
'MOVE_DELAY(2000)
'''''ԭ��''''
'
''
'Z1=5412
'Z2=-800
'Z3=1400
'Z4=0   ' rx  ����
'Z5=0   ' ry  ����
'Z6=0   ' rz  ƫ��
'manipulator_control(Z1,Z2,Z3,Z4,Z5,Z6)
'WAITIDLE
'MOVE_DELAY(2000)



'Z1=5212
'Z2=-700
'Z3=1300
'Z4=0   ' rx  ����
'Z5=0   ' ry  ����
'Z6=0   ' rz  ƫ��
'manipulator_control(Z1,Z2,Z3,Z4,Z5,Z6)
'WAITIDLE
'MOVE_DELAY(2000)
''
''
'DELAY(2000)

DIM a,b,c

a=AIN(0) '��ȡͨ�� 1 �� AD
a=AIN(0) *10 /4095 'ͨ�� 1 �ĵ�ѹֵ     ''ʵ�ʵ�ѹ0.278V������410mm
PRINT "��ǰ��ѹ��" a
b = a*1000*0.099 + 98.586  '��Ӧ�ľ���
c = b*0.7985
'

PRINT "��ഫ������⵽�ľ���" b 

PRINT "ֱ�߾���" c


'
''
'
''
'''
'''��һ������
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
''''''�ڶ�������
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
'''''''����������
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
'IF SCAN_EVENT(IN(12)) > 0   THEN  ' IN12 IN13�����ش���
'	PRINT "���侭��β��"
'	end_effector_control()
'ENDIF
''''''
''''''
'''''''���ĸ�����
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
'IF SCAN_EVENT(IN(12)) > 0 AND SCAN_EVENT(IN(13)) > 0 THEN  ' IN12 IN13�����ش���
'	end_effector_control()
'ENDIF
''''''
'''''''���������
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
'IF SCAN_EVENT(IN(12)) > 0 AND SCAN_EVENT(IN(13)) > 0 THEN  ' IN12 IN13�����ش���
'	end_effector_control()
'ENDIF
''''''
''''''
'''''''����������
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
'IF SCAN_EVENT(IN(12)) > 0 AND SCAN_EVENT(IN(13)) > 0 THEN  ' IN12 IN13�����ش���
'	end_effector_control()
'ENDIF
''''''
'''''''���߸�����
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
'''''''�ڰ˸�����
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
'''''''�ھŸ�����
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
'Z4=0   ' rx  ����
'Z5=0   ' ry  ����
'Z6=0   ' rz  ƫ��
'manipulator_control(Z1,Z2,Z3,Z4,Z5,Z6)
'WAITIDLE
'MOVE_DELAY(2000)



END

SUB manipulator_control(t1,t2,t3,t4,t5,t6)

	BASE(0,1,2,3,4,5)  '�����ؽ���
	CONNFRAME(121,10,6,7,8,9,10,11)'���ģʽ
	WAIT LOADED

	'���̸߶�170mm

	PRINT "frame״̬��" MTYPE(0)
	IF MTYPE(0)=33 THEN   '�ж��Ƿ������״̬  ������ mtype(6) = 34�� ���״̬�ж� Mtype(0) = 33
		BASE(6,7,8,9,10,11)
		PRINT "t1-t6:"t1,t2,t3,t4,t5,t6

		table(100,t1,t2,t3,t4,t5,t6)
		frame_trans2(100,200,0)
		?"��ⷵ�أ�"frame_trans2(100,200,0)'-1�н�0�޽�

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
			MOVEABS(TABLE(200),TABLE(201),TABLE(202),TABLE(203),TABLE(204))  'moveabs�������˶�ָ�MOVE_PT����λʱ�����
			?"errswitch"
			TRACE "DPOS(0) =" DPOS(0)
			TRACE "DPOS(1) =" DPOS(1)
			TRACE "DPOS(2) =" DPOS(2)
			TRACE "DPOS(3) =" DPOS(3)
			TRACE "DPOS(4) =" DPOS(4)
			

			?"λ�ÿɴ�"
		ELSE
			?"������е�޶���Χ"
			
		ENDIF
		
	ELSE
		?"û�н������״̬������ᱨ��"

	ENDIF


	''CONNREFRAME(121,10,0,1,2,3,4,5)'����ģʽ
	''BASE(0,1,2,3,4,5)
	''MOVEABS(0,0,0,0,0,0)  'MOVEABS,�ؽ�������˶�
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



	'''''''��۴��Ƹ�'''''''




	BASE(13)
	IF REMAIN_BUFFER(13)>0 THEN   '���ؿ���ʣ��ʹ�õĻ������,������ 0 ʱ��ʾ��ǰ��Ļ���ռ�������ʱ����������õ�ǰ����˶�ָ�����������ֱ�������п�λ��
		?"�Ƹ��˶�"
		
		FORCE_SPEED(13)=80   ' ��һ���ٶ�
		'STARTMOVE_SPEED(13)=100   '��һ����ʼ�ٶ�
		ENDMOVE_SPEED(13)=500   '��һ�ν����ٶ�
		MOVESP(40) AXIS(13)  'SP �ٶ�Ӧ���ڲ岹�˶�ָ��� SP ��׺֮���ָ����� MOVESP�� MOVECICRSP������ʱ�˶��ٶ�'���� FORCE_SPEED ������������ SPEED ������
		
		FORCE_SPEED(13)=500   ' �ڶ����ٶ�
		'STARTMOVE_SPEED(13)=500   '�ڶ�����ʼ�ٶ�
		ENDMOVE_SPEED(13)=500   '�ڶ��ν����ٶ�
		MOVESP(500) AXIS(13)  'SP �ٶ�Ӧ���ڲ岹�˶�ָ��� SP ��׺֮���ָ����� MOVESP�� MOVECICRSP������ʱ�˶��ٶ�'���� FORCE_SPEED ������������ SPEED ������
	

		
		FORCE_SPEED(13)=100    ' ���Ķ��ٶ�
		'STARTMOVE_SPEED(13)=100   '���Ķ���ʼ�ٶ�
		ENDMOVE_SPEED(13)=10
		MOVESP(90) AXIS(13)
		?"���˶�"
		MOVE_DELAY(500)'�ȴ�����ץȡ
		
		
		
'�����˶��ֶ��ٶȿ��Ʋ���
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
'		DATUM_IN=9 '���� IN9 ��Ϊԭ�㿪��     
'		INVERT_IN(9,OFF) '��ת IN9 ��ƽ�źţ������źŽ��з�ת(ZMC ������)     ON�Ƿ�ת OFF�ǲ���ת������ָ��Ӧ�ÿ��Բ�Ҫ��                
'		'ͨ�����������Ϊԭ�㿪���źţ� 1 ��Ч��
'		'������ԭ�㿪�غ� ZMC ���������� OFF ʱ��Ϊ���ź����룬Ҫ�෴Ч��������
'		'INVERT_IN() '��ת��ƽ�� (ECI ϵ�п���������)��
'		TRIGGER   '�Զ�����ʾ����
'
'		DATUM(3)'ģʽ3����ԭ��'   DATUM������ԭ���˶�  ��speed���ٶ�
'		
'		WAITIDLE
'		?"���˶�"
		
		
		
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
'		DATUM_IN=9 '���� IN9 ��Ϊԭ�㿪��     
'		INVERT_IN(9,OFF) '��ת IN9 ��ƽ�źţ������źŽ��з�ת(ZMC ������)     ON�Ƿ�ת OFF�ǲ���ת������ָ��Ӧ�ÿ��Բ�Ҫ��                
'		'ͨ�����������Ϊԭ�㿪���źţ� 1 ��Ч��
'		'������ԭ�㿪�غ� ZMC ���������� OFF ʱ��Ϊ���ź����룬Ҫ�෴Ч��������
'		'INVERT_IN() '��ת��ƽ�� (ECI ϵ�п���������)��
'		TRIGGER   '�Զ�����ʾ����
'
'		DATUM(3)'ģʽ3����ԭ��'   DATUM������ԭ���˶�  ��speed���ٶ�
'		
'		WAITIDLE
'		?"���˶�"
		
		
		
		
		'MOVE_DELAY(1000)   '���һ�����ģ��ȴ���һ��ץȡ

	ENDIF

'����ǰ�ƣ�shift+tab

ENDSUB		
	
SUB conveyer_control()
	
	'b=IN(9)
	'PRINT "IN9:" IN(9)
	
	BASE(14)  '��۴��ʹ�
	REP_DIST = 100000000   '��������ѭ����Χ
	RS_LIMIT = -200000000 '��������λ���������-REP_DIST �Ż���Ч
	FS_LIMIT = 200000000 '��������λ������С�� REP_DIST �Ż���Ч
	DPOS(14)=0
	MPOS(14)=0   '����������λ��
	UNITS(14)=552731
	SPEED=200         '������3,ʵ������Լ117����/��
	ACCEL(13)=2000  '���ٶ�
	DECEL(13)=2000  '���ٶ�
	VMOVE(1) AXIS(14)
	
	
'	
'	
'	''''''''С�۴��ʹ�'''''''
	BASE(12)
	REP_DIST = 100000000   '��������ѭ����Χ
	RS_LIMIT = -200000000'��������λ���������-REP_DIST �Ż���Ч
	FS_LIMIT = 200000000 '��������λ������С�� REP_DIST �Ż���Ч
	DPOS(12)=0
	MPOS(12)=0   '����������λ��
	UNITS(12)=552731   '��ֱ��58mm
	ACCEL(13)=2000  '���ٶ�
	DECEL(13)=2000  '���ٶ�
	
	
	BASE(15)
	REP_DIST = 100000000   '��������ѭ����Χ
	RS_LIMIT = -200000000'��������λ���������-REP_DIST �Ż���Ч
	FS_LIMIT = 200000000 '��������λ������С�� REP_DIST �Ż���Ч
	DPOS(15)=0
	MPOS(15)=0   '����������λ��
	UNITS(15)=125424   '��ֱ��106.5mm �ܳ�334.41��
	ACCEL(15)=2000  '���ٶ�
	DECEL(15)=2000  '���ٶ�
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
'			'?"�ٶ�Ϊ700"
'			
'		ELSE 
'			SPEED = 150
'			VMOVE(1) AXIS(12)
'			'?"�ٶ�Ϊ100"
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
		PRINT "���Ҳ�������紫������" IN(13)
		IF IN(12)>0 AND IN(13) > 0  THEN  ' IN12 �½��ش���,����������ͬʱ����Ƚ��ѣ�д��һ��
			PRINT "���Ҳ�������紫������" IN(13)
		
			PRINT "���䵽��β��"
		
			PRINT "���侭��β��"
			end_effector_control()
		ENDIF
	WEND
ENDSUB




SUB identification()


		
	BASE(20,21,22,23,24,25) 				'ѡ���������
	'CONNREFRAME(121,10,0,1,2,3,4,5) 		'�����������ӡ�
	CONNREFRAME(123,50,26,27,28,29,30,31) 		'�����������ӡ�
	
	WAIT LOADED
	PRINT "frame״̬��" MTYPE(20)
	
	
	
	
	
	?"��λ������ֵ��"VR(0),VR(1),VR(2),VR(3),VR(4)
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
		
		IF MTYPE(20)=34 THEN   '�ж��Ƿ������״̬  ������ mtype(6) = 34�� ���״̬�ж� Mtype(0) = 33
			BASE(26,27,28,29,30,31)
			

			table(350,DPOS(26),DPOS(27),DPOS(28),DPOS(29),DPOS(30),0)
			'table(350,0,25,0,0,0,0)
			'BASE(20,21,22,23,24,25)
			frame_trans2(350,450,1)
			?"���ⷵ�أ�"frame_trans2(350,450,1)'-1�н�0�޽�
		 
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
		'''''IN(6) ���һ�е�۱���   IN(7)  �·���е�۱���   IN(14)  ��ͣ�ź�
		
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
'WAIT UNTIL DPOS(0)>1000 '�ȴ��� 0 ���е� 1000



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


'INVERT_IN(5,ON) '��ת IN5 ��ƽ�źţ������źŽ��з�ת(ZMC ������)
