RAPIDSTOP(2)  '����������


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
		AXIS_ADDRESS(3)=0+7
		AXIS_ADDRESS(4)=0+8
		
		AXIS_ADDRESS(12)=0+4
		AXIS_ADDRESS(13)=0+5
		AXIS_ADDRESS(14)=0+6
		
		
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
				'DRIVE_CONTROLWORD(5)=0
				'DRIVE_CONTROLWORD(6)=0  
				
                DELAY (10)

                DATUM(0)                                                      '������ԭ���˶���ZMC ������Ϊ 0 ������Ч������Ϊ OFF ״̬ʱ����ʾ����ԭ��/��λ

                DELAY (100)         

                ?"��ʹ��׼��"
				AXIS_ENABLE(0)=1  
				AXIS_ENABLE(1)=1
                AXIS_ENABLE(2)=1                   '��ʹ��
				AXIS_ENABLE(3)=1   
				AXIS_ENABLE(4)=1 
				AXIS_ENABLE(12)=1 
				AXIS_ENABLE(13)=1
				AXIS_ENABLE(14)=1
				'AXIS_ENABLE(5)=1  
				'AXIS_ENABLE(6)=1  
                WDOG=1                                                        'ʹ���ܿ���

                ?"��ʹ�����"

ELSE

                ?"���߿���ʧ��"

        ENDIF

ELSE

        ?"����ɨ��ʧ��"

ENDIF


DELAY(5000)


'BASE(14)
'UNITS(14)=552730
'SPEED=50             '������3
'VMOVE(-1) AXIS(14)     '-1 �����˶��� 1 �����˶�

'5417��0��1458
'5418��450��1160
'5418��60��1160
'5418��-330��1160
'
'5418��-330��980
'5418��60��980
'5418��450��980
'
'5418��450��800
'5418��60��800
'5418��-330��800



'''''�������е�ֲ�������




DIM baseL,L1,L2,L3,L4,L5,L6,L7,L8,L9,L10,L11,L12,crank(3),dok(4)
baseL = 355
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
L12 = 1189.44 

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
FS_LIMIT = 15,38,35.5,18,15,15        '�����������λ�ǽ�λ��ô�� 
RS_LIMIT = -15,-35,-61.5,-18,-15,-15
'DPOS=0,0,0,0,0,0
'MPOS=0,0,0,0,0,0



UNITS = Pules1OneCircle/360,Pules2OneCircle/360,Pules3OneCircle/360,Pules4OneCircle/360,Pules5OneCircle/360,Pules6OneCircle/360   '��units���ÿ��������
'DPOS=0,0,0,0,0,0
'MPOS= 0,0,0,0,0,0                             '���ùؽ����λ�ã��˴�Ҫ����ʵ��������޸ġ�



speed=5,5,5,5,5,5                        '�ٶȲ�������,5�Ѿ�����С�ٶ�
accel=50,50,50,50,50,50
decel=50,50,50,50,50,50
FASTDEC=1000,1000,1000,1000,1000,1000
CLUTCH_RATE=1000,1000,1000,1000,1000,1000                'CONNECT ���ӵ��ٶȣ����ڶ��������ʴ� 0 �����ñ��ʵĸı�ʱ�䣬 ratio/��

'CONNECT,ͬ���˶�ָ��,����ǰ���Ŀ��λ���� driving_axis ��Ĳ���λ��ͨ�����ӳ������ӡ�
BASE(3,4)
'DPOS=0,0
'MPOS=0,0




BASE(12)
'DPOS=0,0
'MPOS=0,0
UNITS(12) = 552370 
ACCEL(12) = 1000
DECEL(12) = 1000

RS_LIMIT = -20000000 '��������λ���������-REP_DIST �Ż���Ч
FS_LIMIT = 200000000

BASE(13)   'ѡ�����2

'���������23λ���ֱ���8388608��һ���������ֱ����������תһȦ�����嵱��=8388608/360=23301  ��Ҫ���Ǽ��ٱ���

UNITS(13)=534306   '���嵱��552731   ���嵱��һ����Ϊ����� 1mm �� 1������������������嵱��p= e*k/L  e�������ֱ���8388608��LΪͬ�����ܳ���kΪ���ٱ�
SPEED(13)=450  '�ٶ�����Ϊ1000
CREEP(13)=50   ' ��ԭ�㷴�������ٶȣ��ٶ�Ϊ700�������ǹ��㣿
ACCEL(13)=2000  '���ٶ�
DECEL(13)=2000  '���ٶ�
REP_DIST = 100000000   '��������ѭ����Χ
RS_LIMIT = 0 '��������λ���������-REP_DIST �Ż���Ч
FS_LIMIT = 580 '��������λ������С�� REP_DIST �Ż���Ч
'DPOS(13)=0
'MPOS(13)=0   '����������λ��
MERGE = ON   '���������岹
SRAMP=200    '�Ӽ��ٱ仯�ʣ�ʹ���ٶ�����ƽ�����ڻ�е��ͣ��Ӽ���ʱ���ٶ�������Χ��0-250����֮��
FASTDEC(13)=300   '������ٶȣ��ﵽ��λ���쳣ֹͣʱ�Զ����á�
IF IN(9) > 0 THEN
	?"����ԭ�㣡"
ELSE
	DATUM_IN=9 '���� IN9 ��Ϊԭ�㿪��     
	INVERT_IN(9,OFF) '��ת IN9 ��ƽ�źţ������źŽ��з�ת(ZMC ������)     ON�Ƿ�ת OFF�ǲ���ת������ָ��Ӧ�ÿ��Բ�Ҫ��                
	'ͨ�����������Ϊԭ�㿪���źţ� 1 ��Ч��
	'������ԭ�㿪�غ� ZMC ���������� OFF ʱ��Ϊ���ź����룬Ҫ�෴Ч��������
	'INVERT_IN ��ת��ƽ�� (ECI ϵ�п���������)��
	TRIGGER   '�Զ�����ʾ����

	DATUM(3)'ģʽ3����ԭ��'   DATUM������ԭ���˶�  ��speed���ٶ�
	?"���ڸ�λ��"
	WAITIDLE
	?"��λ�ɹ���ԭ���ѻ��㣡"
ENDIF
DPOS(13)=0 '��13�Ƹ˵�λ��������'
MPOS(13)=0


''''''��۴��ʹ�'''''''
BASE(14)
UNITS(14) = 552731
ACCEL(14) = 1000
DECEL(14) = 1000
REP_DIST = 100000000   '��������ѭ����Χ
RS_LIMIT = -200000000 '��������λ���������-REP_DIST �Ż���Ч
FS_LIMIT = 200000000 '��������λ������С�� REP_DIST �Ż���Ч
DPOS(14)= 0
MPOS(14) = 0   '����������λ��



BASE(0,1,2)
FWD_IN=0,1,2  '�ֱ�����������λ����,���� IN0 ��Ϊ�� 0 ��������λ�ź�
INVERT_IN(0,ON) '��ת�ź�
INVERT_IN(1,ON)  'ZMC ϵ�п��������� ON ʱ��Ϊ���ź�����
INVERT_IN(2,ON)

BASE(0,1,2)
REV_IN=3,4,5  '�ֱ�����������λ����,���� IN3 ��Ϊ�� 0 �ĸ�����λ�ź�
INVERT_IN(3,ON) '��ת�ź�
INVERT_IN(4,ON)  'ZMC ϵ�п��������� ON ʱ��Ϊ���ź�����
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
speed=200,100,100,100,100,100                        '�ٶȲ�������  100uits/s
accel=350,1000,1000,1000,1000,1000
decel=350,1000,1000,1000,1000,1000
'decel=1000,1000,1000,1000,1000,1000
'ATYPE=0,0,0,0,0,0                                            '����Ϊ������,ǰ�����߳�ʼ������������
'DPOS=0,0,0,0,0,0     
TABLE(10,baseL,L1,L2,L3,L4,L5,L6,L7,L8,L9,L10,L11,L12,crank(0),crank(1),crank(2),dok(0),dok(1),dok(2),dok(3),Pules1OneCircle, Pules2OneCircle, Pules3OneCircle, Pules4OneCircle, Pules5OneCircle , PulesROneCircle,0,0,0,0) 'У����ĩ�˲���
'�Ƚϵ���������   �����洢�� TABLE10 ��ʼ��λ��  ���TABLE��ʲô���ǲ�̫��⣬�ֲ���Ҳ��˵�����ݴ洢��λ��



UNITS=1000,1000,1000,1000,1000,1000         '�˶����ȣ�Ҫ��ǰ���ã���;���ܱ仯   UNITS���嵱��



BASE(0,1,2,3,4,5)  '�����ؽ���
CONNFRAME(121,10,6,7,8,9,10,11)'���ģʽ
WAIT LOADED

'BASE(6,7,8,9,10,11)
'MOVEABS(5417,0,1458,0,0,0)

'''  ������ '''''
'���̸߶�170mm

PRINT "frame״̬��" MTYPE(0)
IF MTYPE(0)=33 THEN   '�ж��Ƿ������״̬  ������ mtype(6) = 34�� ���״̬�ж� Mtype(0) = 33
	BASE(6,7,8,9,10,11)
	DIM Z1,Z2,Z3,Z4,Z5,Z6
	Z1=5417.428
	Z2=0
	Z3=1458
	Z4=0  'rx  ����
	Z5=0  'ry  ����
	Z6=0  'rz  ƫ��
	table(100,Z1,Z2,Z3,Z4,Z5,Z6)
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
		
		  
		'MOVEABS(TABLE(200),TABLE(201),TABLE(202),TABLE(203),TABLE(204))  'moveabs�������˶�ָ�MOVE_PT����λʱ�����
		
		'FORCE_SPEED = 1,1,1,1,1,1
		'MOVEABSSP(TABLE(200),TABLE(201),TABLE(202),TABLE(203),TABLE(204),0)
		
'		BASE(6,7,8,9,10,11)
'		MOVEABS(Z1,Z2,Z3,Z4,Z5,Z6)
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

END