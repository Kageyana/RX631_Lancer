#ifndef LINECHASE_H_
#define LINECHASE_H_
//====================================//
// �C���N���[�h									//
//====================================//
#include "PeripheralFunctions.h"
#include "LineChase.h"
#include "I2C_MPU-9255.h"
#include <math.h>
//====================================//
// �V���{����`									//
//====================================//
// �ً}��~
#define	STOPPING_METER		20		// ��~����

// �e�Z�N�V�����ł̖ڕW���x�@x/10[m/s]
#define SPEED_STRAIGHT			54	// �ʏ�g���[�X
#define SPEED_CURVE_BRAKE		20	// �J�[�u�u���[�L
#define SPEED_CURVE_R600		38	// R600�J�[�u���x
#define SPEED_CURVE_R450		34	// R450�J�[�u���x
#define SPEED_CURVE_STRAIGHT	44	// S���J�[�u�������x

#define ENC_BEFORECURVE 		600	// �J�[�u�O�̌������
#define ENC_AFTERCURVE 		600	// �J�[�u��̈�����

// �J�[�u�֘A
#define CURVE_R600_START	20		// R600�J�nAD�l
#define CURVE_R450_START	140		// R450�J�nAD�l

// �W���C���֘A
#define AD_3V3VOLTAGE		0.806	// 3V����1AD�l������̓d��[mV]
#define AD_5VOLTAGE		1.22		// 5V����1AD�l������̓d��[mV]
#define GYROVOLTAGE		0.67		// �d�����p�����x[mV/deg/s]
#define SLOPEUPPERLINE_IMU		4		// ���⌟�o�p�x
#define SLOPELOWERLINE_IMU		-4		// ����⌟�o�p�x
#define SLOPEUPPERLINE_AD		17		// ���⌟�o�p�x
#define SLOPELOWERLINE_AD		-14		// ����⌟�o�p�x
#define INTEGRAL_LIMIT		200		// �p���x�ώZ����

#define PI					3.141592	// �~����
#define RIGHTCURVE_ENCODER	78.5		// �E�֒��S����G���R�[�_�[�̒��S�܂ł̋���
#define LEFTCURVE_ENCODER	74.5		// ���֒��S����G���R�[�_�[�̒��S�܂ł̋���

#define TARGETDISTANCE		14850	// �I�܂ł̃p���X
#define TARGETDISTANCE_E	6187.5	// E�I�܂ł̋���
#define TARGETDISTANCE_F	7425		// F�I�܂ł̋���
#define TARGETDISTANCE_ABCD	4950	// A,B,C,D�I�܂ł̋���
#define DEGTOAD			12.28	// 1���������AD�l

// PID�Q�C���֘A
//�����g���[�X
#define KP			20
#define KI			3
#define KD		58

// �p�x����
#define KP2		9
#define KI2		90
#define KD2		90

// ���x����
#define KP3		6
#define KI3		20
#define KD3		0

// ���p�x����
#define KP4		4
#define KI4		4
#define KD4		90

// �ً}��~�֘A
#define STOP_SENSOR1		60		// �Z���T�S��
#define STOP_SENSOR2		100		// �Z���T�S����
#define STOP_ENCODER		10		// �G���R�[�_��~(�Ђ�����Ԃ����H)
#define STOP_GYRO			100		// �}�C�i�X�̉����x���m(�R�[�X���痎�����H)
#define STOP_COUNT		10000	// ���Ԓ�~
//====================================//
// �O���[�o���ϐ��̐錾							//
//====================================//
// �p�^�[���A���[�h�֘A
extern char 	pattern;			// �p�^�[���ԍ�
extern char	lcd_mode;			// LCD�\���I��
extern char	slope_mode;		// ��`�F�b�N		0:����n��	1:����I���	2:�����n��	3:�����I���
extern char	angle_mode;		// �T�[�{PWM�ύX	0:�����g���[�X	1:�p�x����
extern char	pushcart_mode;		// �艟�����[�h��	0:�������s	1:�艟��
extern char	msdset;			// MicroSD�����������ꂽ��	0:���������s	1:����������
extern char	IMUSet;			// IMU�����������ꂽ��	0: ���������s	1:����������

extern char	targetmarker;		// �I�̔ԍ�


// �p�����[�^�֘A
// ����
extern short	stopping_meter;			// ��~����
// ���x
extern short	speed_straight;			// �ʏ�g���[�X
extern short	speed_curve_brake;		// �J�[�u�u���[�L
extern short	speed_curve_r600;		// R600�J�[�u���x
extern short	speed_curve_r450;		// R450�J�[�u���x
extern short	speed_curve_straight;	// S���J�[�u�������x

extern short 	enc_buforecurve;		// �J�[�u�O�̌������
extern short	enc_aftercurve;			// �J�[�u��̈�����

// �T�[�{�p�x
extern short	angle_rightclank;		// �E�N�����N����p�x
extern short	angle_leftclank;			// ���N�����N����p�x
extern short	angle_rightchange;		// �E���[���`�F���W����p�x
extern short	angle_leftchange;		// �E���[���`�F���W����p�x

// �^�C�}�֘A
extern short		cnt_gyro;			// �p�x�v�Z�p�J�E���^

// �p�x�֘A
extern double 		TurningAngleEnc;	// �G���R�[�_���狁�߂�����p�x
extern double		PichAngleAD;		// �A�i���O�W���C�����狁�߂��s�b�`�p�x

// ���[�^�[�֘A
extern signed char 	motorPwm;	// ���[�^�[����PWM
extern short		targetSpeed;	// �ڕW���x

// �Q�C���֘A
extern char	kp_buff, ki_buff, kd_buff;
extern char	kp2_buff, ki2_buff, kd2_buff;
extern char 	kp3_buff, ki3_buff, kd3_buff;
extern char 	kp4_buff, ki4_buff, kd4_buff;

// �f���֘A
extern char demo;

// �T�[�{�֘A
extern double		Int;			// I�����ώZ�l(�����g���[�X)
extern short 		SetAngle;		// �ڕW�p�x
extern signed char 	ServoPwm;	// �����g���[�X�T�[�{PWM
extern signed char 	ServoPwm2;	// �p�x�T�[�{PWM

extern short 		SetAngle2;		// �ڕW�p�x
extern signed char 	ServoPwm3;	// �p�x�T�[�{PWM
//====================================//
// �v���g�^�C�v�錾								//
//====================================//
// �}�[�J�[�֘A
signed char check_crossline( void );
signed char check_rightline( void );
signed char check_leftline( void );
signed char check_slope( void );

// �p�x�֘A
void getPichAngleAD( void );
void getTurningAngleEnc(void);

// �G���R�[�_�֘A
unsigned int enc_mm( short mm );

// ���[�^�[�֘A
void motorControl( void );

// ���֍��֘A
void diff ( signed char pwm );

// �T�[�{�֘A
void servoControl( void );
void servoControl2( void );

// �I�֘A
void targettheta (void);

#endif // LINECHASE_H_