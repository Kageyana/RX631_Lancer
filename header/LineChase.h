#ifndef LINECHASE_H_
#define LINECHASE_H_
//====================================//
// インクルード									//
//====================================//
#include "PeripheralFunctions.h"
#include "LineChase.h"
#include "I2C_MPU-9255.h"
#include <math.h>
//====================================//
// シンボル定義									//
//====================================//
// 緊急停止
#define	STOPPING_METER		20		// 停止距離

// 各セクションでの目標速度　x/10[m/s]
#define SPEED_STRAIGHT			54	// 通常トレース
#define SPEED_CURVE_BRAKE		20	// カーブブレーキ
#define SPEED_CURVE_R600		38	// R600カーブ速度
#define SPEED_CURVE_R450		34	// R450カーブ速度
#define SPEED_CURVE_STRAIGHT	44	// S字カーブ直線速度

#define ENC_BEFORECURVE 		600	// カーブ前の減速区間
#define ENC_AFTERCURVE 		600	// カーブ後の安定区間

// カーブ関連
#define CURVE_R600_START	20		// R600開始AD値
#define CURVE_R450_START	140		// R450開始AD値

// ジャイロ関連
#define AD_3V3VOLTAGE		0.806	// 3V時の1AD値あたりの電圧[mV]
#define AD_5VOLTAGE		1.22		// 5V時の1AD値あたりの電圧[mV]
#define GYROVOLTAGE		0.67		// 電圧毎角加速度[mV/deg/s]
#define SLOPEUPPERLINE_IMU		4		// 上り坂検出角度
#define SLOPELOWERLINE_IMU		-4		// 下り坂検出角度
#define SLOPEUPPERLINE_AD		17		// 上り坂検出角度
#define SLOPELOWERLINE_AD		-14		// 下り坂検出角度
#define INTEGRAL_LIMIT		200		// 角速度積算時間

#define PI					3.141592	// 円周率
#define RIGHTCURVE_ENCODER	78.5		// 右輪中心からエンコーダーの中心までの距離
#define LEFTCURVE_ENCODER	74.5		// 左輪中心からエンコーダーの中心までの距離

#define TARGETDISTANCE		14850	// 的までのパルス
#define TARGETDISTANCE_E	6187.5	// E的までの距離
#define TARGETDISTANCE_F	7425		// F的までの距離
#define TARGETDISTANCE_ABCD	4950	// A,B,C,D的までの距離
#define DEGTOAD			12.28	// 1°あたりのAD値

// PIDゲイン関連
//白線トレース
#define KP			20
#define KI			3
#define KD		58

// 角度制御
#define KP2		9
#define KI2		90
#define KD2		90

// 速度制御
#define KP3		6
#define KI3		20
#define KD3		0

// 槍角度制御
#define KP4		4
#define KI4		4
#define KD4		90

// 緊急停止関連
#define STOP_SENSOR1		60		// センサ全灯
#define STOP_SENSOR2		100		// センサ全消灯
#define STOP_ENCODER		10		// エンコーダ停止(ひっくり返った？)
#define STOP_GYRO			100		// マイナスの加速度検知(コースから落ちた？)
#define STOP_COUNT		10000	// 時間停止
//====================================//
// グローバル変数の宣言							//
//====================================//
// パターン、モード関連
extern char 	pattern;			// パターン番号
extern char	lcd_mode;			// LCD表示選択
extern char	slope_mode;		// 坂チェック		0:上り坂始め	1:上り坂終わり	2:下り坂始め	3:下り坂終わり
extern char	angle_mode;		// サーボPWM変更	0:白線トレース	1:角度制御
extern char	pushcart_mode;		// 手押しモード可否	0:自動走行	1:手押し
extern char	msdset;			// MicroSDが初期化されたか	0:初期化失敗	1:初期化成功
extern char	IMUSet;			// IMUが初期化されたか	0: 初期化失敗	1:初期化成功

extern char	targetmarker;		// 的の番号


// パラメータ関連
// 距離
extern short	stopping_meter;			// 停止距離
// 速度
extern short	speed_straight;			// 通常トレース
extern short	speed_curve_brake;		// カーブブレーキ
extern short	speed_curve_r600;		// R600カーブ速度
extern short	speed_curve_r450;		// R450カーブ速度
extern short	speed_curve_straight;	// S字カーブ直線速度

extern short 	enc_buforecurve;		// カーブ前の減速区間
extern short	enc_aftercurve;			// カーブ後の安定区間

// サーボ角度
extern short	angle_rightclank;		// 右クランク旋回角度
extern short	angle_leftclank;			// 左クランク旋回角度
extern short	angle_rightchange;		// 右レーンチェンジ旋回角度
extern short	angle_leftchange;		// 右レーンチェンジ旋回角度

// タイマ関連
extern short		cnt_gyro;			// 角度計算用カウンタ

// 角度関連
extern double 		TurningAngleEnc;	// エンコーダから求めた旋回角度
extern double		PichAngleAD;		// アナログジャイロから求めたピッチ角度

// モーター関連
extern signed char 	motorPwm;	// モーター制御PWM
extern short		targetSpeed;	// 目標速度

// ゲイン関連
extern char	kp_buff, ki_buff, kd_buff;
extern char	kp2_buff, ki2_buff, kd2_buff;
extern char 	kp3_buff, ki3_buff, kd3_buff;
extern char 	kp4_buff, ki4_buff, kd4_buff;

// デモ関連
extern char demo;

// サーボ関連
extern double		Int;			// I成分積算値(白線トレース)
extern short 		SetAngle;		// 目標角度
extern signed char 	ServoPwm;	// 白線トレースサーボPWM
extern signed char 	ServoPwm2;	// 角度サーボPWM

extern short 		SetAngle2;		// 目標角度
extern signed char 	ServoPwm3;	// 角度サーボPWM
//====================================//
// プロトタイプ宣言								//
//====================================//
// マーカー関連
signed char check_crossline( void );
signed char check_rightline( void );
signed char check_leftline( void );
signed char check_slope( void );

// 角度関連
void getPichAngleAD( void );
void getTurningAngleEnc(void);

// エンコーダ関連
unsigned int enc_mm( short mm );

// モーター関連
void motorControl( void );

// 内輪差関連
void diff ( signed char pwm );

// サーボ関連
void servoControl( void );
void servoControl2( void );

// 的関連
void targettheta (void);

#endif // LINECHASE_H_