///////////////////////////////////////////////////////////////////////////
//												//
//  FILE	:Klic_trace.c								//
//  DATE	:Thr, Dec 21, 2017							//
//  DESCRIPTION :走行プログラム							//
//  CPU TYPE	:RX631								//
//												//
///////////////////////////////////////////////////////////////////////////
//====================================//
// インクルード									//
//====================================//
#include "PeripheralFunctions.h"
#include "LineChase.h"
#include "SetUp.h"
#include "SCI.h"
#include "E2dataFlash.h"
#include "I2C_LCD.h"
#include "MicroSD.h"
#include "I2C_MPU-9255.h"
#include "MemorryTrace.h"
#include <stdio.h>

//====================================//
// グローバル変数の宣言							//
//====================================//
// 走行パターン関連
char		pattern = 0;	// パターン番号
char		countdown = 0x10;
short 	angle_center;

// モード関連
char		curve_moed;	// カーブ判定	0:カーブ以外	1:カーブ走行中
char		error_mode;	// 0:距離停止 1:センサ全灯 2:センサ全消灯 3:エンコーダ停止 4:ジャイロ反応

// タイマ関連
// 1msタイマ
unsigned int 		cnt1;		// 走行用タイマカウント
unsigned short	 	cnt_out;	// コースアウト判定用タイマ
unsigned short	 	cnt_out2;	// コースアウト判定用タイマ2
unsigned short	 	cnt_out3;	// コースアウト判定用タイマ3
unsigned short	 	cnt_out4;	// コースアウト判定用タイマ4
static char			Timer10;	// 1msカウント用

char	BeforeCurve = 1;
//====================================//
// プロトタイプ宣言								//
//====================================//
void init_Parameter ( bool lcd );
//====================================//
// メインプログラム								//
//====================================//
void main(void){
	short i;
	unsigned int ui;
	
	//=================================//
	// 初期化									//
	//=================================//
	inti_lcd();			// LCD初期化
	
	motor_f( 0, 0 );
	motor_r( 0, 0 );
	servoPwmOut( 0 );
	//servoPwmOut2( 0 );
	
	pushcart_mode = 0;		// 手押しモードoff
	slope_mode = 0;		// 上り坂チェック
	angle_mode = 0;		// 白線トレース
	start = 0;				// ゲートスタート
	
	init_BeepS();			// ブザー初期化
	SET_SCI_C1
	
	// SCI1初期化
	if ( !init_IMU() ) {
		setBeepPatternS( 0x8000 );
		IMUSet = 1;
	} else {
		setBeepPatternS( 0xcc00 );
		init_SCI1( UART, RATE_230400 );
		IMUSet = 0;
	}
	wait_lcd(100);
	// フラッシュ初期化
	if( !initFlash() ) {
		setBeepPatternS( 0x8000 );
		readFlashSetup( 1, 1, 1 ,1 ,1 ,1 ,1 ,1);	// データフラッシュから前回パラメータを読み込む
	} else{
		setBeepPatternS( 0xcc00 );
	}
	wait_lcd(100);
	// MicroSDカード初期化
	if( !init_msd() ) {
		setBeepPatternS( 0x8000 );
		msdset = 1;
	} else {
		setBeepPatternS( 0xcc00 );
		msdset = 0;
	}
	wait_lcd(100);
	while(1){
		__setpsw_i();
		if( pattern >= 11 && pattern <= 99 ) {
			if( !pushcart_mode ) {		
				// 手押しモードOFF
				if( cnt1 >= 100 ) {		// 動き出してから
					if ( EncoderTotal >= ( PALSE_METER * stopping_meter ) ) { // 距離超過の場合
						error_mode = 0;
						pattern = 101;
					} else if ( cnt_out >= STOP_SENSOR1 ) {	// センサ全灯
						error_mode = 1;
						pattern = 101;
					} else if ( cnt_out2 >= STOP_SENSOR2 ) {	// センサ全消灯
						error_mode = 2;
						pattern = 101;
					} else if ( cnt_out3 >= STOP_ENCODER ) {	// エンコーダ停止(ひっくり返った？)
						error_mode = 3;
						pattern = 101;
					} else if( cnt_out4 >= STOP_GYRO ) {	// マイナスの加速度検知(コースから落ちた？)
						error_mode = 4;	
						pattern = 101;
					}
					/*
					// Buletoothで外部から停止
					if ( stopWord == 1 ) {
						error_mode = 5;
						pattern = 101;
					}
					*/
					/*
					// 一定時間で停止
					if( cnt1 >= STOP_COUNT ) {
						pattern = 101;
					}
					*/
				}
			} else {			
				// 手押しモードON
				lcdPosition( 0, 0 );
				lcdPrintf("mark %x  ", targetmarker);
				lcdPosition( 0, 1 );
				lcdPrintf("    %4d",SetAngle2);
			}
			// スイッチで停止
			if ( tasw_get() == 0x4 ) {
				error_mode = 6;
				pattern = 101;
			}
		} else if ( pattern >= 100 ) {
			lcd_mode = 1;
			lcdPosition( 0, 0 );
			lcdPrintf("TIME  %d", error_mode);
			lcdPosition( 0, 1 );
			lcdPrintf(" %5dms", ui);
		}
		
	switch( pattern ) {
		//-------------------------------------------------------------------
		// 【000】スタート処理
		//-------------------------------------------------------------------
		case 0:
			// スタート前設定
			setup();
			if ( start && !pushcart_mode ) {
				demo = 0;		// デモモード解除
				angle_mode = 0;	// 白線トレース
				Int = 0;			// 積分リセット
				txt= txt_data;		// 受信配列リセット
				cnt_byte = 0;		// 受信カウントリセット
				
				if ( msdset ) init_log();	// ログ記録準備
				
				if ( !fixSpeed ) writeFlashBeforeStart(1, 0, 1, 1, 1, 1, 1);	// 速度パラメータをデータフラッシュに保存
				else writeFlashBeforeStart(0, 0, 1, 1, 1, 1, 1);		// 速度パラメータ以外を保存
				
				wait_lcd(500);		// 500ms待つ
				cnt1 = 0;
				pattern = 1;
				break;
			} else if ( start && pushcart_mode ) {
				// 手押しモードの場合すぐに通常トレース
				if ( msdset ) init_log();	// ログ記録準備
				
				// 白線トレース用PIDゲイン保存
				// 角度制御用PIDゲイン保存
				writeFlashBeforeStart(0, 0, 1, 1, 0, 1, 0);
				setBeepPatternS( 0xfff0 );
				// 変数初期化
				init_Parameter( 1 );
				break;
			}
			break;
			
		case 1:
			SetAngle2 = 0;
			//servoPwmOut2( ServoPwm3 );
			servoPwmOut( ServoPwm );
			if ( start == 1 ) {
				// カウントダウンスタート
				if ( cnt1 >= 3000 ) {
					setBeepPatternS( 0xfff0 );
					
					// 変数初期化
					init_Parameter( 0 );
					break;
				} else if ( !(cnt1 % 1000) ) {
					setBeepPatternS( 0x8000 );
					led_out( countdown );
					countdown = countdown >> 1;
					break;
				}
			} else if ( start == 2 ) {
				// スタートゲート開放スタート
				pattern = 2;
				break;
			}
			break;
			
		case 2:
			servoPwmOut( ServoPwm );
			// スタートバー開閉待ち
			if ( !startbar_get() ) {
				// 変数初期化
				init_Parameter( 0 );
				break;
			}
			// LED点滅処理
			if ( cnt1 >= 2000 ) cnt1 = 0;
			if ( cnt1 < 1000 ) {
				led_out( 0x04 );
			} else {
				led_out( 0x08 );
			}
			break;
		//-------------------------------------------------------------------
		// 【010】トレース処理
		//-------------------------------------------------------------------
		case 11:
			SetAngle2 = 0;
			//servoPwmOut2( ServoPwm3 );
			servoPwmOut( ServoPwm );
			targetSpeed = speed_straight * SPEED_CURRENT;
			diff( motorPwm );
			i = getServoAngle();
			led_out( 0x00 );
			
			// 的マーカーチェック
			if ( check_rightline() ) {
				enc1 = 0;
				setBeepPatternS( 0x8000 );
				pattern = 21;
				break;
			}
			if ( check_leftline() ) {
				enc1 = 0;
				setBeepPatternS( 0x8000 );
				pattern = 22;
				break;
			}
			if ( check_crossline() ) {
				enc1 = 0;
				setBeepPatternS( 0x8000 );
				pattern = 12;
				break;
			}
			/*
			// カーブチェック
			if ( i >=  CURVE_R600_START || i <= -CURVE_R600_START ) {
				enc1 = 0;
				curve_moed = 1;
				pattern = 12;
				break;
			}*/
			break;
			
		case 12:
			// カーブブレーキ
			SetAngle2 = 0;
			//servoPwmOut2( ServoPwm3 );
			servoPwmOut( ServoPwm );
			targetSpeed = speed_curve_brake * SPEED_CURRENT;
			led_out( 0x1e );
			diff( motorPwm );
			
			if( enc1 > enc_mm(enc_buforecurve) ) {		// 600mm進む
				enc1 = 0;
				pattern = 13;
				break;
			}
			break;
			
		case 13:
			// R600カーブ走行
			SetAngle2 = 0;
			//servoPwmOut2( ServoPwm3 );
			servoPwmOut( ServoPwm );
			targetSpeed = speed_curve_r600 * SPEED_CURRENT;
			diff( motorPwm );
			i = getServoAngle();
			
			//直線チェック
			if( i <  CURVE_R600_START && i > -CURVE_R600_START ) {
				enc1 = 0;
				curve_moed = 0;
				pattern = 14;
				break;
			}
			break;
			
		case 14:
			SetAngle2 = 0;
			//servoPwmOut2( ServoPwm3 );
			servoPwmOut( ServoPwm );
			targetSpeed = speed_curve_straight * SPEED_CURRENT;
			diff( motorPwm );
			if( enc1 > enc_mm(enc_aftercurve) ) {		// 600mm進む
				enc1 = 0;
				pattern = 11;
				break;
			}
			break;
		//-------------------------------------------------------------------
		// 【020】マーカー誤検知処理
		//-------------------------------------------------------------------
		case 21:
			SetAngle2 = 0;
			//servoPwmOut2( ServoPwm3 );
			servoPwmOut( ServoPwm );
			targetSpeed = speed_straight * SPEED_CURRENT;
			diff( motorPwm );
			
			// カーブマーカーチェック
			if ( check_crossline() ) {
				enc1 = 0;
				setBeepPatternS( 0x8000 );
				pattern = 12;
				break;
			}
			if ( enc1 >= enc_mm( 60 ) ) {
				if ( targetmarker == 0xe ) {
					targetmarker = 0xf;
				} else if ( targetmarker == 0xf ) {
					targetmarker = 0xa;
				} else if ( targetmarker == 0xb ) {
					targetmarker = 0xc;
				} else if ( targetmarker == 0xd ) {
					targetmarker = 0xe;
				} else if ( targetmarker == 0 ) {
					targetmarker = 0xe;
				}
				
				enc1 = 0;
				pattern = 31;
				break;
			}
			break;
			
		case 22:
			SetAngle2 = 0;
			//servoPwmOut2( ServoPwm3 );
			servoPwmOut( ServoPwm );
			targetSpeed = speed_straight * SPEED_CURRENT;
			diff( motorPwm );
			
			// カーブマーカーチェック
			if ( check_crossline() ) {
				enc1 = 0;
				setBeepPatternS( 0x8000 );
				pattern = 12;
				break;
			}
			if ( enc1 >= enc_mm( 60 ) ) {
				if ( targetmarker == 0xa ) {
					targetmarker = 0xb;
				} else if ( targetmarker == 0xc ) {
					targetmarker = 0xd;
				}
				
				enc1 = 0;
				pattern = 41;
				break;
			}
			break;
			
		//-------------------------------------------------------------------
		// 【030】右的
		//-------------------------------------------------------------------
		case 31:
			//targettheta();
			if ( targetmarker == 0xe ) {
				SetAngle2 = 517;
			} else if ( targetmarker == 0xf ) {
				SetAngle2 = 596;
			} else {
				SetAngle2 = 398;
			}
			servoPwmOut( ServoPwm );
			//servoPwmOut2( ServoPwm3 );
			targetSpeed = speed_straight * SPEED_CURRENT;
			diff( motorPwm );
			// 500mm進む
			if ( enc1 > enc_mm( 600 ) ) {
				enc1 = 0;
				pattern = 32;
				break;
			}
			break;
			
		case 32:
				SetAngle2 = 0;
				//servoPwmOut2( ServoPwm3 );
				servoPwmOut( ServoPwm );
				targetSpeed = speed_straight * SPEED_CURRENT;
				diff( motorPwm );
				if ( enc1 > enc_mm( 200 ) ) {
					enc1 = 0;
					pattern = 11;
					break;
				}
				break;
		//-------------------------------------------------------------------
		// 【040】左的
		//-------------------------------------------------------------------
		case 41:
			SetAngle2 = -398;
			servoPwmOut( ServoPwm );
			//servoPwmOut2( ServoPwm3 );
			targetSpeed = speed_straight * SPEED_CURRENT;
			diff( motorPwm );
			// 500mm進む
			if ( enc1 > enc_mm( 600 ) ) {
				enc1 = 0;
				pattern = 32;
				break;
			}
			break;
			
		case 42:
				SetAngle2 = 0;
				//servoPwmOut2( ServoPwm3 );
				servoPwmOut( ServoPwm );
				targetSpeed = speed_straight * SPEED_CURRENT;
				diff( motorPwm );
				if ( enc1 > enc_mm( 200 ) ) {
					enc1 = 0;
					pattern = 11;
					break;
				}
				break;
		//-------------------------------------------------------------------
		//【100】停止処理
		//-------------------------------------------------------------------
		case 101:
			enc1 = 0;	
			ui = cnt1;	// 走行時間取得
			pattern = 102;
			break;
			
		case 102:
			servoPwmOut( ServoPwm );
			//servoPwmOut2( 0 );
			targetSpeed = 0;
			motor_f( motorPwm, motorPwm );
			motor_r( motorPwm, motorPwm );
			
			if( Encoder <= 1 && Encoder >= -1 ) {
				enc1 = 0;
				pattern = 103;
				break;
			}
			break;
			
		case 103:
			servoPwmOut( 0 );
			motor_f( 0, 0 );
			motor_r( 0, 0 );
			
			if( msdset == 1 ) {
				pattern = 104;
				cnt1 = 0;
				break;
			}else{
				setBeepPatternS( 0xaa00 );
				pattern = 106;
				break;
			}
			break;
			
		case 104:
			// 最後のデータが書き込まれるまで待つ
			if ( cnt1 <= 1000 ) {	// 500ms待つ
				if( checkMicroSDProcess() == 11 ) {
					msdFlag = 0;			// ログ記録終了
					microSDProcessEnd();        // microSDProcess終了処理
					pattern = 105;
					break;
				}
			} else {			// 500ms以上経過したら終了
				setBeepPatternS( 0xf0f0 );
				pattern = 107;
				break;
			}
			break;
			
		case 105:
			// 終了処理が終わるまで待つ
			if( checkMicroSDProcess() == 0 ) {
				// MicroSD最終書き込みアドレス保存
				flashDataBuff[ 0 ] = msdWorkaddress >> 16;
				flashDataBuff[ 1 ] = msdWorkaddress & 0xffff;	// 終了アドレス
				writeFlashData( MSD_STARTAREA, MSD_ENDAREA, MSD_DATA, 2 );
				pattern = 106;
				setBeepPatternS( 0xa8a8 );
				break;
			}
			break;
			
		case 106:
			// LED点滅処理
			if( cnt1 >= 200 ) cnt1 = 0;
			if( cnt1 < 100 ) {
				led_out( 0x1f );
			}else{
				led_out( 0x00 );
			}
			break;
			
		case 107:
			// LED点滅処理
			if( cnt1 >= 200 ) cnt1 = 0;
			if( cnt1 < 100 ) {
				led_out( 0x15 );
			}else{
				led_out( 0x00 );
			}
			break;
			
		default:
			pattern = 101;
			break;
			
	} // end of "switch ( pattern )"
	} // end of "while ( 1 )"
}
///////////////////////////////////////////////////////////////////////////
// モジュール名 Timer									//
// 処理概要     1msごとにタイマ割り込み						//
// 引数         なし									//
// 戻り値       なし									//
///////////////////////////////////////////////////////////////////////////
void Timer (void) {
	short s;
	
	__setpsw_i();
	//　タイマカウント
	if ( pattern >= 11 ) {
		if ( pattern <= 99 ) {
			if ( pattern != 21 ) {				// クロスライン通過時は無視
				if ( sensor_inp() == 0x7 || sensor_inp() == 0x5 ) {	// センサ全灯
					cnt_out++;	
				} else {
					cnt_out = 0;
				}
			}
			if (getAnalogSensor() >= -100 && getAnalogSensor() <= 100) cnt_out2++;	// センサ全消灯
			else cnt_out2 = 0;
			if ( Encoder <= 1 && Encoder >= -1 ) cnt_out3++;		// エンコーダ停止(ひっくり返った？)
			else cnt_out3 = 0;
			s = (short)RollAngleIMU;
			if ( s >= 5 || s <= -5 ) cnt_out4++;
			else	cnt_out4 = 0;
		}
		// ウォッチドッグタイマの｣カウントリフレッシュ
		R_PG_Timer_RefreshCounter_IWDT();
	} else if ( pattern < 11 ) {
		cnt_setup++;
		cnt_setup2++;
		cnt_setup3++;
		cnt_swR++;
		cnt_swL++;
		cnt_flash++;
	}
	cnt0++;
	cnt1++;
	cnt_gyro++;
			
	// LCD表示
	if ( lcd_mode ) lcdShowProcess();

	// エンコーダカウント
	getEncoder();

	// PID制御値算出
	if ( angle_mode ) servoControl2();		// 角度
	else 			servoControl();	// 白線
	motorControl();		// モータ
	servoControl3();
	
	// MicroSD書き込み
	microSDProcess();
	if ( msdFlag ) sendLog();
	
	Timer10++;
	
	// 通信
	if ( IMUSet ) {
		// I2C通信で加速度及び角速度を取得
		if ( Timer10 % 5 == 0 ) {
			IMUProcess();
			getTurningAngleIMU();
			getPichAngleIMU();
			getRollAngleIMU();
			if (cnt_gyro > 200) {
				RollAngleIMU = 0;
				PichAngleIMU = 0;
				cnt_gyro  = 0;
			}
		}
	} else {
		// UART受信
		commandSCI1();
		getTurningAngleEnc();
		if (cnt_gyro > 200) {
			PichAngleAD = 0;
			cnt_gyro  = 0;
		}
	}

	// 10ｍごとに実行
	switch ( Timer10 ) {	
	case 1:
		// ブザー
		beepProcessS();
		break;
	case 2:
		// スイッチ読み込み
		getSwitch();
		break;
	case 3:
		break;
	case 5:
		break;
	case 6:
		break;
	case 7:
		break;
	case 8:
		break;
	case 9:
		break;
	case 10:
		Timer10 = 0;
		break;
	default:
		break;
	}

}

///////////////////////////////////////////////////////////////////////////
// モジュール名 init_Parameter							//
// 処理概要     変数の初期化								//
// 引数         lcd: 1 lcd表示  0 lcd非表示						//
// 戻り値       なし									//
///////////////////////////////////////////////////////////////////////////
void init_Parameter ( bool lcd ) {
	cntmpattern2 = 0;	// 記録走行カウントリセット
	EncoderTotal = 10;	// 総走行距離
	cnt1 = 0;			// タイマリセット
	enc1 = 0;			// 区間距離リセット
	lcd_mode = lcd;		// LCD表示OFF
	msdFlag = 1;		// データ記録開始
	targetSpeed = speed_straight * SPEED_CURRENT; // 目標速度設定
	
	// 角度積算値リセット
	TurningAngleIMU = 0;
	RollAngleIMU = 0;
	PichAngleIMU = 0;
	R_PG_Timer_Start_IWDT();	// 独立ウォッチドッグタイマのカウントスタート
	pattern = 11;		// 通常走行
}