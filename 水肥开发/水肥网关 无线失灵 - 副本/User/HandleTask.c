#include "HandleTask.h"
#include "systemclock.h"
#include "string.h"
#include "DataFlash.h"
#include "Stm32_Configuration.h"
#include <stdlib.h>
#include "stm32f10x_it.h"
#include "stm32f10x.h"
#include "SI7021.h"
#include  <math.h>
#include "MQTTPacket.h"
#include "StackTrace.h"

#include "SI7021.h"
#include "max44009.h"
#include "password_MD5.h"


/* 2017年4月全网关硬件说明：
USART2：PD3~PD7(Remap);CDMA或GSM
无线指示灯：PB6;
SPI1电源初始化 PB7;
SPI2:插座P13: P13_1 VCC, P13_2 PB14-模块SPI2_MISO, P13_3 PB15-模块SPI2_MISI, P13_4 PB12-模块SPI2_NSS, P13_5 PB13-模块SPI2_SCK, P13_7 GND, P13_8 NRST（系统rest）, P13_9 PB11-模块INT,其余悬空

BOOT0:插座P12: P12_1 VCC, P12_2 BOOT0,下载短接

USART1:插座P11: USART1,P11_1 VCC, P11_2  PA10-RX1，P11_3 PA9-TX1，P11_4 GND;串口下载程

SWD:插座P10：SWD接口，P10_1 VCC,P10_2 SWCLK,P10_3 NRST,P10_6 GND,P10_7 SWDIO,1-6脚相对并排，2-7脚相对并排，其余插脚不用

WIFI:插座P9：WIFI接口，与USART2二选一，P9_1 PD0-模块REST,P9_3 模块ADC 悬空,P9_5 PD7-模块CH_PD,P9_7 模块GPIO16 悬空,P9_9 模块GPIO14 悬空,P9_11 模块GPIO12 悬空,P9_13 模块GPIO13 悬空,P9_15 VCC
	   P9_2 PD6-RX2-接模块TX 上拉470ΩVCC？,P9_4 PD5-TX2-接模块RX 上拉470ΩVCC?,P9_6 模块GPIO4 悬空,P9_8 模块GPIO5 悬空,P9_10 模块GPIO0 悬空,P9_10 模块GPIO2 悬空,P9_14-PD_16 GND

USB:插座P8：USB接口，P8_1 +5V？， P8_2 PA11-USB_DM-USB_N, P8_3 PA12-USB_DP-USB_P, P8_4 GND 需要重新设计？

UART5:插座P7：UART5，PD2-R-UART5_RX,PB8-收发切换，(重复？U6_7 U6-7为指示灯 )PC12-D-UART5_TX,AIO3、BIO3,RS485，

URAT4:插座P6：UART4, PC11-R-UART4_RX,PB9-收发切换，PC10-D-UART4_TX,AIO2（标注错误？）、BIO2，RS485 触摸屏

   插座P5：采集接口，P5_1 24VOUT-PA11, P5_2 GND, P5_3 DATA5-PA0(二氧化碳)，P5_4 SCK-PC3

   插座P4: 采集接口，P4_1 VCC, P4_2 GND, P4_3 DATA4-PA1(空气温湿度)，P4_4 SCK-PC3

   插座P3: 采集接口，P3_1 VCC, P3_2 GND, P3_3 DATA3-PC0(光照度)，P3_4 SCK-PC3

   插座P2: 采集接口，P2_1 24VOUT-PA11, P2_2 GND, P2_3 DATA1-PC2(土壤水份)，P2_4 DATA2-PC1(土壤温度)

USART3:插座P1：电源 USART3接口，P1_1 InVCC, P1_2 GND,PD9-R-USART3_RX(Remap),PD10-收发切换，PD8-D-USART3_TX(Remap),P1_3 AIO1、P1_4 BIO1，RS485

   U6 ESP8266WIFI接口
	 五路输入管脚配置:P2:PC1土壤温度(电机电流)DATA2、PC2土壤水分(液位)DATA1;P3:PC0光照度DATA3;P5:PA0二氧化碳DATA5;P4:PA1空气温湿度DATA4;PC3 为SCK;
输入输出：PE7-24V电源输出控制
触摸屏连接：P6(AI02、BI02)/P7(24V0、GND)
串口1连接：P11-1-VCC,P11-2-RXD,P11-3-TXD,P11-4-GND.说明：靠近CPU1的管脚为P11-1
采集方式，参数个数，滤波次数；采集方式：  00 无采集，01 光照 k=100 b=0，02 空气温湿度 湿度k=524.29 b=-6 温度 k=372.96 b=-46.85，
									   03 4~20ma，04 二氧化碳,
																			 05 大气压力输入（CPS131）压力：k=728.18 b=+30 单位：kpa；温度：k=436.91 b=-40 单位：℃
																			 06 开关量输入，07 频率输入，08 0~1Vdc输入，09 RS485
									   10 数字压力（cps120）液位：触摸屏k=193.16 b=30；单位：kpa,大气压力 触摸屏k=182.04  b=30；单位：kpa
																				温度=空气温度的通道cps120 触摸屏k=99.30  b=-40；单位：℃）
																			 11 差压液位;表压液位差（LWP5050GD）；车载	k=1，b=0 单位：pa
																			 12 表压液位（LWP5050GD）P4空气湿度通道
																			   沼液池液位（参数1）：k=9854.89.69 b=-1.02；单位：M(0-5.6M) 已有压力补偿。压力显示：kpa:k=1008.25,b=-10
																			   沼液车液位（参数1）：k=9766.21，b=-1.03；差压液位（参数8） k=9766.21，b=0
																			 ,温度（参数2）=空气温度通道LWP5050GD  触摸屏k=524.29  b=-40；单位：℃
	 缺省定义： 缺省定义：参数1-P3-通道0：光照度；  参数2-P4-通道1：空气湿度；参数3-P4-通道1：空气温度；    参数4-P2-DAT1-通道2：土壤水分；参数5-P2-DAT2-通道3：土壤温度；
			   参数6-P5-通道4：二氧化碳；参数7-P7(AI03、BI03)/P6(24V+0、GND)-通道5：RS485-流量；参数8-无通道：虚拟差压液位*/
//土壤水份温度传感器接法：宗色：+，兰色：—（要与屏蔽线相连）；黑色：水分；灰色：温度。
/*Modbus子站ID*/
#define SLAVE_ID                 0x01

/*Modbus功能码*/
#define READ_COIL_STATUS         0x01   /*读线圈寄存器  :不支持广播，规定要读的起始线圈和线圈量                  */
#define READ_INPUT_STATUS        0x02	/*读状态寄存器	:不支持广播，规定要读的输入起始地址，以及输入信号的数量  */
#define READ_HOLDING_REGISTER    0x03	/*读保持寄存器	:不支持广播，规定要读的寄存器起始地址及寄存器的数量      */
#define READ_INPUT_REGISTER      0x04 	/*读输入寄存器	:不支持广播，规定要读的寄存器起始地址及寄存器的数量      */
#define WRITE_SINGLE_COIL        0x05 	/*写单线圈寄存器:支持广播，规定被请求线圈的ON/OFF状态，0xFF00值请求线圈处
														 于ON状态，0x0000值请求线圈处于OFF状态，其他值对线圈无效 */
#define WRITE_SINGLE_REGISTER    0x06 	/*写单保持寄存器:支持广播，请求的预置值在查询数据区                      */
#define WRITE_MULTIPLE_COIL      0x0F	/*写多线圈寄存器:支持广播                                                */
#define WRITE_MULTIPLE_REGISTER  0x10	/*写多保持寄存器:支持广播                                                */

/*RS485收发切换*/
#define TXENABLE3		         GPIO_SetBits(GPIOD, GPIO_Pin_10)
#define RXENABLE3		         GPIO_ResetBits(GPIOD, GPIO_Pin_10)
#define TXENABLE4		         GPIO_SetBits(GPIOB, GPIO_Pin_9)
#define RXENABLE4		         GPIO_ResetBits(GPIOB, GPIO_Pin_9)
#define TXENABLE5		         GPIO_SetBits(GPIOB, GPIO_Pin_8)
#define RXENABLE5		         GPIO_ResetBits(GPIOB, GPIO_Pin_8)

#define false 0
#define true  1
#define bool u8

/*CRC低位字节表*/
const unsigned char crc_lo[256] =
{
    0x00,0xC0,0xC1,0x01,0xC3,0x03,0x02,0xC2,0xC6,0x06,0x07,0xC7,0x05,0xC5,0xC4,
    0x04,0xCC,0x0C,0x0D,0xCD,0x0F,0xCF,0xCE,0x0E,0x0A,0xCA,0xCB,0x0B,0xC9,0x09,
    0x08,0xC8,0xD8,0x18,0x19,0xD9,0x1B,0xDB,0xDA,0x1A,0x1E,0xDE,0xDF,0x1F,0xDD,
    0x1D,0x1C,0xDC,0x14,0xD4,0xD5,0x15,0xD7,0x17,0x16,0xD6,0xD2,0x12,0x13,0xD3,
    0x11,0xD1,0xD0,0x10,0xF0,0x30,0x31,0xF1,0x33,0xF3,0xF2,0x32,0x36,0xF6,0xF7,
    0x37,0xF5,0x35,0x34,0xF4,0x3C,0xFC,0xFD,0x3D,0xFF,0x3F,0x3E,0xFE,0xFA,0x3A,
    0x3B,0xFB,0x39,0xF9,0xF8,0x38,0x28,0xE8,0xE9,0x29,0xEB,0x2B,0x2A,0xEA,0xEE,
    0x2E,0x2F,0xEF,0x2D,0xED,0xEC,0x2C,0xE4,0x24,0x25,0xE5,0x27,0xE7,0xE6,0x26,
    0x22,0xE2,0xE3,0x23,0xE1,0x21,0x20,0xE0,0xA0,0x60,0x61,0xA1,0x63,0xA3,0xA2,
    0x62,0x66,0xA6,0xA7,0x67,0xA5,0x65,0x64,0xA4,0x6C,0xAC,0xAD,0x6D,0xAF,0x6F,
    0x6E,0xAE,0xAA,0x6A,0x6B,0xAB,0x69,0xA9,0xA8,0x68,0x78,0xB8,0xB9,0x79,0xBB,
    0x7B,0x7A,0xBA,0xBE,0x7E,0x7F,0xBF,0x7D,0xBD,0xBC,0x7C,0xB4,0x74,0x75,0xB5,
    0x77,0xB7,0xB6,0x76,0x72,0xB2,0xB3,0x73,0xB1,0x71,0x70,0xB0,0x50,0x90,0x91,
    0x51,0x93,0x53,0x52,0x92,0x96,0x56,0x57,0x97,0x55,0x95,0x94,0x54,0x9C,0x5C,
    0x5D,0x9D,0x5F,0x9F,0x9E,0x5E,0x5A,0x9A,0x9B,0x5B,0x99,0x59,0x58,0x98,0x88,
    0x48,0x49,0x89,0x4B,0x8B,0x8A,0x4A,0x4E,0x8E,0x8F,0x4F,0x8D,0x4D,0x4C,0x8C,
    0x44,0x84,0x85,0x45,0x87,0x47,0x46,0x86,0x82,0x42,0x43,0x83,0x41,0x81,0x80,
    0x40
};

/*CRC高位字节表*/
const unsigned char crc_hi[256] =
{
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
    0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,
    0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,
    0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,
    0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
    0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,
    0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
    0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,
    0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,
    0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
    0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,
    0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,
    0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
    0x40
};

__attribute__((section("NO_INIT"), zero_init)) static u8 Collectors[65][16];//光照度、空气湿度、空气温度、土壤水分（液位）、土壤温度（电机电流）、二氧化碳；其余保留
__attribute__((section("NO_INIT"), zero_init)) static  u8 Controllers[32][8];
//平台配置第1个控制器2100H、2104H、2108H、210CH，第2个控制器2110H、2114H、。。。；2100H-22FCH为控制器开关命令

//__attribute__((zero_init)) static u8 I_current_limit[32][8];
/*平台配置第1个控制器限流2300H、2304H、2308H、230CH，第2个控制器限流2130H、2134H、。；2300H-24FCH为4个通道的限流参数；暂时不配置（本软件无此功能）*/

__attribute__((section("NO_INIT"), zero_init)) static u8 hand_auto_flg[73][2];
/*接收平台下发的手自动切换命令：00H、00H-手动，01H、00H-自动。
	共64个单自控回路，9个批量自控回路。	2500H为第1单自控回路，2504H为第2单自控回路，。。。
	2500H-25FCH为64个单自控回路；2600H-2620H为9个批量自控回路。
	hand_auto_flg接收触摸屏及平台下发的自控切换命令和回复平台自控状态使用，hand_auto_flg[n][0]、[1]
	分别为：00H、00H手动，01H、00H自动*/
static u8 hand_auto_count[73] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
                                };//手自动状态上报平台次数，0则不上报平台
__attribute__((section("NO_INIT"), zero_init)) static u8 ZZ_Wired_flag[65];
__attribute__((section("NO_INIT"), zero_init)) static u8 ZZ_Wireles_flag[65];
/*有线、无线通信子站通道标志；0x00不存在,0x01存在；0x02曾经存在，正在检测；
	 ZZ_Wireles_flag[64]!=0表示网关初始化完成
	 ZZ_Wired_flag[64]!=0表示网关有采集功能 */

__attribute__((section("NO_INIT"), zero_init)) static u8 crtl_cmd_num[32][4];//接收到触摸屏或平台下发的控制命令后记录允许向子站下发命令的次数，按单个控制点下发
static u8 ctrl_j = 0;//crtl_cmd_num[ctrl_i][ctrl_j],向子站发控制命令记录每个控制器的控制点计数
//向子站发送控制命令次数，收到平台或触摸屏的控制命令，该单元置3，允许通过有线向子站发送3次该命令，收到子站回复该单元清0
//static u8 collector_j=0;//向采集器或控制器子站发读传感器数据点的计数;无线一次性发出读批量命令

__attribute__((section("NO_INIT"), zero_init)) static u8 crtl_cmd_numWX[32][4];
//向子站发送控制命令次数，收到平台或触摸屏的控制命令，该单元置3，允许通过无线向子站发送3次该命令，收到子站回复该单元清0
static u8 ctrl_wx_j = 0;//crtl_cmd_numWX[ctrl_i][ctrl_j],向子站发控制命令记录每个控制器的控制点计数
//向子站发送控制命令次数，收到平台或触摸屏的控制命令，该单元置3，允许通过有线向子站发送3次该命令，收到子站回复该单元清0

__attribute__((section("NO_INIT"), zero_init)) static u8 init_cmd_numYX[64];
//网关上电通过有线向子站发送查询命令的次数，单元值为0正常查询，单元值为n，重复查询n次。收到回复清0，转为正常查询
__attribute__((section("NO_INIT"), zero_init)) static u8 init_cmd_numWX[64];
//网关上电通过无线向子站发送查询命令的次数，单元值为0正常查询，单元值为n，重复查询n次。收到回复清0，转为正常查询

static u8 Query_Wired_WirelesYX = 0;//有线
static u8 set_param_flag[32];
static u8 CSH_countYX = 0;//有线初始化完成=1

#define CONTROLLERS_CMD         0x00
#define COLLECTORS_CMD          0x01
#define ZZ_QUERY_COLLECTOR      0x02
#define ZZ_QUERY_CONTROLLER     0x03

static u8  Query_Flag = CONTROLLERS_CMD;
static u8  Query_Index_Controller = 0;
static u8  Query_Index_Collector = 0;
static u8  Query_IndexZZ_C_YX = 0;
static u8  Query_IndexZZ_K_YX = 0;

u8  ReceiveData1[256];
static u8  ReportData1[128];
u8  ReceiveData2[256];
static u8  ReportData2[256];
u8  ReceiveData3[128];
static u8  ReportData3[128];
u8  ReceiveData4[256];
static u8  ReportData4[256];
static u8  ReceiveData5[128];
static u8  ReportData5[128];
extern u8   RxFlag1;		//串口1接收标志位，0正在接收，1接收完毕
extern u8   TxFlag1;		//串口1发送标志位，0正在发送，1发送完毕
extern u8   RxFlag2;		//串口2接收标志位，0正在接收，1接收完毕
extern u8   TxFlag2;		//串口2发送标志位，0正在发送，1发送完毕
extern u8   RxFlag3;    //串口2接收标志位，0正在接收，1接收完毕
extern u8   TxFlag3;		//串口3发送标志位，0正在发送，1发送完毕
extern u8   RxFlag4;		//串口5接收标志位，0正在接收，1接收完毕
extern u8   TxFlag4;		//串口5发送标志位，0正在发送，1发送完毕
extern u8   RxFlag5;		//串口5接收标志位，0正在接收，1接收完毕
extern u8   TxFlag5;		//串口5发送标志位，0正在发送，1发送完毕

extern u8   RecDataBuffer1[];
extern u8   RecLen1;
extern u8   RecDataBuffer2[];
extern u8   RecLen2;
extern u8   RecDataBuffer5[];
extern u8   RecLen5;
extern u8   RecDataBuffer3[];
extern u8   RecLen3;
extern u8   RecDataBuffer4[];
extern u8   RecLen4;
extern u8   USART1SendTCB[];
extern u8   USART1BufferCNT;
extern u8   USART2SendTCB[];
extern u8   USART2BufferCNT;
extern u8   USART3SendTCB[];
extern u8   USART3BufferCNT;
extern u8   UART4SendTCB[];
extern u8   UART4BufferCNT;
extern u8   UART5SendTCB[];
extern u8   UART5BufferCNT;

static void RxReport1(u8 len, u8 *pData);
static void RxReport2(u8 len, u8 *pData);
static void RxReport3(u8 len, u8 *pData);
static void RxReport4(u8 len, u8 *pData);
static void RxReport5(u8 len, u8 *pData);
static void RxReport1_csb_yw(u8 len, u8 *pData);
static void RxReport1_YANHUA_touch_screen(u8 len, u8 *pData);//研华触摸屏返回网关采集的8个检测参数

static u16 CRCReport4;//RxReport4函数的临时变量定义
static u16  GetCRC16(u8 *Msg, u16 Len);
static  u8 gc_i, gc_CRCHigh, gc_CRCLow;//高CRC字节初始化,低CRC字节初始化;GetCRC16函数的临时变量定义
static	u16 gc_index;		  //CRC循环中的索引;GetCRC16函数的临时变量定义

static void Send_slave_cmd(void);
static u8 bytelen3 = 0;//Send_slave_cmd函数的临时变量定义

static void WriteDataToBuffer(u8 port, u8 *ptr, u8 start, u8 len);

static u8 WriteMultipleRegister(u8 Slave_ID, u16 addr, u16 num, u8 *pData, u8 *temp);
static u16 wm_CRC_Val;//WriteMultipleRegister函数的临时变量定义
static u8  WriteSingleRegister(u8 Slave_ID, u16 addr, u8 *pData, u8 *temp);

static u8 ReadData(u8 Slave_ID, u8 function_code, u16 addr, u16 num, u8 *temp);
static u16 rd_CRC_Val;//ReadData函数的临时变量定义

/*以下为出厂网关设定参数*/
//static u8 factory_gateway_setflg=0;//当收到触摸屏《设定确定》=1，变量地址：EF_1E，返回0 ，而factory_gateway_setflg=《设定确定》=1，将factory_gateway_set[159]写入flash，随后factory_gateway_setflg=0
/*采集方式，参数个数，滤波次数；采集方式：00 无采集，01 光照 k=100 b=0，02 空气温湿度 湿度k=524.29 b=-6 温度 k=372.96 b=-46.85，
									   03 4~20ma，04 二氧化碳,05 大气压力输入，06 开关量输入，07 频率输入，08 0~1Vdc输入，09 RS485
									   10 数字液位（cps120）触摸屏k=193.16 b=30；单位：kpa
																			 ，温度=空气温度的通道cps120 触摸屏k=99.30  b=-40；单位：℃）
																			 12 表压液位（LWP5050GD）空气湿度通道 触摸屏k=9854.89 b=-0.102；单位：M 已有压力补偿
																			 12 表压液位（LWP5050GD）P4空气湿度通道
																			   沼液池液位（参数1）：k=9854.89.69 b=-1.02；单位：M(0-5.6M) 已有压力补偿。压力显示：kpa:k=1008.25,b=-10
																			   加压液位（参数1）：k=9766.21，b=-1.03；差压液位（参数8） k=9766.21，b=0
																			 ,温度（参数2）=空气温度通道LWP5050GD  触摸屏k=524.29  b=-40；单位：℃

	以下为网关采集通道5设定：
			11 差压液位;表压液位差（LWP5050GD）；车载
			13 超声波流量（常州果红军），参数7 瞬时流量,公式再除62.5,转换成单位为：立方米/小时；
		  14 电磁流量计（上海帆杨、科霸），参数7 累计流量 单位：L；
		  15 串口1超声波液位，参数8-转换液位数据；参数8的变量个数factory_gateway_set[28]设定净高，单位：cm
					16 电磁流量计-表压液位实验，参数7-累计流量 单位：L，参数8-距离数据  单位：mm；车载
					17 电磁流量计-脉冲型超声波液位实验，参数7-流量，累计流量 单位：L，参数8-距离数据  单位：mm；车载
					19 沼液车超声波液位（脉冲宽度）；参数8，单位：mm，k=1，b=0；车载
					20 沼液车高静压差压液位；参数8，单位：mm，k=1，b=0；车载
					21 沼液车电容射频液位；参数8，单位：mm，k=1，b=0；车载
					   factory_gateway_set[19]=调整高度；factory_gateway_set[20]=滤波次数；factory_gateway_set[28]=净高；factory_gateway_set[29]=清零下限
					22 RS485(风速+风向)地址：0x01福州芯仪自动化仪表有限公司+(PM1.0+PM2.5+PM10)地址:0x02广州龙戈电子科技(5个参数)
			24 DO+水温 (网关子站实现,采集网关不设定)
					25 可燃气体报警器，站地址：0x01，威海精讯畅通电子科技有限公司；数值：0x0016（25）=0.25%；k=100，b=0，报警低报：0x0018:0x0096（150=浓度*10,15%LEL）
			29 RS485土壤水分、温度、EC（水分最多可接8个，水分+温度最多可接4个，水分+温度+EC可接2个）；土壤传感器子站地址：FE、FD、FC、FB、。。。
					   factory_gateway_set[27]=29,[28]=传感器个数,[29]=1(水分)、=2（水分+温度）、=3（水分+温度+EC）
						 土壤水分：k=10,b=0;土壤温度：k=10,b=-40;土壤EC： k=10,b=0
					30 RS485 气体传感器（威海精讯畅通）
					   factory_gateway_set[27]=30,[28]=传感器个数,[29]=1;传感器站地址从1、2...；寄存器起始地址：0x00 0x06；读一个参数
						 （如果传感器个数大于1，H2S为1#，NH3为2#）
					31 研华触摸屏+水质传感器
					   factory_gateway_set[27]=31,[28]=传感器个数,[29]=1;传感器站地址从1（液位、流量）、2（电导）4（pH）、8(浊度)、10（DO/温度）；
						 流量：格仪表；电导、pH、浊度：北京博海志远科技；DO/温度：福州水位
	以下为网关采集通道1设定P4：
	18 距离检测，单位:mm，参数2，mm，k=1，b=0 ；脉冲型超声波液位;P4-CLK--Trig/TX,P4-DAT4--Echo/RX
	23 感应式雨雪传感器，单位：每三分钟雨滴数；k=1，b=0 ；P4-CLK--不接,P4-DAT4--OUT

	以下为采集器设定
			28 日照时数（通道0 光照）
						通道0   采集方式=28   参数个数=10 lux下限（5小时后）计数清零,滤波次数=100lux计数上限
							K=60，b=0 单位：分
					254（通道5） RS485土壤水分、温度、EC（水分最多可接8个，水分+温度最多可接4个，水分+温度+EC可接2个）；
						土壤传感器子站地址：FE、FD、FC、FB、。。。；土壤水分：k=10,b=0;土壤温度：k=10,b=-40;土壤EC： k=10,b=0
					   采集方式=254,参数个数=传感器个数,滤波次数=1(水分)、=2（水分+温度）、=3（水分+温度+EC）
	以下为控制器设定
		 32 通知控制器为控制采集方式（水肥机施灌区控制阀及RS485采集使用），通道4方式（第5通道），通道5（第6通道）与采集器相同。
			33 涡轮流量计采集 PC0 PA1 PA0
			34 为串口5 帆扬流量计水肥总管流量采集
			35 为串口5 中航LED字符卡 采集数据显示，[28]为第几组参数，[29]为参数个数，滚动显示
			新增 36 中航字符卡，factory_gateway_set[28] 表示下接的采集器个数，01表示只有1个采集器，02表示两个采集器，采集五要素，03表示三个采集器，采集5+1个参数
	采集方式最后编码为：35；
	*/
u8  factory_gateway_set[255] =            //触摸屏下发出厂网关设定参数需要写入flash0x0805 C000，从256k开始写入，0x0804加1，则增加64k,每次写入必需2Kbyte
{
    0x01,  //网络类型=1 电信，2 移动，3 以太网，4 WiFi，5 USB，0 无连接；触摸屏发送命令 EF 06 00 00 01 00 9F 14，功能号：0x06,地址：EF_00 ,factory_gateway_set[0]
    0x01,  //协议类型=1 TCP/IP;2 MQTT（酸梅果）; 3 SDK（极码）; 4 电信MQTT（标准MQTT）;其它；触摸屏：功能号：0x06,地址：EF_01,factory_gateway_set[1]
    0x00,0x00,0x00,0xf8, //网关ID3~0;触摸屏：功能号：0x06,地址：EF_02~EF_05,factory_gateway_set[2]~[5]
    0x00,0x00,0x00,0x00, //采控器ID3~0;功能号：0x06,地址：EF_06~EF_09,factory_gateway_set[6]~[9]
    0x00,  //GPS定位=1，有GPS定位功能，0则无；功能号：0x06,地址：EF_0A,factory_gateway_set[10]
    0x01,  //网关采集=1，有网关采集功能，0则无；功能号：0x06,地址：EF_0B,factory_gateway_set[11]
    33,01,00,  //网关采集通道0参数设定：采集方式，参数个数，滤波次数；采集方式：00 无采集，01 光照，02 空气温湿度，03 4~20ma，04 二氧化碳
    33,02,00,  //网关采集通道1参数设定; 采集方式：05 大气压力输入，06 开关量输入，07 频率输入，08 0~1Vdc输入，09 RS485 ；见上面采集方式
    00,01,15,  //网关采集通道2参数设定; 触摸屏：功能号：0x06,地址：EF_0C~EF_1D,factory_gateway_set[12]~[29]
    8,01,15,  //网关采集通道3参数设定;
    33,01,00,  //网关采集通道4参数设定;
    34,03,16,	//网关采集通道5参数设定;factory_gateway_set[27]~[29]  33水肥涡轮流量计，34上海帆扬电磁流量计测总管流量

    //factory_gateway_set[29]指定时间
    0x15,0x31,0x31,0x35,0x2E,0x32,0x33,0x39,0x2E,0x31,0x33,0x34,0x2E,0x31,0x36,0x35,0x3A,0x30,0x30,0x35,0x30,0x32,//信产平台
    // 0x15,0x31,0x34,0x30,0x2E,0x31,0x34,0x33,0x2E,0x30,0x32,0x33,0x2E,0x31,0x39,0x39,0x3A,0x30,0x31,0x38,0x38,0x33,//极码平台
    //《实际字符串长度，IP地址：端口号》实际字符串长度为编程时需要加入的
    //触摸屏下发命令：EF 10 00 1F 00 0F 1D 4D 43 47 53 53 54 52 3A 31 31 35 2E 32 33 39 2E 31 33 34 2E 31 36 35 3A 30 30 35 30 32 91 76
    //信产平台，写《IP地址端口》，15个变量21个字节，此处为：115,239,134,165:00502 ，前8个字节为MCGS自己加入：4D 43 47 53 53 54 52 3A（MCGSSTR:）；
    //SDK平台IP地址：140.143.023.199:01883 极码平台
    //爱WiFi平台地址:183.159.037.034:03010 ;0x31,0x38,0x33,0x2E,0x31,0x35,0x39,0x2E,0x30,0x33,0x37,0x2E,0x30,0x33,0x34,0x3A,0x30,0x33,0x30,0x31,0x30,
    //编程以：字节数-8=实际输入的字节数
    //触摸屏：功能号：0x10,地址：EF_1F~EF_29,IP地址:端口号为factory_gateway_set[30]~[51]

    0x10,0x4A,0x4D,0x57,0x47,0x49,0x44,0x30,0x31,0x3A,0x53,       //0x04,0x36,0x35,0x30,0x31
    0x44,0x4B,0x30,0x30,0x30,0x30,0x0E,0x0D,0x0C,0x0B,
    0x0A,0x09,0x08,0x07,0x06,0x05,0x04,0x03,0x02,0x01,
    //	《实际字符串长度，客户端ID》,实际字符串长度为编程时需要加入factory_gateway_set[52]=0x04
    //写《客户端ID》，15个变量30个字节，此处为16个字节：JMWGID01:SMG0000，前8个字节为MCGS自己加入：（MCGSSTR:）4个变量；共19个变量
    //JMWGID02:SDK0000 共16个字节 0x4A,0x4D,0x57,0x47,0x49,0x44,0x30,0x31,0x3A,0x53,
    //0x44,0x4B,0x30,0x30,0x30,0x30
    //触摸屏：功能号：0x10,地址：EF_2A~EF_38,factory_gateway_set[52]~[82]

    0x1B,0x76,0x31,0x2F,0x64,0x65,0x76,0x69,0x63,0x65,0x73,
    0x2F,0x6D,0x65,0x2F,0x72,0x70,0x63,0x2F,0x72,0x65,
    0x71,0x75,0x65,0x73,0x74,0x2F,0x2B,0x03,0x02,0x01,
    0x73,0x75,0x62,0x73,0x63,0x72,0x69,0x62,0x65,0x20,
    //	《实际字符串长度，订阅主题》,实际字符串长度为编程时需要加入factory_gateway_set[83]=0x1B
    //	写《订阅主题》，20个变量40个字节，前8个字节为MCGS自己加入：（MCGSSTR:）4个变量;共24个变量;
    //触摸屏：功能号：0x10,地址：EF_39~EF_4C,factory_gateway_set[83]~[123]；共定义了27个字节v1/devices/me/rpc/request/+

    0x17,0x76,0x31,0x2F,0x64,0x65,0x76,0x69,0x63,0x65,0x73,
    0x2F,0x6D,0x65,0x2F,0x74,0x65,0x6C,0x65,0x6D,0x65,
    0x74,0x72,0x79,0x07,0x06,0x05,0x04,0x03,0x02,0x01,
    //《实际字符串长度，推送主题》,实际字符串长度为编程时需要加入factory_gateway_set[124]=0x11
    //写《推送主题》，15个变量30个字节，此处为17个字节：/JMWGID0000/SMG01，前8个字节为MCGS自己加入：（MCGSSTR:）；
    //触摸屏：功能号：0x10,地址：EF_4D~EF_5B,factory_gateway_set[124]~[154]；共定义了23个字节 v1/devices/me/telemetry

    0x09,0x6A,0x69,0x61,0x6F,0x73,0x69,0x38,0x32,0x30,0x05,          //jiaosi502
    0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,
    0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x2A,0x2B,
    //《实际字符串长度，用户名》,实际字符串长度为编程时需要加入factory_gateway_set[155]=0x05
    //	写《用户名》，16个变量32个字节，此处为5个字节：smg01，前8个字节为MCGS自己加入：（MCGSSTR:）；
    //jiaosi502 共9个字节  0x6A,0x69,0x61,0x6F,0x73,0x69,0x35,0x30,0x32
    //jiaosi820 共9个字节  0x6A,0x69,0x61,0x6F,0x73,0x69,0x38,0x32,0x30
    //触摸屏：功能号：0x10,地址：EF_5C~EF_6A,factory_gateway_set[155]~[187]

    0x00,0x73,0x6D,0x67,0x36,0x30,0x33,0x00,0x01,0x02,0x03,
    0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,
    0x0E,0x0F,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19
    //《实际字符串长度，用户名密码》,实际字符串长度为编程时需要加入的
    //写《用户名密码》，16个变量32个字节，此处为6个字节：smg603,(0x73,0x6D,0x67,0x36,0x30,0x33)，前8个字节为MCGS自己加入：（MCGSSTR:）；
    //触摸屏：功能号：0x10,地址：EF_6B~EF_6D,factory_gateway_set[188]=0x06如果为0不需要密码,factory_gateway_set[188]~[220]
};//保留33个字节为3个手机号码，每个号码为11个字节

/*网关连接平台定义*/
//通讯模块接口控制定义
#define RESET_CTL_ENABLE		 GPIO_SetBits(GPIOD, GPIO_Pin_13)
#define RESET_CTL_DISABLE		 GPIO_ResetBits(GPIOD, GPIO_Pin_13)
#define TERM_ON_CTL_ENABLE       GPIO_SetBits(GPIOD, GPIO_Pin_14)
#define TERM_ON_CTL_DISABLE      GPIO_ResetBits(GPIOD, GPIO_Pin_14)
#define WAKEUP_CTL_ENABLE        GPIO_SetBits(GPIOD, GPIO_Pin_15)
#define WAKEUP_CTL_DISABLE       GPIO_ResetBits(GPIOD, GPIO_Pin_15)

/*以下为管理员子站设定，共8组（8页），每页设定4个控制器，每个控制器4个通道，每个通道3个参数6个字节，共有字节数6*4*4=96个字节
	3个参数分别为：最大限流、延时启动时间、持续运行时间,采集器8个一组，*/

static u8 ctrlslave_param_set[4][24];
//控制器设定的每个设定页面设定4个控制器（共8个设定页面），以控制器为单位分为4组,每组12个参数（每个控制通道3个参数）
static s8 ctrlslave_param_flg[4] = { 0,0,0,0 };
//记录网关有线向控制子站下发参数的次数，在收到触摸屏下发的参数，对应子站的设定标志为3，每向子站下发一次减1,或收到子站回复清0.
static s8 ctrlslave_param_flgWX[4] = { 0,0,0,0 };
//记录网关无线向控制子站下发参数的次数，在收到触摸屏下发的参数，对应子站的设定标志为3，每向子站下发一次减1,或收到子站回复清0.
static u8 set_finish_flg = 0;
//设置确认标志，当set_finish_flg=0，向子站发送设定参数，发送完成返回0给触摸屏，即ctrlslave_param_flg[i]=0（i=0~3）；有线、无线共用
static u8 set_finish_flgEF = 0;
static u8 set_finish_flgEE = 0;
static u8 set_finish_flgF1 = 0;
static u8 set_finish_flgF2 = 0;
static u8 set_finish_flgF3 = 0;
static u8 slave_xiabiao_I = 0, slave_xiabiao_J = 0;//无线、有线共用
static u8 YX_Slave_ID = 0;
static u8 WX_Slave_ID = 0;

static u8 first_xiabiao_I = 0;//记录当前页面设定第1个子站设定的子站地址;有线
static u8 firstwx_xiabiao_i = 0;//记录当前页面设定第1个子站设定的子站地址;无线

static u8 slave_set_finish = 0;//用于set_slave_param函数中判断有线下发子站设定参数是否全部完成，没有完成不等于0
static u8 slave_set_finishWX = 0;//用于set_slave_paramWX函数中判断无线下发子站设定参数是否全部完成，没有完成不等于0
static u16 slave_param_addr = 0;//计算16位变量的地址（有线、无线共用）
static void set_slave_param(void);//网关设定子站参数函数（有线）
static void set_slave_paramWX(void);//网关设定子站参数函数（无线）
static u8 slave_set_flg = 0;
//触摸屏下发的设定确认变量，slave_set_flg!=0表示有线确定设置，网关可以向子站下发参数了。
//slave_set_flg=1控制器有线设定;slave_set_flg=2采集器有线设定
static u8 slave_set_flgWX = 0;
//触摸屏下发的设定确认变量，slave_set_flgWX!=0表示无线确定设置，网关可以向子站下发参数了。
//slave_set_flgWX=1控制器无线设定;slave_set_flgWX=2采集器无线设定
static s8 cjqslave_param_flg[8] = { 0,0,0,0,0,0,0,0 };
//记录网关有线向采集及控制子站下发参数的次数，在收到触摸屏下发的参数，对应子站的设定标志为3，每向子站下发一次减1,或收到子站回复清0.
static s8 cjqslave_param_flgWX[8] = { 0,0,0,0,0,0,0,0 };
//记录网关无线向采集及控制子站下发参数的次数，在收到触摸屏下发的参数，对应子站的设定标志为3，每向子站下发一次减1,或收到子站回复清0.
static u8 cjqslave_param_set[8][36];
//采集器和控制器采集页面设定8个采集器或控制器（共8个页面），以采集器或控制器为单位分为8组，每组18个参数（每个采集通道3个参数，6通道采集器）；有线、无线共用
static u8 set_slave_Index = 0;//有线记录下发子站参数的子站偏移地址，子站地址=first_xiabiao_I+set_slave_Index
static u8 set_slave_IndexWX = 0;//无线记录下发子站参数的子站偏移地址，子站地址=firstWX_xiabiao_i+set_slave_IndexWX
static u8 cjym_ID = 0;//采集器页面ID
static u8 cjym_old_ID = 0;
static u8 kzym_ID = 0;//控制器页面ID
static u8 kzym_old_ID = 0;

static u8   send_flg = 0x00;
//发送命令标志位，没有通信模块（串口2）时 send_flg=0x00
//send_flg=0x01通信模块初始化，0x02和0x03为正在向平台上报的AT指令和数据指令,0x04和0x05正在回复平台下发控制指令的AT指令和数据指令。
static u8 send_message_type = 0x00;
//0x01为通信模块初始化的at指令类型，0x02为向平台回复控制指令的类型，0x03为向平台上报数据的类型，0x04类型为快速回复极码平台的控制指令
//在收到通信模块ERROR返回时使用。
static u8 next_atdata_flg = 0x00;//在接收到通信模块的OK回复时，下一次要执行的程序

#define CMD_WAIT_TIME    6000  //向CDMA发送命令最大等待时间，ms单位
#define DATA_WAIT_TIME   5000  //向CDMA发送数据命令最大等待回复时间，ms单位
#define Ethernet_WAIT_TIME   300  //向Ethernet模块发送数据命令等待时间，ms单位
#define INTER_SEND       5000  //每条上报的间隔
#define INTER_MESS       150   //5ms(（20ms正常0x14 TCP）   发送信息之后等待10ms)

static void Net_connect_init(void);//平台连接初始化

static void cdma_tcp_init(void);//电信TCP/IP连接
//static void cdma_mqtt_sdk_init(void);//电信mqtt_sdk连接
//static void cdma_other_init(void);//电信other连接

static void gprs_tcp_init(void);//移动TCP/IP连接；该函数包括了 移动mqtt连接 gprs_mqtt_init，移动sdk连接 gprs_sdk_init
static void gprs_other_init(void);//移动other连接

static void Ethernet_tcp_init(void);//电信TCP/IP连接
static void Ethernet_mqtt_sdk_init(void);//电信mqtt_sdk连接
static void Ethernet_other_init(void);//电信other连接

static void send_platform(void);//向平台发送数据

static void cdma_tcp_send(void);//电信TCP/IP发送数据;包含移动TCP/IP发送数据 gprs_tcp_send
static void cdma_mqtt_send(void);//电信mqtt发送数据;包含移动mqtt发送数据 	gprs_mqtt_send
static void cdma_sdk_send(void);//电信sdk发送数据	;包含移动sdk发送数据 	gprs_sdk_send
//static void cdma_other_send(void);//电信other发送数据

static void gprs_other_send(void);//移动other发送数据

static void Ethernet_tcp_send(void);//电信TCP/IP发送数据
static void Ethernet_mqtt_send(void);//电信mqtt发送数据
static void Ethernet_sdk_send(void);//电信sdk发送数据
static void Ethernet_other_send(void);//电信other发送数据

static void cdma_cmd_receive(u8 len, u8 *pBuf);//电信TCP/IP接收CDMA命令返回处理函数
static void cdma_tcp_receive(u8 len, u8 *pBuf);//电信TCP/IP接收平台数据处理处理函数;包含了 移动TCP/IP接收平台数据 gprs_tcp_receive
static void cdma_SMS_receive(u8 len, u8 *pBuf);//电信TCP/IP接收短信处理函数;包含了 移动TCP/IP接收短信处理函数 gprs_SMS_receive
static void cdma_mqtt_receive(u8 len, u8 *pBuf);//电信mqtt接收平台数据;包含了移动mqtt接收平台数据 gprs_mqtt_receive
static void cdma_sdk_receive(u8 len, u8 *pBuf);//电信sdk接收平台数据;包含了移动sdk接收平台数据 	gprs_sdk_receive
//static void cdma_other_receive(u8 len,u8 *pBuf);//电信other接收平台数据

static void gprs_cmd_receive(u8 len, u8 *pBuf);//移动TCP/IP接收CDMA命令返回处理函数
static void gprs_other_receive(u8 len, u8 *pBuf);//移动other接收平台数据

static void Ethernet_tcp_receive(u8 len, u8 *pBuf);//电信TCP/IP发送数据
static void Ethernet_mqtt_receive(u8 len, u8 *pBuf);//电信mqtt发送数据
static void Ethernet_sdk_receive(u8 len, u8 *pBuf);//电信sdk发送数据
static void Ethernet_other_receive(u8 len, u8 *pBuf);//电信other发送数据

static void wg_reply_cmd(void);//回复平台控制命令及查询
static void jm_platform_reply(void);//为了平台下发控制命令后快速得到返回状态而设置
static unsigned char jm_reply_cmd[16] = "{H2100:0000}";//返回极码平台下发的控制命令

static void cdma_tcp_reply(void);//电信TCP/IP回复平台控制命令及查询；包含移动TCP/IP回复平台控制命令及查询gprs_tcp_reply
static void cdma_mqtt_reply(void);//电信mqtt回复平台控制命令及查询；包含移动mqtt回复平台控制命令及查询gprs_mqtt_reply
static void cdma_sdk_reply(void);//电信sdk回复平台控制命令及查询	；包含	移动sdk回复平台控制命令及查询	gprs_sdk_reply
static void cdma_other_reply(void);//电信other回复平台控制命令及查询

static void gprs_other_reply(void);//移动other回复平台控制命令及查询

static void Ethernet_tcp_reply(void);//Ethernet TCP/IP回复平台控制命令及查询
static void Ethernet_mqtt_reply(void);//Ethernet mqtt回复平台控制命令及查询
static void Ethernet_sdk_reply(void);//Ethernet sdk回复平台控制命令及查询
static void Ethernet_other_reply(void);//Ethernet other回复平台控制命令及查询

static u8   cmd_flg = 0x03; //初始化平台连接步骤标志位，0xFF表示连接完成
static u8   module_send_flg = 0;//cdma_tcp_send()，上报的标志。0xFF为默认值

/*网关初始化定义*/
static void wg_init_readflash(void);//网关初始化读flash
static u8   net_connect_count = 0;

/*网关向平台发送数据定义*/
static unsigned char real_send[256]; //上报数据
static u8 sendnum_mflg = 0x00; //sendnum_mflg 控制采集器、控制器上报数据顺序,sendnum_mflg是记录第几组上报
static void send_at_cdma(unsigned char send_data_len[], u8 at_cmd_len);		//发送数据的at指令到通讯模块，CDMA：at_cmd_len=21，GPRS:at_cmd_len=15
static void send_at_gprs(unsigned char send_data_len[], u8 at_cmd_len);		//发送数据的at指令到通讯模块，CDMA：at_cmd_len=21，GPRS:at_cmd_len=15
static void send_data_module(unsigned char send_data[], unsigned char send_data_len[]);		//发送数据到CDMA;只能在send_at_cdma函数执行完成才能执行
static void send_Ethernet_module(unsigned char send_data[], unsigned char send_data_len[]);		//发送数据到Ethernet模块;

static void alter_send(u8 n, u8 flg);
static void alter_hand_auto(u8 n, u8 flg);
static void chge_coltsnd(u8 flg);
static unsigned char at_send_cdma[128] = { "AT^IPSENDEX=1,2,094\r\n" }; //CDMA为"AT^IPSENDEX=1,2,094\r\n" ;向平台发送AT指令,MQTT为94，TCB为75。(暂时设定）
static unsigned char at_send_gprs[128] = { "AT^SISW=0,094\r\n" }; //CDMA为"AT^IPSENDEX=1,2,094\r\n" ;向平台发送AT指令,MQTT为94，TCB为75。(暂时设定）
static unsigned char reply_xiafa_data[128];//向平台回复收到的数据，reply_xiafa_cmd存放的是回复命令

//手机短信定义
static bool send_mess = false;  //发送短信上报标志
static u8 main_call[11] = { 0x31,0x33,0x39,0x30,0x35,0x37,0x31,0x34,0x36,0x37,0x32 };// {"13905714672"}
static u8 voice_call[11] = { 0x31,0x33,0x33,0x37,0x36,0x38,0x31,0x32,0x39,0x31,0x30 }; // {"13376812910"}
static u8 third_call[11] = { 0x31,0x35,0x31,0x36,0x38,0x34,0x31,0x33,0x33,0x32,0x31 }; //{"15168413321"}
static bool is_number(u8 *sorce, u8 len);//短信使用
static u8 match_str(u8 *dst, u8 dst_len, u8 *sor, u8 sor_len);//如果匹配返回相同段的最后一位的地址，否则返回0

void initialRealSendBuff(void);//初始化平台上报数据数组
static void handlecmd(u8 *reply_xiafa_cmd, u16 len);//处理平台下发指令
static void set_ctrl_data(u8 *_addr, u8 *_data, u8 len);		//len==2

//SDK定义
static char SDK_topicString_pub1[128] = "v1/devices/me/rpc/response/";//27个字符，MQTT发送的主题/gateway/v01/pub/EDF0F8AF681C4D30AAC4
void mqtt_publish1(unsigned char *real, int pub_len);//推送主题1
static u16 ctrl_adrr, ctrl_cmd, offset_addrX, offset_addrY;
static u8 ctrl_key[4], ctrl_value[4];
static char *mqtt_bcmd;//mqtt返回命令指针
static	u8 mqtt_bcmdxb;//mqtt返回指令下标
static	u8 sdk_ctrl_reply[128];
static	u8 sdk_len, sdk_bcmdxb;
static	u8 send_count = 0;
//mqtt定义
void mqtt_connect(void);
void mqtt_subscribe(void);//MQTT主题订阅
void mqtt_publish(unsigned char *real, int pub_len);//MQTT主题订发送及内容
static u8 mqtt_len;

/*该函数将要发送平台的2字节起始地址(keyX_addr)、2字节的数据int型数组(value)及参数个数n转换为：key1:value1，key2:value2，...
	keyn:valuen 的ASC码,并封装成{key1:value1，key2:value2，... keyn:valuen}送到key_data_send数组中*/
static u8 made_keyX_value(u16 keyX_addr, u8 *value, u8 n, u8 *key_data_send);

/*该函数将要发送平台的2字节起始地址(keyX_addr)、2字节的数据int型数组(value)及参数个数n转换为：{"key1":value1}，{"key2":value2}，...
	keyn:valuen 的ASC码,并封装成[{"key1":value1}，{"key2:value2}，... {"keyn":valuen}]送到key_data_send数组中*/
static u8 made_keyX_value4(u16 keyX_addr, u8 *value, u8 n, u8 *key_data_send);

/*该函数将要发送平台的2字节起始地址(keyX_addr)、2字节的数据int型数组(valueN)转换成4字节数据数组(valueF)及参数个数n转换为：key1:valueF1，key2:valueF2，...
	keyn:valueFn 的ASC码,并封装成{key1:valueF1，key2:valueF2，... keyn:valueFn}送到key_data_send数组中;valueFN的值为XXXX.XX*/
static u8 made_keyX_valueF(u16 keyX_addr, u8 *value, float *valueF_k_a, u8 n, u8 *key_data_send);

/*该函数将要发送平台的2字节起始地址(keyX_addr)、2字节的数据int型数组(valueN)转换成4字节数据数组(valueF)及参数个数n转换为：{"key1":valueF1}，{"key2":valueF2}，...
	{"keyn:valueFn"} 的ASC码,并封装成{{"key1":valueF1}，{"key2":valueF2}，... {"keyn":valueFn}}送到key_data_send数组中;valueFN的值为XXXX.XX*/
static u8 made_keyX_valueF4(u16 keyX_addr, u8 *value, float *valueF_k_a, u8 n, u8 *key_data_send);

//433MHZ无线定义
static u8  SI4463_Channel = 0;
//static void	Get_WX_Channel(void);//433MHZ无线信道选择，共用32个信道
static void Clear_Buf(u8 buf[], u8 length, u8 data);
extern u8 	SI4463_RxBUFF[];
extern u8 	SI4463_RxLenth;
extern u8 	Int_BUFF[];
static void SI4463Receive(u8 len, u8 *pData);
static void SI4463_SENDZZ(void);
static u8 SI4463_TxBUFF[128];
static u8  WX_Query_Flag = CONTROLLERS_CMD;
static u8  Query_Index_ControllerWX = 0;
static u8  Query_Index_CollectorWX = 0;
static u8  Query_IndexZZ_C_WX = 0;
static u8  Query_IndexZZ_K_WX = 0;
static u8 Query_Wired_WirelesWX = 0;//无线
//static u8  Query_Index_setX  = 0;
static u8 WX_len = 0;
static u8 CSH_countWX = 0;//无线初始化完成=1
u8 close_433MHZ = 1;//不使用433MHZ通信或433MHZ通信出问题，则close_433MHZ=0

//子站地址及无线信道设定
/*set_slaveID_channel[]={老地址； 新地址； 老信道； 新信道； 子站仿采集器数量； 子站仿控制器数量； 仿采集器子站始地址；
					 仿控制器子站始地址;仿采变量起址;仿采变量数量;仿控变量起址;仿控变量数量;仿控电流起址;仿控电流数量;
					 采集器串口3下位机采集子站地址;采集器串口3下位机控制子站地址};
特别说明：采集器作为主站，利用串口3管理其下位机，采集下位机子站地址与控制下位机子站地址通常设置相同;也可以分别设定，增加灵活性*/
static u8 set_slaveID_channel[16] = { 240,0,65,1,0,0,0,0,0,0,0,0,0,0,0,0 };
static u8 slaveID_channel_flg = 0;//设定完成标志，收到触摸屏子站地址设定命令赋值5，完成子站设定值为0
static u8 slaveID_channel_flgWX = 0;//设定完成标志，收到触摸屏子站地址设定命令赋值5，完成子站设定值为0
__attribute__((section("NO_INIT"), zero_init)) static u8 online_slaveID[64];//有线在线子站的地址
__attribute__((section("NO_INIT"), zero_init)) static u8 online_slaveID_WX[64];//无线在线子站的地址
__attribute__((section("NO_INIT"), zero_init)) static u8  ZZ_temp_stateYX;
__attribute__((section("NO_INIT"), zero_init)) static u8  ZZ_temp_stateWX;

//传感器采集数据定义开始
extern __IO uint16_t ADC_ConvertedValue[];
static void wgcollector_data(void);  //传感器采集数据
static u8  ReadDataCNT = 0;
static u8  TD_param_num = 0;
__attribute__((section("NO_INIT"), zero_init)) static u8  wgcollector_data_buff[16];
static void startadc(void);
#define MEASURE_PERIOD          800
#define TX5_CMD_PERIOD          2000
#define IO_CMD_PERIOD          3000

/*ADC采集函数定义*/
u16 Get_Adclvbo(u8 TD_Xnumber, u16 TD_xiaxian, u16 TD_shangxian);	//采集100个数据，排序取40~60平均值
u16 First_Getaverage(u8 td_xnumber, u8 maxlvbo_xnumber, u16 temp_adc);//初始化平均值
u16 TD_Getaverage(u8 td_xnumber, u8 tdlvbo_xnumber, u16 temp_xadc, u8 tdcycle_xi);//ADC通道求平均值
u16 SelectTD(u8 TD_number);//选择通道，采集各个通道的值
/*求通道平均值定义*/
const u8 MaxTD_number = 3;		  //2个ADC采集通道
const u8 Maxlvbo_number = 50;	  //平均数组的最大次数

static u8 TDlvbo_number[MaxTD_number] = { 50,50,50 };	//通道0的50次数据求平均值，4秒/次
//通道1的50次数据求平均值，4秒/次
static  u16 Adc_average[MaxTD_number][Maxlvbo_number]; //ADC求平均值数组，存放历史采集数据
static u8 tdcycle_i[MaxTD_number] = { 0,0,0 }; //循环次数数组，0~TDlvbo_number[i]-1，i为通道号，i=0、1、....
static u8 First_adc_average[MaxTD_number] = { 0,0,0 }; //首次滤波，当First_adc_average=0时，调用First_Getaverage(u8 TD_number,u8 Maxlvbo_number,u16 temp_adc)
//否则调用  TD_Getaverage(u8 TD_number,u8 TDlvbo_number[TD_number],u16 temp_adc,u8 tdcycle_i[TD_number])
static u16 collector_temp, temp_adc, temp_flow, temp_level; //传感器检测数据暂存变量
static u16 dp_temp_level;
static u16 dr_temp_level, lag_value = 0;
static  u32 PA1_pulse_time;
static u16 temp_level_up = 1500;
static u8   TD_number;//ADC通道号
//传感器采集数据定义结束
//串口1、5应用程序定义
static void IOxh_send_cmd(void);
const u8 uart1_cmd_csb[1] = { 0x55 }; //超声波液位计
//static u8 uart1_data_buf[2]={0x00,0x00};
static void uart5_send_cmd(void);
const u8 uart5_cmd_csb[8] = { 0x01,0x03,0x00,0x0A,0x00,0x02,0xE4,0x09 }; //超声波流量计（果红军）
const u8 uart5_cmd_dc[8] = { 0x01,0x03,0x00,0x03,0x00,0x02,0x34,0x0B }; //上海帆科霸流量计
const u8 uart5_cmd_fydc[8] = { 0x01,0x03,0x00,0x00,0x00,0x02,0xC4,0x0B }; //上海帆杨电磁流量计查询瞬时流量
const u8 uart5_cmd_tzkd[8] = { 0x01,0x03,0x00,0x00,0x00,0x02,0xC4,0x0B }; //上海帆杨电磁流量计查询瞬时流量

const u8 uart5_cmd_LEL[8] = { 0x01,0x03,0x00,0x06,0x00,0x01,0x64,0x0B }; //可燃气体报警器，站地址：0x01，威海精讯畅通电子科技有限公司
static u8 uart5_data_buf[6] = { 0x00,0x00,0x00,0x00,0x00,0x00 };
//串口5多传感器定义;采集方式=22
static u8 switch_cmd_addr = 0;
const u8 uart5_cmd_PM[8] = { 0x02,0x03,0x00,0x00,0x00,0x03,0x05,0xF8 }; //(PM1.0+PM2.5+PM10)广州龙戈电子科技
static u8 data_buf_PM[6];
const u8 uart5_cmd_FSFX[8] = { 0x01,0x03,0x00,0x01,0x00,0x03,0x54,0x0B }; //(风向（2字节整型）+风速（4字节浮点）)地址：0x01福州芯仪自动化仪表有限公司
static u8 data_buf_FXFS[4];
static union
{
    float  wind_speed_float;//风速浮点数
    u8 wind_speed_int[4];
} data_buf_FS;

static u8 water_param_buf[16];
static union
{
    float  water_param_float;//风速浮点数
    u8 water_param_int[4];
} water_param_data;

__attribute__((section("NO_INIT"), zero_init))  u8 report_last_rain[180];
static u8 switch_cmd_TR485_addr = 0xFE;
static u8 switch_cmd_RS485_addr = 0x01;
static u8 switch_cmd_RS485_CNT = 0x00;
static u8 data_RS485[8][6];

//其它定义
static u8 halt_module = 0;
static u8 halt_RxReport2 = 0;
static u16 dword_asc_hex(u8 *dword_asc);
static void bytelen_to_asc(u8 *zfc, u8 len);//把1个字节十六进制数转换为十进制数值的asc码
static void byte_to_asc(u8 byte_hex, u8 *byte_asc);//把1个字节十六进制数转换为对应asc码
static void cycle_cmd(void);//测试各运行程序事件是否正常
static u8 error_num = 0;//AT指令执行错误的次数
#define MAX_ERROR_NUM    0x05   //AT指令最大允许错误重复执行的次数
#define OPEN_OUT24V		         GPIO_SetBits(GPIOE, GPIO_Pin_7)
#define CLOSE_OUT24V		         GPIO_ResetBits(GPIOE, GPIO_Pin_7)
static u8 RxReport2_step_flg = 0;//RxReport2_step_flg!=0表示收到的命令已处理
static u8 RxReport2_useless_count = 0;//记录无效命令的次数，大于250次则重新初始化平台通信模块
static u8 RxReport1_step_flg = 0;//RxReport1_step_flg!=0表示收到的命令已处理
static char message_len_char1[3];
static u8 send_message_len1;
static char message_len_char2[3];
static u8 send_message_len2;
static u8 RxReport2_len = 0;
static u8 RxReport3_len = 0;
static u8 RxReport4_len = 0;
static u8 RxReport5_len = 0;
static u8 cj_init_value[16] = { 0x00,0x00,0x4A,0x0C,0x42,0x44,0x06,0x01,0x62,0x02,0x00,0x00,0x00,0x00,0x00,0x00 }; //为了初始化值显示0，对下列参数设置零点
/* 光照度=X/100+0;空气湿度=X/524.29-6;空气温度=X/372.96-46.85;土壤水分=X/10.43-25.12;土壤温度=X/8.69-70.14(-40~80℃);二氧化碳=X/1-0;其它=X/1-0*/
static u8 kzcj_init_value[16] = { 0x06,0x01,0x06,0x01,0x06,0x01,0x06,0x01,0x06,0x01,0x06,0x01,0x06,0x01,0x06,0x01 }; //为了初始化值显示0,262~1305对应0~10A
static u16 send_Collectors_time =500;
u16 level1_temperature;
u16 level2_temperature;
static u16 absolute_pressure_upper;
static u16 absolute_pressure_lower = 9950;
__attribute__((section("NO_INIT"), zero_init))  u16 absolute_pressure_zero[50];
static u16 pressure_correct = 20;
//浮点数定义开始
static union
{
    float k_b_float[65][8 * 2];//zero_rang.k_b_float[65]
    u8 array_k_b[65 * 8 * 2][4];
} zero_rang;

static u8 float_to_string(float data, u8 *str);
//浮点数定义结束
//USART3 DMA通信定义
static void WriteDataToDMA_BufferTX1(uint16_t size);
static void WriteDataToDMA_BufferTX2(uint16_t size);
static void WriteDataToDMA_BufferTX3(uint16_t size);
static void WriteDataToDMA_BufferTX4(uint16_t size);

//频率检测定义
__attribute__((section("NO_INIT"), zero_init)) u16  TIM2_FrequencyPA0[61];//采用TIM2输入通道1（CC1）捕获方式；二氧化碳
__attribute__((section("NO_INIT"), zero_init)) u16  TIM2_FrequencyPC0[61];//EXTI_Line0中断方式，光照度
__attribute__((section("NO_INIT"), zero_init)) u16  TIM2_FrequencyPA1[61];//EXTI_Line1中断方式,空气温湿度
u8   freq_I;
u8 TIM5_pulsePA1_NUM = 32;
u8 level_num_err = 0;//如果20次发送检测请求命令，没有收到TIME5D1中断,则系统重启。在TIME5中断程序中level_num_err清零;在IOxh_send_cmd函数中加1
__attribute__((section("NO_INIT"), zero_init)) u32  TIM5_pulsePA1[50];//PA1脉冲宽度，单位：us
static u32 TIM5_sort_pulsePA1[50];//PA1脉冲宽度，单位：us
static  s16 temp_levelS;
//读回子站设定参数定义
static u8 formula_param_finish[22];
static u8 formula_read_finishF1 = 1;
static u8 slave_CJparam_set[65][36];
static u8 collector_param_finish[11];
static u8 collector_read_finishF2 = 1;
static u8 slave_KZparam_set[32][24];
static u8 ctrl_param_finish[4];
static u8 ctrl_read_finishF3 = 1;
//爱wifi MD5定义
static u8 md5_devId_devKey[64] = { "testCode1-DTU-20170925-000068e8f64bff906049dea06e2646af9701a7" };
//static u8 md5_devId_devKey[68]={"testCode1-DTU-20170925-000068e8f64bff-9060-49de-a06e-2646af9701a7"};
static u8 md5_password[16];
static u8 md5_password_ascii[32];
//水肥机定义
static union   //子站地址0x51
{
    float set_float[28];//水肥机管道布置及PID设定
    u8 set_int[28][4];
} fertigation51;
static u8 set_fertigation51_finish[28];
static u8 read_fertigation_finish51;
static void chge_fertisnd(u8 flg);
static union   //子站地址0x52
{
    float prarm_float[6];//水肥机管道布置及PID设定
    u8 param_int[6][4];
} fertigation52;
/*
param_fertigation52[38] 记录0x52设备下发的整型数 定义如下
[0][1] 施肥次数
[2][3] 施水次数
[4][5] 已运行预滴
[6][7] 预滴时间
[8][9]  已设定施肥时间
[10][11] 每亩施灌量初值
[12][13]  已运行清洗
[14][15]  清洗时间
[16][17]  一键施肥灌溉
[18][19]  常规施灌开始
[20][21]  滴灌喷淋切换
[22][23]  重新设定
[24][25]   允许配肥1
[26][27]   允许配肥2
[28][29]   允许配肥3
[30][31]   允许施肥1
[32][33]   允许施肥2
[34][35]   允许施肥3
[36][37]   正在施肥标志

所有数据都是低位在前，高位在后
*/
static u8 param_fertigation52[38]; //增加 正在施肥标志 下发
static u8 reportfertigation52[3][16];
u8 fertigation52databuf[12] = { 0 }; //用于拷52下发的浮点数，转成2字节
static u8 fertigation_flg = 0;//用于区分常用网关还是水肥网关，根据不同的触摸屏程序，在触摸屏下发查询地址0x52时自动区分（置1）。

#define MAX_SFJ_I 12
#define SF_SlaveID_0 37
#define SF_SlaveID_1 38              //定义两个控制器子站用于控制阀门
static void SF_Para_Trans(void);

u8 SF_cmd_num[28][2] = { 0 }; //用于限制消息发送次数，当接收回复时将该下标处清0,二维数组表示两路控制器
void SF_Flow_Measu(void);
void SF_Flow_Trans(void);
u8 SF_Qeury_index=0;      //查询下发后收到的回复确认
u8 SF_Trans_flg=0;					//参数下发标志位，向两路控制器下发设定参数
u16 SF_collector_temp = 0;
static u8 SF_wgcollector_data_buff[16] = {0};
u8 SF_TD_param_num = 0;
#define SF_Para_Tran_time 500
extern u8 SF_USART3SendTCB[128];
u8 SF_Flow_Trans_Flg = 0;      //流量下发标志位，向两路控制器下发配肥流量以及总管流量
u16 SF_Flow_Total[6] = {0};  //取6个瞬时总管流量的平均值下发
u8 SF_Flow_Total_I = 0;
u8 SF_Flow_Total_flg = 0;
u8 SF_Lvbo641=12;
u8 SF_Lvbo642=12;
u8 SF_Lvbo646=12;


//#define LEDDelayTime 150
///*用做水肥机网关，从41#~64#控制器也作为9#~32#采集器上报6个参数，最后二个参数供水肥机触摸屏使用*/

//void LEDShowAirTemp(vs16 temperature); //温度为有符号数 -20℃

//void LEDShowAirHumi(vs16 humidity);      //湿度从0-100

//void LEDShowSoilTemp(vs16 temperature);

//void LEDShowSoilHumi(vs16 humidity);
//void LEDShow();     //大屏数据显示
//u8 LEDShowFlg = 0;  //表示当前显示的是哪组数据
//u8 LEDSwitchFlg = 0;  //切换标志
//vs16 LEDDataTemp = 0;   //数据缓存

//void LEDDelay(u8 flg,vs16 data);


////void LEDShowIllumin(u16 illuminance);

////void LEDShowRainDorp(u8 raindrop);
//u32 getLEDCRC(u8 *message,u16 length);
//static u8 test[100];

/*中航字符卡              factory_gateway_set[27] == 36	*/
void LedDisplay1(u8 region,vs16 airtemp,u16 airhumi,u16 illumi);  //字符分区1显示  空气温湿度+光照度
void LedDisplay2(u8 region,vs16 soiltemp,u16 soilhumi,u16 CO2density); //字符分区2显示   土壤温湿度+光照度

void LedDisplayERR1(u8 region);
void LedDisplayERR2(u8 region);
static const char AirTemp[] = "空气温度";
static const char AirHumi[] = "空气湿度";
static const char SoilTemp[] = "土壤温度";
static const char SoilHumi[] = "土壤湿度";     //长度10
static const char Illumi[] = "光照度";        //长度8
static const char CO2Density[10] = {0x00,0x43,0x00,0x4f,0x00,0x32,0xc5,0xa8,0xb6,0xc8};   //长度9 43 4f 32 c5 a8 b6 c8 "CO2浓度"
u8 LEDSwitchFlg = 0;  //切换标志
u8 LEDCollectorNum = 1;
vs8 ZHLEDShowAirTemp(vs16 temperature,u8 *message);
vs8 ZHLEDShowAirHumi(u16 Humidity,u8 *message);
vs8 ZHLEDShowIllumi(u16 Illuminance,u8 *message);
vs8 ZHLEDShowSoilHumi(u16 Humidity,u8 *message);
vs8 ZHLEDShowSoilTemp(vs16 temperature,u8 *message);
vs8 ZHLEDShowCO2Density(u16 CO2Density,u8 *message); //格式0-10000PPM
void LEDFunc(void);
u32 getLEDCRC(u8 *message,u16 length);
#define LEDCollecID1 0
#define LEDCollecID2 1
#define LEDCollecID3 2
#define LEDEDELAYTIME 250

/*****************************************************/

void Period_Events_Handle(u32 Events)
{
    u8 i;

    if (Events&SYS_INIT_EVT)
    {
        if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
        {
            //采集器、控制器的采集参数初始化开始
            for (i = 0; i <= 31; i++)
            {
                memcpy(Collectors[i], cj_init_value, 16);
            }
            for (i = 32; i <= 63; i++)
            {
                memcpy(Collectors[i], kzcj_init_value, 16);
            }
            memcpy(Collectors[64], cj_init_value, 16);
            //采集器、控制器的采集参数初始化结束
            memset(Controllers, 0x00, sizeof(Controllers));
            memset(set_param_flag, 0, sizeof(set_param_flag));
            memset(ZZ_Wireles_flag, 0, sizeof(ZZ_Wireles_flag));
            memset(ZZ_Wired_flag, 0, sizeof(ZZ_Wired_flag));
            memset(crtl_cmd_num, 0, sizeof(crtl_cmd_num));
            memset(crtl_cmd_numWX, 0, sizeof(crtl_cmd_numWX));
            memset(online_slaveID, 0, sizeof(online_slaveID));
            memset(online_slaveID_WX, 0, sizeof(online_slaveID_WX));
            memset(hand_auto_flg, 0, sizeof(hand_auto_flg));
            memset(wgcollector_data_buff, 0, sizeof(wgcollector_data_buff));
            memset(Adc_average, 0, sizeof(Adc_average));
            memset(TIM2_FrequencyPA0, 0, sizeof(TIM2_FrequencyPA0));
            memset(TIM2_FrequencyPC0, 0, sizeof(TIM2_FrequencyPC0));
            memset(TIM2_FrequencyPA1, 0, sizeof(TIM2_FrequencyPA1));
            memset(TIM5_pulsePA1, 0, sizeof(TIM5_pulsePA1));
            memset(report_last_rain, 0, sizeof(report_last_rain));

            for (i = 0; i < sizeof(absolute_pressure_zero); i++)
            {
                absolute_pressure_zero[i] = 9950;
            }
            for (i = 0; i < sizeof(TIM5_pulsePA1); i++)
            {
                TIM5_pulsePA1[i] = 9695;//0x9696us=6553.5mm,2字节液位最大数；9695us=1648.15mm
            }
            /*自测试子站数量，主要是控制器子站是否存在，如果存在则定期发送控制命令。采集器和控制器共64个子站检测参数轮流查询不受影响。
            	通过传感器参数查询命令，测出控制器是否存在，这样传感器参数刷新速度减慢，控制命令下发加快，3表示检测参数查询三次没有
            	收到回复，则认为该控制器不存在。参数查询还是发送，但控制命令不再下发。一但收到回复，该数组对应单元重新置3*/
            memset(init_cmd_numYX, 10, sizeof(init_cmd_numYX));
            memset(init_cmd_numWX, 4, sizeof(init_cmd_numWX));
            ZZ_temp_stateYX = 0;
            ZZ_temp_stateWX = 0;
        }
        RCC_ClearFlag();
        wg_init_readflash();
        initialRealSendBuff();
        //		Get_WX_Channel();//无线信道选择初始化
        if (close_433MHZ != 0)
        {
            SI4463_Init();
            Start_timerEx(WX_SENDZZ_EVT, 3777);
        }//触摸屏出厂设置应加入变量
//    RXENABLE3;//测试
        RXENABLE4;
        RXENABLE5;
        ZZ_Wired_flag[64] = factory_gateway_set[11];//有无网关采集功能，有=1
        cmd_flg = 0x03;
        send_flg = 0x00;
        OPEN_OUT24V;
        PCout(3) = 0;
        if (factory_gateway_set[1] == 4)
        {
            MD5_CTX md5;
            MD5Init(&md5);
            memset(md5_devId_devKey, 0, sizeof(md5_devId_devKey));
            memcpy(md5_devId_devKey, factory_gateway_set + 156, factory_gateway_set[155]);
            memcpy(md5_devId_devKey + factory_gateway_set[155], factory_gateway_set + 189, factory_gateway_set[188]);
            MD5Update(&md5, md5_devId_devKey, strlen((char *)md5_devId_devKey));
            MD5Final(&md5, md5_password);
            for (i = 0; i <= 15; i++)
            {
                byte_to_asc(md5_password[i], md5_password_ascii + 2 * i);
            }
        }
        Start_timerEx(WG_SENDZZ_EVT, 3000);
        Start_timerEx(NET_INIT_EVT, 3000);

        if(factory_gateway_set[27] == 36)       //中航字符卡 网关采集参数显示
        {
            Start_timerEx(LED_SHOW_EVT, 5000);
        }
        if (factory_gateway_set[11] != 0)
        {
            u16  logic_ture;
            if ((factory_gateway_set[12] >= 6 && factory_gateway_set[12] <= 7) || factory_gateway_set[12] == 10 ||factory_gateway_set[12] == 33|| factory_gateway_set[12] == 28)   //开关量及频率信号输入管脚PC0上拉20k电阻；通道0（光照）=10 cps120压力输入
            {
                LIGHT_DTA_R;//光照度数据管脚PC0上拉输入
                GPIOC->ODR = 1 << 0;//PC0高电平上拉	；否则无法输出高电平
                if (factory_gateway_set[12] == 7 || factory_gateway_set[12] == 33)   //PC0频率输入
                {
                    EXTI->PR = 1 << 0;  //清除LINE0上的中断标志位
                    EXTI->IMR |= 1 << 0;//不屏蔽line0上的中断，中断使能
                    EXIT0_Enable(ENABLE);
                }
            }
            logic_ture = factory_gateway_set[15] == 6 || factory_gateway_set[15] == 7 || factory_gateway_set[15] == 10 || factory_gateway_set[15] == 12;//PA1
            logic_ture = logic_ture || factory_gateway_set[15] == 23 || factory_gateway_set[15] == 33;
            if (logic_ture != 0)
                //开关量及频率信号输入管脚PA1上拉20k电阻；通道1（空气温湿度）=10 cps120压力输入；通道1（空气温湿度）=12 表压液位输入
            {
                DATA_INPUT;//	空气温湿度数据管脚PA1,上拉/下拉模式输入，由PxODR寄存器约定
                GPIOA->ODR = 1 << 1;//PA1高电平上拉	；否则无法输出高电平
                if (factory_gateway_set[15] == 7 || factory_gateway_set[15] == 23 || factory_gateway_set[15] == 33)   //PA1频率输入
                {
                    EXTI->PR = 1 << 1;  //清除LINE1上的中断标志位
                    EXTI->IMR |= 1 << 1;//不屏蔽line1上的中断，中断使能
                    EXTI1_Enable(ENABLE);
                }
            }

            if ((factory_gateway_set[24] >= 6 && factory_gateway_set[24] <= 7) || factory_gateway_set[24] == 10 ||  factory_gateway_set[24] == 33)   //开关量及频率信号输入管脚PA0上拉20k电阻
            {
                CPS_DTA_R;//	二氧化碳数据管脚PA0输入
                GPIOA->ODR = 1 << 0;//PA0高电平上拉	；否则无法输出高电平
                if (factory_gateway_set[24] == 7 || factory_gateway_set[24] == 33)   //PA0为频率输入
                {
                    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1); //清除TIM2上的TIM_IT_CC1中断标志位；规定PA0
                    TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);//不屏蔽TIM2上的TIM_IT_CC1的中断，中断使能
                }
            }
            logic_ture = factory_gateway_set[12] == 7 || factory_gateway_set[15] == 7 || factory_gateway_set[24] == 7;
            logic_ture = logic_ture || factory_gateway_set[15] == 23;
            logic_ture = logic_ture || factory_gateway_set[12] == 33 || factory_gateway_set[15] == 33 || factory_gateway_set[24] == 33;//水肥涡轮流量计频率输入
            if (logic_ture)   //有频率输入，打开TIM2及相应的中断
            {
                TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
                TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
                //					TIM_ITConfig(TIM2,TIM_IT_CC1,ENABLE);

                TIM_Cmd(TIM2, ENABLE);
            }
            if (factory_gateway_set[15] == 18)  	//脉冲宽度检测，单位：us
            {
                DATA_INPUT;//	空气温湿度数据管脚PA1,上拉/下拉模式输入，由PxODR寄存器约定
                GPIOA->ODR = 0 << 1;//PA1低电平下拉	；否则无法保证低电平
                TIM_ClearITPendingBit(TIM5, TIM_IT_CC2);
                TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
                TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
                TIM_ITConfig(TIM5, TIM_IT_CC2, ENABLE);
                TIM_Cmd(TIM5, ENABLE);
            }

            startadc();
            Start_timerEx(WGCOLLECTOR_DATA_EVT, 2500);

            Start_timerEx(SF_Flow_Measu_EVT, 2650);
            Start_timerEx(TX5_CMD_EVT, 2600);
            Start_timerEx(IO_XUNHUAN_CMD_EVT, 5000);
        }//网关有采集，启动采集进程
        ctrl_wx_j = 0;

        Start_timerEx(CYCLE_CMD_EVT, 65000);//检测各事件程序运行是否工作正常
    }
    YX_LED_TOGGLE;  //系统运行指示灯闪烁

    if (Events&RX4_DELAY_EVT)   //触摸屏通信;PC10--TX4,PC11--RX4,PB9--切换控制;AI02、BI02串口4
    {
        RxFlag4 = 1;
        RxReport4(RecLen4, ReceiveData4);
    }
		if(close_433MHZ!=0)//不使用433MHZ通信或433MHZ通信出问题，则close_433MHZ=0;
	{	
  if(Events&WX_CMD_EVT)//设置SI4463处于接收状态，等待接收中断
	{
		Clear_Buf(SI4463_RxBUFF,SI4463_RxLenth,0);
		SI4463_SET_PROPERTY_1( PKT_FIELD_1_LENGTH_12_8, 0x00 );
		SI4463_SET_PROPERTY_1( PKT_FIELD_1_LENGTH_7_0, 0x01 );
	 	SI4463_START_RX( SI4463_Channel, 0, PACKET_LENGTH, 8, 3, 3 );  
	}

	if(Events&WX_RECEIVE_EVT)//433MHZ无线接收
	{		
		SI4463_RxLenth = SI4463_READ_PACKET(SI4463_RxBUFF);			
		SI4463Receive(SI4463_RxLenth,SI4463_RxBUFF);		
	}

	if(Events&WX_SENDZZ_EVT)
	{ 
		WX_LED_TOGGLE;
		SI4463_SENDZZ(); //无线发送命令到子站
		Start_timerEx(WX_CMD_EVT,50);   
	}	

  if(Events&WX_SET_SLAVEPARAM_EVT) //网关无线向子站发送控制命令及查询命令；无线通信，SPI1
	{
		WX_LED_TOGGLE;
		set_slave_paramWX();
    Start_timerEx(WX_CMD_EVT,120); 		
	}
 }
	 if (Events&RX3_DELAY_EVT)   //网关与子站通信；PD8--TX,PD9--RX,PD10--切换控制；有线通信，AI01、BI01 串口3
    {
        //		if(RxReport3_len!=0x15){test_count++;}
        RxReport3(RxReport3_len, ReceiveData3);
    }
    if (Events&RX3_TIMEOUT_EVT)
    {
        RecDataBuffer3[0] = RecLen3 - 1;
        RecLen3 = 1;
        RxReport3_len = RecDataBuffer3[0];
        memcpy(ReceiveData3, RecDataBuffer3 + 1, RecDataBuffer3[0]);
        Start_timerEx(RX3_DELAY_EVT, 3);//2*RecDataBuffer3[0]
    }
    if (Events&CYCLE_CMD_EVT)
    {
        cycle_cmd();//定期循环执行，无任何条件判断其它模块运行是否正常;
    }
    if (Events&WG_SENDZZ_EVT)    //网关向子站发送控制命令及查询命令；有线通信，AI01、BI01 串口3
    {
        Send_slave_cmd();
    }

    if (Events&SF_Flow_Measu_EVT)    //测量配肥流量并启动发送程序水肥流量数据；有线通信，AI01、BI01 串口3
    {
        if(fertigation_flg == 1)
        {
            SF_Flow_Measu();
        }
				Start_timerEx(SF_Flow_Measu_EVT,1500);   //流量采集1.5s一次
    }
 
    if (Events&SF_Para_Trans_EVT)   //接收到水肥0x51 PID参数设定数据
    {
        if(fertigation_flg == 1)
        {
            SF_Para_Trans();    //向两路控制器发送触摸屏设定的参数
        }
    }

    if (Events&LED_SHOW_EVT)   //中航LED显示程序    // factory_gateway_set[29] == 36 长屏滚动； factory_gateway_set[28] 选择采集器个数
    {
        LEDFunc();
    }

   

    if (Events&SF_Flow_Trans_EVT)    //网关向子站发送水肥流量数据；有线通信，AI01、BI01 串口3
    {
        if(fertigation_flg == 1)
        {
            SF_Flow_Trans();
        }

    }
    if (Events&SET_SLAVEPARAM_EVT)    //网关有线向子站发送参数设定命令；有线通信，AI01、BI01 串口3
    {
        set_slave_param();
    }

    if (Events&NET_INIT_EVT)   //初始化模块链接平台
    {
        net_connect_count++;
        if (net_connect_count > 70)
        {
            __set_FAULTMASK(1);
            NVIC_SystemReset();
            while (1);
        }
        halt_module = 0;
        Stop_timerEx(WG_REPLY_EVT);
        Stop_timerEx(SEND_PLATFORM_EVT);
        Net_connect_init();
    }
    if (Events&SEND_PLATFORM_EVT)   //模块发送数据到平台
    {
        if ((halt_RxReport2 >= 10))
        {
            while (1);
        }
        if (factory_gateway_set[0] <= 2 && (factory_gateway_set[0] != 0)) halt_RxReport2++;
        send_platform();//模块发送数据到平台
    }
    if (Events&RX2_DELAY_EVT)   //模块接收平台数据
    {
        halt_RxReport2 = 0x00;//在SEND_PLATFORM_EVT事件中判断
        RxFlag2 = 1;
        RxReport2_step_flg = 0;//RxReport2_step_flg是为了减少CPU运行负担
        //RxReport2_step_flg=1表示链路函数处理过cdma_cmd_receive	;RxReport2_step_flg=2表示协议函数处理过cdma_tcp_receive或cdma_sdk_receive等
        RxReport2(RecLen2, ReceiveData2);
        if (RxReport2_step_flg == 0)   //SDK平台主动断开链路，发数据到平台，平台收不到数据则重新启动。
        {
            RxReport2_useless_count++;
            if (RxReport2_useless_count > 20)
            {
                while (1);
            }
        }
        else
        {
            RxReport2_useless_count = 0;
        }
    }
    if (Events&IO_XUNHUAN_CMD_EVT)   //超声波液位
    {
        IOxh_send_cmd(); //IO循环传感器采集数据
        Start_timerEx(IO_XUNHUAN_CMD_EVT, IO_CMD_PERIOD);
    }
    if (Events&RX1_DELAY_EVT)   //模块接收平台数据
    {
        if (factory_gateway_set[0] == 3)
        {
            RxFlag1 = 1;
            RxReport1_step_flg = 0;//RxReport2_step_flg是为了减少CPU运行负担
            //RxReport1_step_flg=1表示链路函数处理过cdma_cmd_receive	;RxReport2_step_flg=2表示协议函数处理过cdma_tcp_receive或cdma_sdk_receive等
            RxReport1(RecLen1, ReceiveData1);
        }
        if (factory_gateway_set[27] == 15)
        {
            RxReport1_csb_yw(RecLen1, ReceiveData1);//有线以太网不能用超声波液位
        }
        if (factory_gateway_set[27] == 31)
        {
            RxReport1_YANHUA_touch_screen(RecLen1, ReceiveData1);//研华触摸屏返回网关采集的8个检测参数
        }
    }

    if (Events&WG_REPLY_EVT)    //网关回复平台发送的控制命令及参数设定；
    {
        wg_reply_cmd();
    }
    if (Events&JM_PLATFORM_REPLY_EVT)    //快速恢复下发的控制命令；
    {
        jm_platform_reply();
    }

    
    if (Events&WGCOLLECTOR_DATA_EVT)
    {
        if(fertigation_flg != 1)  //等于1表示水肥机网关

        {
            wgcollector_data();    //传感器采集数据
        }

        Start_timerEx(WGCOLLECTOR_DATA_EVT, 1000);
    }
    if (Events&TX5_CMD_EVT)   //网关串口5采集发送命令;PC12--TX5,PD2--RX5,PD1--切换控制;AI03、BI03串口5
    {
        uart5_send_cmd(); //传感器采集数据
        Start_timerEx(TX5_CMD_EVT, TX5_CMD_PERIOD);
    }
    if (Events&RX5_DELAY_EVT)   //网关串口5接收传感器数据；PC12--TX5,PD2--RX5,PD1--切换控制；有线通信，AI03、BI03 串口5
    {
        RxFlag5 = 1;
        RxReport5(RxReport5_len, ReceiveData5);
    }
    if (Events&RX5_TIMEOUT_EVT)
    {
        RecDataBuffer5[0] = RecLen5 - 1;
        RecLen5 = 1;
        RxReport5_len = RecDataBuffer5[0];
        memcpy(ReceiveData5, RecDataBuffer5 + 1, RecDataBuffer5[0]);
        Start_timerEx(RX5_DELAY_EVT, 1);//2*RecDataBuffer5[0]
    }
}

void Scan_Events_Handle(void)
{
    IWDG_ReloadCounter();//独立看门狗喂狗
}

static u8  WriteMultipleRegister(u8 Slave_ID, u16 addr, u16 num, u8 *pData, u8 *temp)
//WriteMultipleRegister(Query_Index_set+33,0x0004,4,I_current_limit[Query_Index_set],senddata)
{
    temp[0] = Slave_ID;
    temp[1] = WRITE_MULTIPLE_REGISTER;
    temp[2] = (addr & 0xFF00) >> 8;
    temp[3] = addr & 0x00FF;
    temp[4] = (num & 0xFF00) >> 8;
    temp[5] = num & 0x00FF;
    temp[6] = (num * 2) & 0x00FF;
    memcpy(temp + 7, pData, num * 2);

    wm_CRC_Val = GetCRC16(temp, 7 + num * 2);

    temp[7 + num * 2] = wm_CRC_Val & 0x00FF;
    temp[8 + num * 2] = (wm_CRC_Val & 0xFF00) >> 8;
    return (9 + num * 2);
}

static u8  WriteSingleRegister(u8 Slave_ID, u16 addr, u8 *pData, u8 *temp)
//WriteSingleRegister(Query_Index_Controller+33,ctrl_j,Controllers[Query_Index_Controller]+ctrl_j*2,ReportData3);
{
    temp[0] = Slave_ID;
    temp[1] = WRITE_SINGLE_REGISTER;
    temp[2] = (addr & 0xFF00) >> 8;
    temp[3] = addr & 0x00FF;
    temp[4] = *pData;
    temp[5] = *(pData + 1);
    //	memcpy(temp+4,pData,2);
    wm_CRC_Val = GetCRC16(temp, 6);
    temp[6] = wm_CRC_Val & 0x00FF;
    temp[7] = (wm_CRC_Val & 0xFF00) >> 8;
    return (8);
}
static u8 ReadData(u8 Slave_ID, u8 function_code, u16 addr, u16 num, u8 *temp)
{
    //	u16 CRC_Val;

    temp[0] = Slave_ID;
    temp[1] = function_code;
    temp[2] = (addr & 0xFF00) >> 8;
    temp[3] = addr & 0x00FF;
    temp[4] = (num & 0xFF00) >> 8;
    temp[5] = num & 0x00FF;
    rd_CRC_Val = GetCRC16(temp, 6);
    temp[6] = rd_CRC_Val & 0x00FF;
    temp[7] = (rd_CRC_Val & 0xFF00) >> 8;
    return 8;
}

static void WriteDataToBuffer(u8 port, u8 *ptr, u8 start, u8 len)
{
    if (port == 3)
    {
        memcpy(USART3SendTCB, ptr + start, len);

        USART3BufferCNT = len;
        TXENABLE3;
        TxFlag3 = 1;
        USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
    }

    if (port == 5)
    {
        memcpy(UART5SendTCB, ptr + start, len);
        UART5BufferCNT = len;
        TXENABLE5;
        TxFlag5 = 1;
        USART_ITConfig(UART5, USART_IT_TXE, ENABLE);
    }
    if (port == 1)
    {
        memcpy(USART1SendTCB, ptr + start, len);

        USART1BufferCNT = len;
        //		TXENABLE1;
        TxFlag1 = 0;
        USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    }
    if (port == 2)
    {
        memcpy(USART2SendTCB, ptr + start, len);
        USART2BufferCNT = len;
        TxFlag2 = 0;

        USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
        //USART_SendData函数中，u32 i=0x004FFFFF;//防止全双工硬件通信时，硬件CTS不起作用造成等待状态反复进行watchdog(3S)不停起作用;2S左右，以看门狗不起作用为准
    }
    if (port == 4)
    {
        memcpy(UART4SendTCB, ptr + start, len);
        UART4BufferCNT = len;
        TXENABLE4;
        TxFlag4 = 0;
        USART_ITConfig(UART4, USART_IT_TXE, ENABLE);
    }
    return;
}

static u16 GetCRC16(u8 *Msg, u16 Len)
{
    gc_CRCHigh = 0xFF;//高CRC字节初始化
    gc_CRCLow = 0xFF;//低CRC字节初始化

    for (gc_i = 0; gc_i < Len; gc_i++)
    {
        gc_index = gc_CRCHigh ^ *Msg++;
        gc_CRCHigh = gc_CRCLow ^ crc_hi[gc_index];
        gc_CRCLow = crc_lo[gc_index];
    }
    return(gc_CRCLow << 8 | gc_CRCHigh);
}

void RxReport4(u8 len, u8 *pData)
{
    if (GetCRC16(pData, len) == 0)
    {
        u16 param_addr_start, param_length;
        switch (pData[0])   //根据触摸屏子站地址来进行下列程序运行
        {
        case 0xEA://管理员子站设定(控制器设定)
            if (pData[1] == 0x06)
            {
                //				 WriteDataToBuffer(5,pData,0,8);//响应，与接收到的完全一致;
                memcpy(UART4SendTCB, pData, 8);
                WriteDataToDMA_BufferTX4(8);
                slave_param_addr = pData[2] << 8 | pData[3];
                if (slave_param_addr <= 383)
                {
                    slave_KZparam_set[slave_param_addr / 12][(slave_param_addr % 12) * 2] = pData[4];//每一个采集器有18个参数，每个参数2个字节
                    slave_KZparam_set[slave_param_addr / 12][(slave_param_addr % 12) * 2 + 1] = pData[5];//每一个采集器有18个参数，每个参数2个字节
                    if (kzym_ID != kzym_old_ID)
                    {
                        first_xiabiao_I = slave_param_addr / 12;//计算每一个设定页面的第一个控制器地址，在有线参数设定函数中使用
                        firstwx_xiabiao_i = slave_param_addr / 12;//计算每一个设定页面的第一个控制器地址，在无线参数设定函数中使用
                        kzym_old_ID = kzym_ID;
                    }
                    slave_xiabiao_I = (slave_param_addr / 12) - first_xiabiao_I;//每个控制器有12组参数
                    slave_xiabiao_J = (slave_param_addr % 12) * 2;//每个参数2个字节，低字节在前
                    ctrlslave_param_set[slave_xiabiao_I][slave_xiabiao_J] = pData[4];
                    ctrlslave_param_set[slave_xiabiao_I][slave_xiabiao_J + 1] = pData[5];
                    ctrlslave_param_flg[slave_xiabiao_I] = 20;
                    ctrlslave_param_flgWX[slave_xiabiao_I] = 20;
                    set_finish_flg = 0;
                }
                if ((slave_param_addr == 384) && (pData[4] != 0x00))   //地址==385-1
                {
                    set_finish_flg = 1;//设定完成置0，返回到触摸屏
                    slave_set_flg = 1;//控制器子站有线设定
                    slave_set_flgWX = 1;//控制器子站无线设定
                    Stop_timerEx(WG_SENDZZ_EVT);
                    Stop_timerEx(WX_SENDZZ_EVT);
                    Start_timerEx(SET_SLAVEPARAM_EVT, 3000);
                    Start_timerEx(WX_SET_SLAVEPARAM_EVT, 3000);
                    Flash_Write(0x0807E800, (unsigned char *)slave_KZparam_set[0], 32 * 12 * 2);//32个控制器，每个控制器3组*4通道=12个变量，每个变量2个字节
                }
                if ((slave_param_addr == 385) && (pData[4] != 0x00))   //地址==386-1;用于区分同一设定页面不重新计算起始下标
                {
                    kzym_ID = pData[4];
                }
                break;
            }
            if (pData[1] == 0x03 && pData[2] == 0x01 && pData[3] == 0x80)   //地址==385-1
            {
                pData[2] = 2;
                pData[3] = set_finish_flg;
                pData[4] = 0;
                CRCReport4 = GetCRC16(pData, 5); //返回5个字节，地址（1），功能号（1），字节数（1）,0x00低值，0x00高值
                pData[5] = CRCReport4 & 0x00FF;      //CRC低位
                pData[6] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
//			    WriteDataToBuffer(5,pData,0,7);
                memcpy(UART4SendTCB, pData, 7);
                WriteDataToDMA_BufferTX4(7);
            }
            break;
        case 0xEB://触摸屏手自动切换设定查询命令
            if (pData[1] == 0x06 && pData[2] == 0)   //触摸屏写手自动状态标志：地址 命令06 变量地址（高2、低3）状态01 00 或 00 00（开关量状态输出4、5）
            {
                //					WriteDataToBuffer(5,pData,0,8);//响应，与接收到的完全一致;
                memcpy(UART4SendTCB, pData, 8);
                WriteDataToDMA_BufferTX4(8);
                hand_auto_flg[pData[3]][0] = pData[4];//变量地址低=0~72；
                hand_auto_flg[pData[3]][1] = pData[5];
                hand_auto_count[pData[3]] = 5;//收到新的手自动状态，向平台上报5次
                break;
            }
            if (pData[1] == 0x03 && pData[2] == 0)
            {
                ReportData4[0] = pData[0];
                ReportData4[1] = pData[1];
                ReportData4[2] = pData[5] * 2;//字节数=变量数*2
                memcpy(ReportData4 + 3, hand_auto_flg[pData[3]], pData[5] * 2);
                CRCReport4 = GetCRC16(ReportData4, 3 + pData[5] * 2);
                ReportData4[3 + pData[5] * 2] = CRCReport4 & 0x00FF;      //CRC低位
                ReportData4[3 + pData[5] * 2 + 1] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
//					WriteDataToBuffer(5,ReportData4,0,3+pData[5]*2+2);
                memcpy(UART4SendTCB, ReportData4, 3 + pData[5] * 2 + 2);
                WriteDataToDMA_BufferTX4(3 + pData[5] * 2 + 2);
                break;
            }
            break;
        case 0xEC://专业人员子站设定（采集器设定）
            if (pData[1] == 0x06)
            {
                //				 WriteDataToBuffer(5,pData,0,8);//响应，与接收到的完全一致;
                memcpy(UART4SendTCB, pData, 8);
                WriteDataToDMA_BufferTX4(8);
                slave_param_addr = pData[2] << 8 | pData[3];
                if (slave_param_addr <= 1151)
                {
                    slave_CJparam_set[slave_param_addr / 18][(slave_param_addr % 18) * 2] = pData[4];//每一个采集器有18个参数，每个参数2个字节
                    slave_CJparam_set[slave_param_addr / 18][(slave_param_addr % 18) * 2 + 1] = pData[5];//每一个采集器有18个参数，每个参数2个字节
                    if (cjym_ID != cjym_old_ID)
                    {
                        first_xiabiao_I = slave_param_addr / 18;//计算每一个设定页面的第一个采集器地址，在有线参数设定函数中使用
                        firstwx_xiabiao_i = slave_param_addr / 18;//计算每一个设定页面的第一个采集器地址，在无线参数设定函数中使用
                        cjym_old_ID = cjym_ID;
                    }

                    slave_xiabiao_I = (slave_param_addr / 18) - first_xiabiao_I;//每个采集器有18组参数
                    slave_xiabiao_J = (slave_param_addr % 18) * 2;//每个参数2个字节，低字节在前
                    cjqslave_param_set[slave_xiabiao_I][slave_xiabiao_J] = pData[4];
                    cjqslave_param_set[slave_xiabiao_I][slave_xiabiao_J + 1] = pData[5];
                    cjqslave_param_flg[slave_xiabiao_I] = 20;
                    cjqslave_param_flgWX[slave_xiabiao_I] = 20;
                    set_finish_flg = 0;
                }
                if ((slave_param_addr == 1152) && (pData[4] != 0x00))   //地址==1153-1
                {
                    u8 i;
                    set_finish_flg = 1;//设定对子站是否完成标志,未完成置1，完成置0，返回到触摸屏
                    slave_set_flg = 2;//采集有线设定
                    slave_set_flgWX = 2;//采集无线设定
                    Stop_timerEx(WG_SENDZZ_EVT);
                    Stop_timerEx(WX_SENDZZ_EVT);
                    Start_timerEx(SET_SLAVEPARAM_EVT, 3000);
                    Start_timerEx(WX_SET_SLAVEPARAM_EVT, 3000);
                    for (i = 0; i < 36; i = i + 2)
                    {
                        slave_CJparam_set[64][i] = factory_gateway_set[12 + i / 2];
                        slave_CJparam_set[64][i + 1] = 0;
                    }
                    Flash_Write(0x0807D800, (unsigned char *)slave_CJparam_set[0], 32 * 18 * 2);//前32个采集器保存
                    Flash_Write(0x0807E000, (unsigned char *)slave_CJparam_set[32], 33 * 18 * 2);//后33个控制器和网关采集参数保存
                }
                if ((slave_param_addr == 1153) && (pData[4] != 0x00))   //地址==1154-1;用于区分同一设定页面不重新计算起始下标
                {
                    cjym_ID = pData[4];
                }
                break;
            }
            if (pData[1] == 0x03 && pData[2] == 0x04 && pData[3] == 0x80)   //地址==1153-1
            {
                pData[2] = 2;
                pData[3] = set_finish_flg;
                pData[4] = 0;
                CRCReport4 = GetCRC16(pData, 5); //返回5个字节，地址（1），功能号（1），字节数（1）,0x00低值，0x00高值
                pData[5] = CRCReport4 & 0x00FF;      //CRC低位
                pData[6] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
//			    WriteDataToBuffer(5,pData,0,7);
                memcpy(UART4SendTCB, pData, 7);
                WriteDataToDMA_BufferTX4(7);
            }
            break;
        case 0xEE://公式参数设定
            param_addr_start = pData[2] << 8 | pData[3];
            if (pData[1] == 0x10 && param_addr_start < 2080)
            {
                memcpy(zero_rang.array_k_b[param_addr_start / 2], pData + 7, pData[6]);
                CRCReport4 = GetCRC16(pData, 6); //返回前6个字节，地址（1），功能号（1），变量起始地址（2），变量个数（2）
                pData[6] = CRCReport4 & 0x00FF;      //CRC低位
                pData[7] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
//			    WriteDataToBuffer(5,pData,0,8);
                memcpy(UART4SendTCB, pData, 8);
                WriteDataToDMA_BufferTX4(8);
            }
            if (param_addr_start == 2080)   //触摸屏的《确定设置EE》
            {
                if (pData[1] == 0x06 && pData[4] != 0x00)   //《确定设置EE》=1;公式参数设定，共65*16*4=4160个字节，分3个bank写入
                {
                    //						WriteDataToBuffer(5,pData,0,8);//响应，与接收到的完全一致;
                    memcpy(UART4SendTCB, pData, 8);
                    WriteDataToDMA_BufferTX4(8);
                    Flash_Write(0x0807B800, (unsigned char *)zero_rang.array_k_b, 384 * 4);//调试时注意不知一次是否能够写入大于2kbyte;公式参数设置共有780*4字节分二次写入
                    Flash_Write(0x0807C000, (unsigned char *)zero_rang.array_k_b + 384 * 4, 384 * 4);//调试时注意不知一次是否能够写入大于2kbyte
                    Flash_Write(0x0807C800, (unsigned char *)zero_rang.array_k_b + 768 * 4, 272 * 4);//0x0807C800~0x0807CC3F;其它数据可以从0x0807CC40开始保存
                    set_finish_flgEE = 0;
                    //						wg_init_readflash();
                    //触摸屏下发公式设定参数需要写入flash0x0804 0000，从256k开始写入，0x0804加1，则增加64k;
                    //特别注意Flash_Write函数定义的写入的最大字节数为512，大于则修改Flash_Write函数中的定义，否则死机（进入HardFault_Handler函数）
                }
                if (pData[1] == 0x06 && pData[4] == 0x00)   //《确定设置EE》=0
                {
                    //						WriteDataToBuffer(5,pData,0,8);//响应，与接收到的完全一致;
                    memcpy(UART4SendTCB, pData, 8);
                    WriteDataToDMA_BufferTX4(8);
                }
                if (pData[1] == 0x03)   //触摸屏查询《确定设置EE》，返回0；地址（1），功能号（1），字节数（1）,0x00低值，0x00高值
                {
                    pData[2] = 2;
                    pData[3] = set_finish_flgEE;
                    pData[4] = 0;
                    CRCReport4 = GetCRC16(pData, 5); //返回5个字节，地址（1），功能号（1），字节数（1）,0x00低值，0x00高值
                    pData[5] = CRCReport4 & 0x00FF;      //CRC低位
                    pData[6] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
//			      WriteDataToBuffer(5,pData,0,7);
                    memcpy(UART4SendTCB, pData, 7);
                    WriteDataToDMA_BufferTX4(7);
                }
            }
            break;
        case 0xEF://出厂网关设定
            if (pData[1] == 0x06 && pData[3] <= 0x1D)   //0x1D=29，前30个设定参数
            {
                memcpy(UART4SendTCB, pData, 8);
                WriteDataToDMA_BufferTX4(8);
                if (pData[3] == 15 && pData[4] == 18)   //超声波脉宽检测，PA1拉高为不停的中断
                {
                    DATA_INPUT;//	空气温湿度数据管脚PA1,上拉/下拉模式输入，由PxODR寄存器约定
                    GPIOA->ODR = 0 << 1;//PA1低电平下拉	；否则无法保证低电平
                    TIM_ClearITPendingBit(TIM5, TIM_IT_CC2);
                    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
                    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
                    TIM_ITConfig(TIM5, TIM_IT_CC2, ENABLE);
                    TIM_Cmd(TIM5, ENABLE);
                }
                else
                {
                    TIM_ClearITPendingBit(TIM5, TIM_IT_CC2);
                    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
                    TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
                    TIM_ITConfig(TIM5, TIM_IT_CC2, DISABLE);
                    TIM_Cmd(TIM5, DISABLE);
                    DATA_INPUT;//	空气温湿度数据管脚PA1,上拉/下拉模式输入，由PxODR寄存器约定
                    GPIOA->ODR = 1 << 1;//PA1高电平上拉	；否则无法保证高电平
                }
                factory_gateway_set[pData[3]] = pData[4];//只取低字节数，高字节数据不用
            }

            if (pData[1] == 0x06 && pData[3] == 124)   //无线设定变量的触摸屏变量地址-1
            {
                close_433MHZ = pData[4];
                //					WriteDataToBuffer(5,pData,0,8);//响应，与接收到的完全一致;
                memcpy(UART4SendTCB, pData, 8);
                WriteDataToDMA_BufferTX4(8);
            }

            if (pData[1] == 0x10)   //触摸屏写功能号0x10只能是字符串，变量都用0x06
            {
                switch (pData[3])
                {
                case 0x1F://IP地址端口,由触摸屏发送过来的命令决定；21个字节+1个字节保存实际字节数；数组下标：30~51；触摸屏变量地址：0x1F~0x29
                    factory_gateway_set[30] = pData[6] - 8;//编程加入本次设定的实际字节数（触摸屏发过来的）
                    memcpy(factory_gateway_set + 31, pData + 15, pData[6] - 8);
                    break;
                case 0x2A://客户端ID,最长字节数为30；数组下标：52~82；触摸屏变量地址：0x2A~0x38
                    factory_gateway_set[52] = pData[6] - 8;//52=30+21+1
                    memcpy(factory_gateway_set + 53, pData + 15, pData[6] - 8);
                    break;
                case 0x39://订阅主题,最长字节数为40；数组下标：83~123；触摸屏变量地址：0x39~0x4C
                    factory_gateway_set[83] = pData[6] - 8;//83=52+30+1
                    memcpy(factory_gateway_set + 84, pData + 15, pData[6] - 8);
                    break;
                case 0x4D://推送主题,最长字节数为30；数组下标：124~154；触摸屏变量地址：0x4D~0x5B
                    factory_gateway_set[124] = pData[6] - 8;
                    memcpy(factory_gateway_set + 125, pData + 15, pData[6] - 8);
                    break;
                case 0x5C://用户名,最长字节数为32；数组下标：155~187；触摸屏变量地址：0x5C~0x6C
                    factory_gateway_set[155] = pData[6] - 8;
                    memcpy(factory_gateway_set + 156, pData + 15, pData[6] - 8);
                    break;
                case 0x6C://用户名密码,最长字节数为32；数组下标：188~220；触摸屏变量地址：0x6D~0x7D
                    factory_gateway_set[188] = pData[6] - 8;
                    memcpy(factory_gateway_set + 189, pData + 15, pData[6] - 8);
                    break;
                default:
                    break;
                }
                CRCReport4 = GetCRC16(pData, 6); //返回前6个字节，地址（1），功能号（1），变量起始地址（2），变量个数（2）
                pData[6] = CRCReport4 & 0x00FF;      //CRC低位
                pData[7] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
//			   WriteDataToBuffer(5,pData,0,8);
                memcpy(UART4SendTCB, pData, 8);
                WriteDataToDMA_BufferTX4(8);
                //				 set_finish_flg=1;
            }

            if (pData[3] == 0x1E)   //触摸屏的《确定设置EF》
            {
                if (pData[1] == 0x06 && pData[4] != 0x00)   //《确定设置EF》=1
                {
                    //						WriteDataToBuffer(5,pData,0,8);//响应，与接收到的完全一致;
                    memcpy(UART4SendTCB, pData, 8);
                    WriteDataToDMA_BufferTX4(8);
                    memcpy(factory_gateway_set + 221, main_call, 11);
                    memcpy(factory_gateway_set + 232, voice_call, 11);
                    memcpy(factory_gateway_set + 243, third_call, 11);
                    factory_gateway_set[254] = close_433MHZ;
                    Flash_Write(0x0807B000, factory_gateway_set, 255);
                    set_finish_flgEF = 0;
                    //						wg_init_readflash();
                    //触摸屏下发出厂网关设定参数需要写入flash0x0807 0000，从256k开始写入，0x0804加1，则增加64k;
                    //特别注意Flash_Write函数定义的写入的最大字节数为512，大于则修改Flash_Write函数中的定义，否则死机（进入HardFault_Handler函数）
                }
                if (pData[1] == 0x06 && pData[4] == 0x00)   //《确定设置EF》=0
                {
                    //						WriteDataToBuffer(5,pData,0,8);//响应，与接收到的完全一致;
                    memcpy(UART4SendTCB, pData, 8);
                    WriteDataToDMA_BufferTX4(8);
                }
                if (pData[1] == 0x03)   //触摸屏查询《确定设置EF》，返回0；地址（1），功能号（1），字节数（1）,0x00低值，0x00高值
                {
                    pData[2] = 2;
                    pData[3] = set_finish_flgEF;
                    pData[4] = 0;
                    CRCReport4 = GetCRC16(pData, 5); //返回5个字节，地址（1），功能号（1），字节数（1）,0x00低值，0x00高值
                    pData[5] = CRCReport4 & 0x00FF;      //CRC低位
                    pData[6] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
//			      WriteDataToBuffer(5,pData,0,7);
                    memcpy(UART4SendTCB, pData, 7);
                    WriteDataToDMA_BufferTX4(7);
                }
            }
            break;//出厂网关设定结束

        case 0xF0://专业人员子站地址设定
            if (pData[1] == 0x06)
            {
                //					WriteDataToBuffer(5,pData,0,8);//响应，与接收到的完全一致;
                memcpy(UART4SendTCB, pData, 8);
                WriteDataToDMA_BufferTX4(8);

                /*pData[3]=0 老地址；=1 新地址；=2 老信道；=3 新信道；=4 子站仿采集器数量；=5 子站仿控制器数量；=6 仿采集器子站始地址；
                				=7 仿控制器子站始地址；=8 仿采变量起址；=9 仿采变量数量；=10 仿控变量起址；=11 仿控变量数量；
                			  =12仿控电流起址； =13 仿控电流数量；=14 采集下位机子站地址；=15 控制下位机子站地址；
                				=16 确定设置F0；特别说明：采集器作为主站，利用串口3管理其下位机，采集下位机子站地址与控制下位机子站地址通常设置相同；
                				也可以分别设定，增加灵活性*/

                if (pData[3] <= 0x0F)
                {
                    set_slaveID_channel[pData[3]] = pData[4];//地址 功能号 变量地址高 变量地址低 数据低 数据高
                    slaveID_channel_flg = 20;//向子站发送站地址及无线信道设定命令的次数
                    slaveID_channel_flgWX = 20;//向子站发送站地址及无线信道设定命令的次数
//						set_finish_flg=0;
                }
                if (pData[3] == 0x10 && pData[4] != 0x00)   //地址pData[3]==16;设定对子站是否完成标志,未完成置1，完成置0，返回到触摸屏
                {
                    set_finish_flg = 1;//设定对子站是否完成标志,未完成置1，完成置0，返回到触摸屏
                    slave_set_flg = 3;//子站地址及无线信道的有线设定标志
                    slave_set_flgWX = 3;//子站地址及无线信道的无线设定标志

                    Stop_timerEx(WG_SENDZZ_EVT);
                    Stop_timerEx(WX_SENDZZ_EVT);
                    Start_timerEx(SET_SLAVEPARAM_EVT, 100);
                    Start_timerEx(WX_SET_SLAVEPARAM_EVT, 150);
                }
            }
            if (pData[1] == 0x03 && pData[2] == 0x00 && pData[3] == 0x10)   //地址==16;设定对子站是否完成标志,未完成置1，完成置0，返回到触摸屏
            {
                pData[2] = 2;
                pData[3] = set_finish_flg;
                pData[4] = 0;
                CRCReport4 = GetCRC16(pData, 5); //返回5个字节，地址（1），功能号（1），字节数（1）,0x00低值，0x00高值
                pData[5] = CRCReport4 & 0x00FF;      //CRC低位
                pData[6] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
//			    WriteDataToBuffer(5,pData,0,7);
                memcpy(UART4SendTCB, pData, 7);
                WriteDataToDMA_BufferTX4(7);
            }
            if (pData[1] == 0x04)   //在线子站地址返回到触摸屏
            {
                u8 i, i_offset;
                i_offset = 0;
                ReportData4[0] = pData[0];
                ReportData4[1] = pData[1];
                ReportData4[2] = pData[5] * 2;//字节数=变量数*2
                if (pData[3] <= 71)
                {
                    if (pData[3] >= 40)
                    {
                        i_offset = 32;   //64个子站分2次读取，第一次读取地址为0，第二次读取地址为40
                    }
                    for (i = 0; i < pData[5]; i++)
                    {
                        ReportData4[2 * i + 3] = online_slaveID[i + i_offset];
                        ReportData4[2 * i + 4] = 0;
                    }
                }
                if (pData[3] >= 77)
                {
                    if (pData[3] >= 110)
                    {
                        i_offset = 32;   //64个无线子站分2次读取，第一次读取地址为77~108，第二次读取地址为110~141
                    }
                    for (i = 0; i < pData[5]; i++)
                    {
                        ReportData4[2 * i + 3] = online_slaveID_WX[i + i_offset];
                        ReportData4[2 * i + 4] = 0;
                    }
                }
                CRCReport4 = GetCRC16(ReportData4, 3 + pData[5] * 2);
                ReportData4[3 + pData[5] * 2] = CRCReport4 & 0x00FF;      //CRC低位
                ReportData4[3 + pData[5] * 2 + 1] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
//					WriteDataToBuffer(5,ReportData4,0,3+pData[5]*2+2);
                memcpy(UART4SendTCB, ReportData4, 3 + pData[5] * 2 + 2);
                WriteDataToDMA_BufferTX4(3 + pData[5] * 2 + 2);
            }
            break;//专业人员子站地址设定结束
        case 0xF1://读回公式参数
            Flash_Read(0x0807B800, (unsigned char *)zero_rang.array_k_b, 384 * 4);
            Flash_Read(0x0807C000, (unsigned char *)zero_rang.array_k_b + 384 * 4, 384 * 4);
            Flash_Read(0x0807C800, (unsigned char *)zero_rang.array_k_b + 768 * 4, 272 * 4);
            param_addr_start = pData[2] << 8 | pData[3];
            param_length = pData[4] << 8 | pData[5];
            if (pData[1] == 0x03 && param_addr_start < 2121 && param_length>1)   //读回公式参数,三个采集器一组16*3个浮点数，共计算为96个变量，字节数192个
            {
                u8 i;//21*96(60H)+1*64(40H)*2=4160个字节
                ReportData4[0] = pData[0];
                ReportData4[1] = pData[1];
                ReportData4[2] = pData[5] * 2;//字节数=变量数*2
                memcpy(ReportData4 + 3, (unsigned char *)zero_rang.array_k_b + (param_addr_start / 96) * 96 * 2, param_length * 2);
                CRCReport4 = GetCRC16(ReportData4, 3 + pData[5] * 2);
                ReportData4[3 + pData[5] * 2] = CRCReport4 & 0x00FF;      //CRC低位
                ReportData4[3 + pData[5] * 2 + 1] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
                memcpy(UART4SendTCB, ReportData4, 5 + param_length * 2);
                WriteDataToDMA_BufferTX4(5 + param_length * 2);
                formula_param_finish[param_addr_start / 96] = 1;
                formula_read_finishF1 = 1;
                for (i = 0; i < 22; i++)
                {
                    formula_read_finishF1 = formula_read_finishF1 & formula_param_finish[i];
                }
                if (formula_read_finishF1 == 1)
                {
                    memset(formula_param_finish, 0x00, sizeof(formula_param_finish));
                    set_finish_flgF1 = 0;
                }
            }
            if (param_addr_start == 2124)   //触摸屏的《确定设置F1》
            {
                if (pData[1] == 0x06 && pData[4] != 0x00)   //《确定设置F1》=1
                {
                    //						WriteDataToBuffer(5,pData,0,8);//响应，与接收到的完全一致;
                    memcpy(UART4SendTCB, pData, 8);
                    WriteDataToDMA_BufferTX4(8);
                    set_finish_flgF1 = 1;
                    //触摸屏下发公式设定参数需要写入flash0x0804 0000，从256k开始写入，0x0804加1，则增加64k;
                    //特别注意Flash_Write函数定义的写入的最大字节数为512，大于则修改Flash_Write函数中的定义，否则死机（进入HardFault_Handler函数）
                }
                if (pData[1] == 0x06 && pData[4] == 0x00)   //《确定设置F1》=0
                {
                    //						WriteDataToBuffer(5,pData,0,8);//响应，与接收到的完全一致;
                    memcpy(UART4SendTCB, pData, 8);
                    WriteDataToDMA_BufferTX4(8);
                }
                if (pData[1] == 0x03)   //触摸屏查询《确定设置F1》，完成返回0；地址（1），功能号（1），字节数（1）,0x00低值，0x00高值
                {
                    pData[2] = 2;
                    pData[3] = set_finish_flgF1;
                    pData[4] = 0;
                    CRCReport4 = GetCRC16(pData, 5); //返回5个字节，地址（1），功能号（1），字节数（1）,0x00低值，0x00高值
                    pData[5] = CRCReport4 & 0x00FF;      //CRC低位
                    pData[6] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
//			      WriteDataToBuffer(5,pData,0,7);
                    memcpy(UART4SendTCB, pData, 7);
                    WriteDataToDMA_BufferTX4(7);
                }
            }
            break;//读回公式参数结束
        case 0xF2://读回采集参数
            Flash_Read(0x0807D800, (unsigned char *)slave_CJparam_set[0], 32 * 18 * 2);//前32个采集器采集参数保存
            Flash_Read(0x0807E000, (unsigned char *)slave_CJparam_set[32], 33 * 18 * 2);//后33个控制器采集和网关采集参数保存
            param_addr_start = pData[2] << 8 | pData[3];
            param_length = pData[4] << 8 | pData[5];
            if (pData[1] == 0x04 && param_addr_start < 1180 && param_length>1)   //读回采集参数,6个采集器参数一组18*6个变量，共计算为108个变量，字节数216个
            {
                u8 i;
                ReportData4[0] = pData[0];
                ReportData4[1] = pData[1];
                ReportData4[2] = pData[5] * 2;//字节数=变量数*2
                memcpy(ReportData4 + 3, (unsigned char *)slave_CJparam_set + (param_addr_start / 108) * 108 * 2, param_length * 2);
                CRCReport4 = GetCRC16(ReportData4, 3 + pData[5] * 2);
                ReportData4[3 + pData[5] * 2] = CRCReport4 & 0x00FF;      //CRC低位
                ReportData4[3 + pData[5] * 2 + 1] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
                memcpy(UART4SendTCB, ReportData4, 5 + param_length * 2);
                WriteDataToDMA_BufferTX4(5 + param_length * 2);
                collector_param_finish[param_addr_start / 108] = 1;
                collector_read_finishF2 = 1;
                for (i = 0; i < 11; i++)
                {
                    collector_read_finishF2 = collector_read_finishF2 & collector_param_finish[i];
                }
                if (collector_read_finishF2 == 1)
                {
                    memset(collector_param_finish, 0x00, sizeof(collector_param_finish));
                    set_finish_flgF2 = 0;
                }
            }
            if (param_addr_start == 1181)   //触摸屏的《读采集参数完成F2》；变量地址-1
            {
                if (pData[1] == 0x06 && pData[4] != 0x00)   //《读采集参数完成F2》=1
                {
                    memcpy(UART4SendTCB, pData, 8);
                    WriteDataToDMA_BufferTX4(8);
                    set_finish_flgF2 = 1;
                }
                if (pData[1] == 0x06 && pData[4] == 0x00)   //《读采集参数完成F2》=0
                {
                    memcpy(UART4SendTCB, pData, 8);
                    WriteDataToDMA_BufferTX4(8);
                }
                if (pData[1] == 0x03)   //触摸屏查询《读采集参数完成F2》，完成返回0；地址（1），功能号（1），字节数（1）,0x00低值，0x00高值
                {
                    pData[2] = 2;
                    pData[3] = set_finish_flgF2;
                    pData[4] = 0;
                    CRCReport4 = GetCRC16(pData, 5); //返回5个字节，地址（1），功能号（1），字节数（1）,0x00低值，0x00高值
                    pData[5] = CRCReport4 & 0x00FF;      //CRC低位
                    pData[6] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
                    memcpy(UART4SendTCB, pData, 7);
                    WriteDataToDMA_BufferTX4(7);
                }
            }
            break;//读回采集参数结束

        case 0xF3://读回控制参数
            Flash_Read(0x0807E800, (unsigned char *)slave_KZparam_set[0], 32 * 12 * 2);//读回32个控制器参数
//			  memcpy(test,pData,8);
            param_addr_start = pData[2] << 8 | pData[3];
            param_length = pData[4] << 8 | pData[5];
            if (pData[1] == 0x04 && param_addr_start < 387 && param_length>1)   //读回控制参数,6个采集器参数一组18*6个变量，共计算为108个变量，字节数216个
            {
                u8 i;
                ReportData4[0] = pData[0];
                ReportData4[1] = pData[1];
                ReportData4[2] = pData[5] * 2;//字节数=变量数*2
                memcpy(ReportData4 + 3, (unsigned char *)slave_KZparam_set + (param_addr_start / 96) * 96 * 2, param_length * 2);
                CRCReport4 = GetCRC16(ReportData4, 3 + pData[5] * 2);
                ReportData4[3 + pData[5] * 2] = CRCReport4 & 0x00FF;      //CRC低位
                ReportData4[3 + pData[5] * 2 + 1] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
                memcpy(UART4SendTCB, ReportData4, 5 + param_length * 2);
                WriteDataToDMA_BufferTX4(5 + param_length * 2);
                ctrl_param_finish[param_addr_start / 96] = 1;
                ctrl_read_finishF3 = 1;
                for (i = 0; i < 4; i++)
                {
                    ctrl_read_finishF3 = ctrl_read_finishF3 & ctrl_param_finish[i];
                }
                if (ctrl_read_finishF3 == 1)
                {
                    memset(ctrl_param_finish, 0x00, sizeof(ctrl_param_finish));
                    set_finish_flgF3 = 0;
                }
            }
            if (param_addr_start == 388)   //触摸屏的《读采集参数完成F2》；变量地址389-1
            {
                if (pData[1] == 0x06 && pData[4] != 0x00)   //《读控制参数完成F3》=1
                {
                    memcpy(UART4SendTCB, pData, 8);
                    WriteDataToDMA_BufferTX4(8);
                    set_finish_flgF3 = 1;
                }
                if (pData[1] == 0x06 && pData[4] == 0x00)   //《读控制参数完成F3》=0
                {
                    memcpy(UART4SendTCB, pData, 8);
                    WriteDataToDMA_BufferTX4(8);
                }
                if (pData[1] == 0x03)   //触摸屏查询《读控制参数完成F3》，完成返回0；地址（1），功能号（1），字节数（1）,0x00低值，0x00高值
                {
                    pData[2] = 2;
                    pData[3] = set_finish_flgF3;
                    pData[4] = 0;
                    CRCReport4 = GetCRC16(pData, 5); //返回5个字节，地址（1），功能号（1），字节数（1）,0x00低值，0x00高值
                    pData[5] = CRCReport4 & 0x00FF;      //CRC低位
                    pData[6] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
                    memcpy(UART4SendTCB, pData, 7);
                    WriteDataToDMA_BufferTX4(7);
                }
            }
            break;//读回控制参数结束
        /*施灌总管压力设定091C、配肥1浓度011、配肥2浓度012、配肥3浓度016、PIKd1、PITi1、PIK0X1、PIK1X1、PIK2X1、PIT0X1、
        	PIT1X1、PIT2X1、PIKd2、PITi2、PIK0X2、PIK1X2、PIK2X2、PIT0X2、PIT1X2、PIT2X2、PIKd3、PITi3、PIK0X3、PIK1X3、PIK2X3、PIT0X3、PIT1X3、PIT2X3
        */
        case 0x51://水肥PID设定开始

            if (pData[1] == 0x10)   //触摸屏写功能号0x10只能是字符串或浮点数
            {
                SF_cmd_num[pData[3] / 2][0] = 30;     //允许向子站发送30次指令，每发一次减1，收到回复清0
                SF_cmd_num[pData[3] / 2][1] = 30;     //允许向子站发送30次指令，每发一次减1，收到回复清0
                memcpy(fertigation51.set_int[pData[3] / 2], pData + 7, pData[6]);
                //               memcpy(USART3SendTCB,pData,13);
                //              WriteDataToDMA_BufferTX3(13);
                CRCReport4 = GetCRC16(pData, 6); //返回前6个字节，地址（1），功能号（1），变量起始地址（2），变量个数（2）
                pData[6] = CRCReport4 & 0x00FF;      //CRC低位
                pData[7] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
                memcpy(UART4SendTCB, pData, 8);
                WriteDataToDMA_BufferTX4(8);
                set_fertigation51_finish[pData[3] / 2] = 1;
                Start_timerEx(SF_Para_Trans_EVT,3200);
            }
            if (pData[3] == 56)   //触摸屏的《读水肥机设定完成0x51》；变量地址57-1
            {
                if (pData[1] == 0x06 && pData[4] != 0x00)   //《读水肥机设定完成0x51》=1
                {
                    u8 i;
                    memcpy(UART4SendTCB, ReceiveData4, 8);
                    WriteDataToDMA_BufferTX4(8);
                    read_fertigation_finish51 = 1;
                    for (i = 0; i < 28; i++)
                    {
                        read_fertigation_finish51 = read_fertigation_finish51 & set_fertigation51_finish[i];
                    }
                    if (read_fertigation_finish51 == 1)
                    {
                        //						 	 memset(set_fertigation51_finish,0x00,sizeof(set_fertigation51_finish));
                        read_fertigation_finish51 = 0;
                    }
                    //                    memcpy(USART3SendTCB,pData,8);
                    //                    WriteDataToDMA_BufferTX3(8);
                }
                if (pData[1] == 0x06 && pData[4] == 0x00)   //《读水肥机设定完成0x51》=0
                {
                    memcpy(UART4SendTCB, pData, 8);
                    WriteDataToDMA_BufferTX4(8);

                    //                    memcpy(USART3SendTCB,pData,8);
                    //                    WriteDataToDMA_BufferTX3(8);
                }
                if (pData[1] == 0x03)   //触摸屏查询《读水肥机设定完成0x51》，完成返回0；地址（1），功能号（1），字节数（1）,0x00低值，0x00高值
                {
                    //                    memcpy(USART3SendTCB,pData,8);
                    //                    WriteDataToDMA_BufferTX3(8);
                    pData[2] = 2;
                    pData[3] = read_fertigation_finish51;
                    pData[4] = 0;
                    CRCReport4 = GetCRC16(pData, 5); //返回5个字节，地址（1），功能号（1），字节数（1）,0x00低值，0x00高值
                    pData[5] = CRCReport4 & 0x00FF;      //CRC低位
                    pData[6] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
                    memcpy(UART4SendTCB, pData, 7);
                    WriteDataToDMA_BufferTX4(7);
                }
            }
            break;//水肥PID设定结束

        case 0x52://水肥开始
            fertigation_flg = 1;//表示该网关是水肥机；在查询9#~32#采集器时只查询6个参数，最后二个供触摸屏使用


            if (pData[1] == 0x10)   //触摸屏写功能号0x10只能是字符串或浮点数
            {
                memcpy(fertigation52.param_int[pData[3] / 2], pData + 7, pData[6]);
                fertigation52databuf[pData[3]] = ((u16)(fertigation52.prarm_float[pData[3] / 2] * 100)) >> 8;//水肥浮点数k=100，b=0
                fertigation52databuf[pData[3] + 1] = ((u16)(fertigation52.prarm_float[pData[3] / 2] * 100)) & 0x00FF;
                //                memcpy(USART3SendTCB,pData,13);
                //                WriteDataToDMA_BufferTX3(13);
                CRCReport4 = GetCRC16(pData, 6); //返回前6个字节，地址（1），功能号（1），变量起始地址（2），变量个数（2）
                pData[6] = CRCReport4 & 0x00FF;      //CRC低位
                pData[7] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
                memcpy(UART4SendTCB, pData, 8);
                WriteDataToDMA_BufferTX4(8);
                break;
            }
            if (pData[1] == 0x06 && pData[3] >= 0x0C)   //触摸屏写功能号0x06写单个寄存器，整型值上报参数
            {
                memcpy(UART4SendTCB, pData, 8);
                WriteDataToDMA_BufferTX4(8);
                param_fertigation52[(pData[3] - 0x0C) * 2] = pData[4];  //每个参数分两个字节存储
                param_fertigation52[(pData[3] - 0x0C) * 2 + 1] = pData[5];
                //                memcpy(USART3SendTCB,ReceiveData4,8);
                //                WriteDataToDMA_BufferTX3(8);
                break;
            }
            if (pData[1] == 0x03 && pData[3] >= 0x0C)   //触摸屏写功能号0x10只能是字符串或浮点数
            {
                //                memcpy(USART3SendTCB,ReceiveData4,8);
                //                WriteDataToDMA_BufferTX3(8);
                ReportData4[0] = pData[0];
                ReportData4[1] = pData[1];
                ReportData4[2] = pData[5] * 2;//字节数=变量数*2
                memcpy(ReportData4 + 3, param_fertigation52 + (pData[3] - 0x0C) * 2, pData[5] * 2);
                CRCReport4 = GetCRC16(ReportData4, 3 + pData[5] * 2);
                ReportData4[3 + pData[5] * 2] = CRCReport4 & 0x00FF;      //CRC低位
                ReportData4[3 + pData[5] * 2 + 1] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
                memcpy(UART4SendTCB, ReportData4, 3 + pData[5] * 2 + 2);
                WriteDataToDMA_BufferTX4(3 + pData[5] * 2 + 2);
                break;
            }
            break;
        default:  //触摸屏与网关正常通讯
            if (pData[1] == 0x04 && pData[0] <= 65)
            {
                if (pData[3] == 0x00)
                {
                    ReportData4[0] = pData[0];
                    ReportData4[1] = pData[1];
                    ReportData4[2] = pData[5] * 2;//字节数=变量数*2
                    ReportData4[3] = ZZ_Wired_flag[pData[0] - 1];
                    ReportData4[4] = 0x00;
                    ReportData4[5] = ZZ_Wireles_flag[pData[0] - 1];
                    ReportData4[6] = 0x00;
                    memcpy(ReportData4 + 7, Collectors[pData[0] - 1], (pData[5] - 2) * 2);
                    CRCReport4 = GetCRC16(ReportData4, 3 + pData[5] * 2);
                    ReportData4[3 + pData[5] * 2] = CRCReport4 & 0x00FF;      //CRC低位
                    ReportData4[3 + pData[5] * 2 + 1] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
//						WriteDataToBuffer(5,ReportData4,0,3+pData[5]*2+2);
                    memcpy(UART4SendTCB, ReportData4, 3 + pData[5] * 2 + 2);
                    WriteDataToDMA_BufferTX4(3 + pData[5] * 2 + 2);
                    break;
                }
                if (pData[3] == 0x01)
                {
                    ReportData4[0] = pData[0];
                    ReportData4[1] = pData[1];
                    ReportData4[2] = pData[5] * 2;//字节数=变量数*2
                    ReportData4[3] = ZZ_Wired_flag[pData[0] - 1];
                    ReportData4[4] = 0x00;
                    memcpy(ReportData4 + 5, Collectors[pData[0] - 1], (pData[5] - 1) * 2);
                    CRCReport4 = GetCRC16(ReportData4, 3 + pData[5] * 2);
                    ReportData4[3 + pData[5] * 2] = CRCReport4 & 0x00FF;      //CRC低位
                    ReportData4[3 + pData[5] * 2 + 1] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
                    memcpy(UART4SendTCB, ReportData4, 3 + pData[5] * 2 + 2);
                    WriteDataToDMA_BufferTX4(3 + pData[5] * 2 + 2);
                    break;
                }
                if (pData[3] >= 0x02)
                {
                    ReportData4[0] = pData[0];
                    ReportData4[1] = pData[1];
                    ReportData4[2] = pData[5] * 2;//字节数=变量数*2
                    memcpy(ReportData4 + 3, Collectors[pData[0] - 1] + (pData[3] - 2) * 2, (pData[5]) * 2);
                    CRCReport4 = GetCRC16(ReportData4, 3 + pData[5] * 2);
                    ReportData4[3 + pData[5] * 2] = CRCReport4 & 0x00FF;      //CRC低位
                    ReportData4[3 + pData[5] * 2 + 1] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
                    memcpy(UART4SendTCB, ReportData4, 3 + pData[5] * 2 + 2);
                    WriteDataToDMA_BufferTX4(3 + pData[5] * 2 + 2);
                    break;
                }
            }
            if (pData[1] == 0x03 && pData[0] <= 64 && pData[0] >= 33)
            {
                ReportData4[0] = pData[0];
                ReportData4[1] = pData[1];
                ReportData4[2] = pData[5] * 2;//字节数=变量数*2
                memcpy(ReportData4 + 3, Controllers[pData[0] - 33], pData[5] * 2);
                CRCReport4 = GetCRC16(ReportData4, 3 + pData[5] * 2);
                ReportData4[3 + pData[5] * 2] = CRCReport4 & 0x00FF;      //CRC低位
                ReportData4[3 + pData[5] * 2 + 1] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
                memcpy(UART4SendTCB, ReportData4, 3 + pData[5] * 2 + 2);
                WriteDataToDMA_BufferTX4(3 + pData[5] * 2 + 2);
                break;
            }
            if (pData[1] == 0x06 && pData[0] <= 64 && pData[0] >= 33)
            {
                memcpy(UART4SendTCB, pData, 8);
                WriteDataToDMA_BufferTX4(8);
                if (pData[3] <= 0x03)
                {
                    Controllers[pData[0] - 33][pData[3] * 2] = pData[4];
                    Controllers[pData[0] - 33][pData[3] * 2 + 1] = pData[5];
                    crtl_cmd_num[pData[0] - 33][pData[3]] = 50;//子站有线发送记录，允许重复发送3次，收到清0，不允许继续发送。
                    crtl_cmd_numWX[pData[0] - 33][pData[3]] = 10;//子站无线发送记录，允许重复发送3次，收到清0，不允许继续发送。
                }
                break;
            }
            if (pData[1] == 0x06 && pData[0] <= 32 && pData[0] >= 9)
            {
                memcpy(UART4SendTCB, pData, 8);
                WriteDataToDMA_BufferTX4(8);
                if (pData[3] >= 0x08 && fertigation_flg == 1)   //表示该网关是水肥机；在查询9#~32#采集器时只查询6个参数，最后二个供触摸屏使用上报平台
                {
                    Collectors[pData[0] - 1][(pData[3] - 8) * 2] = pData[4];
                    Collectors[pData[0] - 1][(pData[3] - 8) * 2 + 1] = pData[5];
                }
                break;
            }
            if (pData[0] == 65 && pData[1] ==0x06)
            {
                memcpy(UART4SendTCB, pData, 8);
                WriteDataToDMA_BufferTX4(8);
                if (pData[3] == 0x0A)   //表示该网关是水肥机；在查询9#~32#采集器时只查询6个参数，最后二个供触摸屏使用上报平台
                {
                    SF_Lvbo641 =  pData[4];
                }
                if (pData[3] == 0x0B)   //表示该网关是水肥机；在查询9#~32#采集器时只查询6个参数，最后二个供触摸屏使用上报平台
                {
                    SF_Lvbo642 =  pData[4];
                }
                if (pData[3] == 0x0C)   //表示该网关是水肥机；在查询9#~32#采集器时只查询6个参数，最后二个供触摸屏使用上报平台
                {
                    SF_Lvbo646 = pData[4];
                }
                break;
            }
            if (pData[0] == 65 && pData[1] ==0x03)
            {
                ReportData4[0] = pData[0];
                ReportData4[1] = pData[1];
                ReportData4[2] = pData[5] * 2;//字节数=变量数*2
                ReportData4[3] =SF_Lvbo641;
                ReportData4[4] = 0x00;
                ReportData4[5] =SF_Lvbo642;
                ReportData4[6] = 0x00;
                ReportData4[7] =SF_Lvbo646;
                ReportData4[8] =0x00;
                CRCReport4 = GetCRC16(ReportData4, 3 + pData[5] * 2);
                ReportData4[3 + pData[5] * 2] = CRCReport4 & 0x00FF;      //CRC低位
                ReportData4[3 + pData[5] * 2 + 1] = (CRCReport4 & 0xFF00) >> 8; //CRC高位
                memcpy(UART4SendTCB, ReportData4, 3 + pData[5] * 2 + 2);
                WriteDataToDMA_BufferTX4(3 + pData[5] * 2 + 2);
                break;
            }
            break;
        }
    }
}
static void wg_init_readflash(void)     //网关初始化读flash;flash从0x0800 0000~0x0807 FFFF，共512k
{
    u8 init_flash_flg[2] = { 0,0 };
    Flash_Read(0x0807B000, init_flash_flg, 2);//触摸屏下发出厂网关设定参数需要写入flash0x0807 C000，从256k开始写入，0x0804加1，则增加64k,每次写入必需2Kbyte
    if (init_flash_flg[0] != 0xFF && init_flash_flg[1] != 0xFF)   //如果已经经过出厂配置，读出厂网关设定。否则使用程序初值
    {
        Flash_Read(0x0807B000, factory_gateway_set, 221);
        Flash_Read(0x0807B000 + 221, main_call, 11);
        Flash_Read(0x0807B000 + 232, voice_call, 11);
        Flash_Read(0x0807B000 + 243, third_call, 11);
        Flash_Read(0x0807B000 + 254, &close_433MHZ, 1);
    }
    Flash_Read(0x0807B800, init_flash_flg, 2);//触摸屏下发出厂网关设定参数需要写入flash0x0804 C000，从256k开始写入，0x0804加1，则增加64k,每次写入必需2Kbyte
    if (init_flash_flg[0] != 0xFF && init_flash_flg[1] != 0xFF)   //如果已经经过出厂配置，读出厂网关设定。否则使用程序初值
    {
        Flash_Read(0x0807B800, (unsigned char *)zero_rang.array_k_b, 384 * 4);
    }
    Flash_Read(0x0807C000, init_flash_flg, 2);//触摸屏下发出厂网关设定参数需要写入flash0x0804 C000，从256k开始写入，0x0804加1，则增加64k,每次写入必需2Kbyte
    if (init_flash_flg[0] != 0xFF && init_flash_flg[1] != 0xFF)   //如果已经经过出厂配置，读出厂网关设定。否则使用程序初值
    {
        Flash_Read(0x0807C000, (unsigned char *)zero_rang.array_k_b + 384 * 4, 384 * 4);
    }
    Flash_Read(0x0807C800, init_flash_flg, 2);//触摸屏下发出厂网关设定参数需要写入flash0x0804 C000，从256k开始写入，0x0804加1，则增加64k,每次写入必需2Kbyte
    if (init_flash_flg[0] != 0xFF && init_flash_flg[1] != 0xFF)   //如果已经经过出厂配置，读出厂网关设定。否则使用程序初值
    {
        Flash_Read(0x0807C800, (unsigned char *)zero_rang.array_k_b + 768 * 4, 272 * 4);//0x0807C800~0x0807CC3F;其它数据可以从0x0807CC40开始保存
    }
    Flash_Read(0x0807D000, init_flash_flg, 2);//触摸屏下发出厂网关设定参数需要写入flash0x0804 C000，从256k开始写入，0x0804加1，则增加64k,每次写入必需2Kbyte
    if (init_flash_flg[0] != 0xFF)   //如果已经经过出厂配置，读网关配置的433MHZ信道设定。否则使用程序初值SI4463_Channel=0
    {
        SI4463_Channel = init_flash_flg[0];
    }
}

static void Send_slave_cmd(void)
{
    //      slave_set_flg=0;//网关正常发送命令给子站
    switch (Query_Flag)
    {
    case CONTROLLERS_CMD:

        //初始化未完成 或 发送子站命令成功收到回复无需发送，同时控制器子站ID小于等于32个。下面的发送子站命令跳过;
        while ((crtl_cmd_num[Query_Index_Controller][ctrl_j] <= 0) && (Query_Index_Controller < 32) && ctrl_j <= 3)   //CSH_Wired_finish==0|
        {
            ctrl_j++;
            if (ctrl_j >= 4)
            {
                Query_Index_Controller++;
                ctrl_j = 0;
            }
        }
        if (Query_Index_Controller < 32 && ctrl_j <= 3)
        {
            bytelen3 = WriteSingleRegister(Query_Index_Controller + 33, ctrl_j, Controllers[Query_Index_Controller] + ctrl_j * 2, ReportData3);
            memcpy(USART3SendTCB, ReportData3, bytelen3);
            WriteDataToDMA_BufferTX3(bytelen3);

            if (crtl_cmd_num[Query_Index_Controller][ctrl_j] > 0) crtl_cmd_num[Query_Index_Controller][ctrl_j]--;//子站发送记录，每发送一次减1，收到回复清0，否则重复发送3次
            ctrl_j++;
            if (ctrl_j >= 4)
            {
                Query_Index_Controller++;
                ctrl_j = 0;
            }
        }
        if (Query_Index_Controller >= 32)
        {
            Query_Index_Controller = 0;
            ctrl_j = 0;
            Query_Flag = COLLECTORS_CMD;
        }
        Start_timerEx(WG_SENDZZ_EVT, send_Collectors_time);
        break;

    case COLLECTORS_CMD:

        while ((online_slaveID[Query_Index_Collector] == 0) && (Query_Index_Collector < 64))
        {
            Query_Index_Collector++;
        }
        if (Query_Index_Collector < 64)
        {
            bytelen3 = ReadData(Query_Index_Collector + 1, READ_HOLDING_REGISTER, 0x0000, 0x0008, ReportData3);
            if (Query_Index_Collector > 7 && fertigation_flg == 1)   //水肥机网关，从9#~32#采集器由41#~64#控制器模拟，只能6个参数，最后二个参数给触摸屏使用
            {
                bytelen3 = ReadData(Query_Index_Collector + 1, READ_HOLDING_REGISTER, 0x0000, 0x0006, ReportData3);
            }
            memcpy(USART3SendTCB, ReportData3, bytelen3);
            WriteDataToDMA_BufferTX3(bytelen3);
            Query_Index_Collector++;
        }
        if (Query_Index_Collector > 63)   //采集器查询完毕，开始查询控制器
        {
            Query_Index_Collector = 0;
            Query_Flag = ZZ_QUERY_COLLECTOR;
            Start_timerEx(WG_SENDZZ_EVT, send_Collectors_time);
            break;
        }
        else
        {
            Query_Flag = CONTROLLERS_CMD;
            Start_timerEx(WG_SENDZZ_EVT, send_Collectors_time);

            break;
        }

    case ZZ_QUERY_COLLECTOR:

        while ((online_slaveID[Query_IndexZZ_C_YX] != 0) && (Query_IndexZZ_C_YX < 32))
        {
            Query_IndexZZ_C_YX++;
        }
        if (Query_IndexZZ_C_YX < 32)
        {
            bytelen3 = ReadData(Query_IndexZZ_C_YX + 1, READ_HOLDING_REGISTER, 0x0000, 0x0008, ReportData3);
            memcpy(USART3SendTCB, ReportData3, bytelen3);
            WriteDataToDMA_BufferTX3(bytelen3);
            if (init_cmd_numYX[Query_IndexZZ_C_YX] > 0)
            {
                init_cmd_numYX[Query_IndexZZ_C_YX]--;
            }
            else
            {
                Query_IndexZZ_C_YX++;
            }
        }
        if (Query_IndexZZ_C_YX > 31)
        {
            Query_IndexZZ_C_YX = 0;   //采集器查询完毕，开始查询控制器
        }
        Query_Flag = ZZ_QUERY_CONTROLLER;
        Start_timerEx(WG_SENDZZ_EVT, send_Collectors_time);

        break;

    case ZZ_QUERY_CONTROLLER:

        while ((online_slaveID[Query_IndexZZ_K_YX + 32] != 0) && (Query_IndexZZ_K_YX < 32))
        {
            Query_IndexZZ_K_YX++;
        }
        if (Query_IndexZZ_K_YX < 32)
        {
            bytelen3 = ReadData(Query_IndexZZ_K_YX + 33, READ_HOLDING_REGISTER, 0x0000, 0x0008, ReportData3);
            memcpy(USART3SendTCB, ReportData3, bytelen3);
            WriteDataToDMA_BufferTX3(bytelen3);
            if (init_cmd_numYX[Query_IndexZZ_K_YX + 32] > 0)
            {
                init_cmd_numYX[Query_IndexZZ_K_YX + 32]--;
            }
            else
            {
                Query_IndexZZ_K_YX++;
            }
        }
        if (Query_IndexZZ_K_YX > 31)   //采集器查询完毕，开始查询控制器
        {
            Query_IndexZZ_K_YX = 0;

            if (ZZ_Wireles_flag[64] == 0)
            {
                u8 i;
                CSH_countYX = 0;
                for (i = 0; i < 64; i++)
                {
                    CSH_countYX = CSH_countYX | init_cmd_numYX[i];
                }
                if ((CSH_countYX == 0) && (CSH_countWX == 0))
                {
                    ZZ_Wireles_flag[64] = 1;
                }
            }
            if (Query_Wired_WirelesYX > 0)
            {
                ZZ_Wired_flag[Query_Wired_WirelesYX - 1] = ZZ_temp_stateYX;
            }
            else
            {
                ZZ_Wired_flag[63] = ZZ_temp_stateYX;
            }
            ZZ_temp_stateYX = ZZ_Wired_flag[Query_Wired_WirelesYX];
            ZZ_Wired_flag[Query_Wired_WirelesYX] = 0x02;//子站有线正在检测通信标志
//							online_slaveID[Query_Wired_WirelesYX]=0x00;//子站有线正在检测通信标志
            if (ZZ_temp_stateYX == 0x02)
            {
                ZZ_temp_stateYX = 0;
            }
            Query_Wired_WirelesYX++;
            if (Query_Wired_WirelesYX > 63)
            {
                Query_Wired_WirelesYX = 0;
            }
        }
        Query_Flag = CONTROLLERS_CMD;
        Start_timerEx(WG_SENDZZ_EVT, send_Collectors_time);

        break;
    default:
        Start_timerEx(WG_SENDZZ_EVT, 500);
        break;
    }
}
static void RxReport3(u8 len, u8 *pData)   //与子站通信，接收子站回复
{
    if (GetCRC16(pData, len) == 0)
    {
        Start_timerEx(WG_SENDZZ_EVT, 122);
        online_slaveID[pData[0] - 1] = pData[0];//表示该子站在线，记录下来
        ZZ_Wired_flag[pData[0] - 1] = 1;//收到子站有线通信，设置标志，0x02有线，不为0表示该子站存在
        init_cmd_numYX[pData[0] - 1] = 0;

        if (pData[1] == 0x03)
        {
            memcpy(Collectors[pData[0] - 1], pData + 3, pData[2]);
            return;
        }

        if (pData[1] == 0x06)
        {
            if ((pData[2] == 0x00) && (pData[3] <= 0x03) && (pData[0] >= 33) && (pData[0] <= 64))
                //slave_set_flg在Send_slave_cmd和RxReport5函数中设定，slave_set_flg==0正常接收子站回复
            {
                crtl_cmd_num[pData[0] - 33][pData[3]] = 0;//表示子站收到控制指令，无需再发送写命令
                return;
            }
        }
        if (pData[1] == 0x10)
        {
            if ((pData[0] >= 33) && (pData[3] >= 4) && (pData[3] <= 15) && (slave_set_flg == 1))   //slave_set_flg==1接收设定参数控制子站回复
            {
                ctrlslave_param_flg[pData[0] - 33] = 0;
                return;
            }
            if ((pData[3] >= 16) && (pData[3] <= 33) && (slave_set_flg == 2))   //slave_set_flg==2接收设定参数采集器和控制器子站回复
            {
                cjqslave_param_flg[pData[0] - firstwx_xiabiao_i - 1] = 0;
                return;
            }
            if ((pData[3] >= 34) && (pData[3] <= 35) && (slave_set_flg == 3))   //slave_set_flg==3接收设定子站地址及信道的回复
            {
                slaveID_channel_flg = 0;
                return;
            }
            if ((pData[0] == SF_SlaveID_0) && (pData[3] >= 40) && (pData[3] < 68))
            {
                SF_cmd_num[pData[3]-40][0] = 0;
                return;
            }
            if ((pData[0] == SF_SlaveID_1) && (pData[3] >= 40) && (pData[3] < 68))
            {
                SF_cmd_num[pData[3]-40][1] = 0;
                return;
            }
        }
    }
}

static void WriteDataToDMA_BufferTX1(uint16_t size)
{
    TxFlag1 = 1;
    //    USART_DMACmd(USART2, USART_DMAReq_Tx, DISABLE);  // 开启串口DMA发送
    //    DMA_Cmd(DMA1_Channel7, DISABLE);        //开始DMA发送
    DMA1_Channel4->CNDTR = (uint16_t)size; // 设置要发送的字节数目
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  // 开启串口DMA发送
    DMA_Cmd(DMA1_Channel4, ENABLE);        //开始DMA发送
}

static void WriteDataToDMA_BufferTX2(uint16_t size)
{
    TxFlag2 = 1;
    //    USART_DMACmd(USART2, USART_DMAReq_Tx, DISABLE);  // 开启串口DMA发送
    //    DMA_Cmd(DMA1_Channel7, DISABLE);        //开始DMA发送
    DMA1_Channel7->CNDTR = (uint16_t)size; // 设置要发送的字节数目
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);  // 开启串口DMA发送
    DMA_Cmd(DMA1_Channel7, ENABLE);        //开始DMA发送
}
static void WriteDataToDMA_BufferTX3(uint16_t size)
{
    TXENABLE3;
    TxFlag3 = 1;
    DMA1_Channel2->CMAR = (uint32_t)USART3SendTCB;
    DMA1_Channel2->CNDTR = (uint16_t)size; // 设置要发送的字节数目
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);  // 开启串口DMA发送
    DMA_Cmd(DMA1_Channel2, ENABLE);        //开始DMA发送
}
static void SF_WriteDataToDMA_BufferTX3(uint16_t size)
{
    TXENABLE3;
    TxFlag3 = 1;
    DMA1_Channel2->CMAR = (uint32_t)SF_USART3SendTCB;
    DMA1_Channel2->CNDTR = (uint16_t)size; // 设置要发送的字节数目
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);  // 开启串口DMA发送
    DMA_Cmd(DMA1_Channel2, ENABLE);        //开始DMA发送
}
static void WriteDataToDMA_BufferTX4(uint16_t size)
{
    //	  test_DMA=size;
    TXENABLE4;
    TxFlag4 = 1;
    DMA2_Channel5->CNDTR = (uint16_t)size; // 设置要发送的字节数目
    USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);  // 开启串口DMA发送
    DMA_Cmd(DMA2_Channel5, ENABLE);        //开始DMA发送
}
void initialRealSendBuff(void)
{
    real_send[0] = 0x01;
    real_send[1] = 0x00;
    real_send[2] = 0x00;
    real_send[3] = 0x00;
    real_send[4] = 0x45;
    real_send[5] = 0x00;
    real_send[6] = factory_gateway_set[2];
    real_send[7] = factory_gateway_set[3];
    real_send[8] = factory_gateway_set[4];
    real_send[9] = factory_gateway_set[5];	  //边缘网关ID
    real_send[10] = 0xCC;
    real_send[11] = 0x01;
    real_send[12] = 0x02;
    real_send[13] = 0x03;
    real_send[14] = factory_gateway_set[6];
    real_send[15] = factory_gateway_set[7];
    real_send[16] = factory_gateway_set[8];
    real_send[17] = factory_gateway_set[9];
    real_send[18] = factory_gateway_set[5];	  //边缘网关ID
    real_send[19] = 0x04;	//0x06 命令标志
    real_send[20] = 0x00;
    real_send[21] = 0x01; //寄存器起始地址
    real_send[22] = 0x50;
    real_send[23] = 0x00;	//数据长度80个字节
}

/*向平台建立链路函数类*/
static void Net_connect_init(void)   //网关初始化网络连接到平台通路
{
    switch (factory_gateway_set[0])   //网络类型=1 电信，2 移动，3 以太网，4 WiFi，5 USB，6 无连接；
    {
    //无连接  开始########################################
    case 0x00:
        if (factory_gateway_set[1] == 0)net_connect_count = 0;
        Start_timerEx(NET_INIT_EVT, 1000);
        break;
    //无连接 结束#########################################
    //电信 cdma 开始---------------------------------
    case 0x01:
        switch (factory_gateway_set[1])   //协议类型=1 TCP/IP;2 MQTT; 3 SDK; 4 其它；
        {
        //电信TCP/IP
        case 0x01:
            cdma_tcp_init();//包含了cdma_mqtt_sdk_init
            break;			//电信MQTT
        case 0x02:
            cdma_tcp_init();
            break;
        //电信SDK
        case 0x03:
            cdma_tcp_init();
            break;
        //电信other
        case 0x04:
            cdma_tcp_init();
            break;
        default:
            Start_timerEx(NET_INIT_EVT, 1000);
            break;
        }
        break;
    //电信 cdma 结束------------------------------------

    //移动 gprs	开始====================================
    case 0x02:
        switch (factory_gateway_set[1])   //协议类型=1 TCP/IP;2 MQTT; 3 SDK; 4 其它；
        {
        //移动TCP/IP
        case 0x01:
            gprs_tcp_init();//该函数包括了gprs_mqtt_init()，gprs_sdk_init();
            break;
        //移动MQTT
        case 0x02:
            gprs_tcp_init();//该函数包括了gprs_mqtt_init()，gprs_sdk_init();
            break;
        //移动SDK
        case 0x03:
            gprs_tcp_init();//该函数包括了gprs_mqtt_init()，gprs_sdk_init();
            break;
        //移动other
        case 0x04:
            gprs_other_init();
            break;
        default:
            Start_timerEx(NET_INIT_EVT, 1000);
            break;
        }
        break;
    //移动 gprs	结束====================================

    //以太网 开始++++++++++++++++++++++++++++++++++++++
    case 0x03:
        switch (factory_gateway_set[1])   //协议类型=1 TCP/IP;2 MQTT; 3 SDK; 4 其它；
        {
        //电信TCP/IP
        case 0x01:
            Ethernet_tcp_init();
            break;			//电信MQTT
        case 0x02:
            Ethernet_mqtt_sdk_init();
            break;
        //电信SDK
        case 0x03:
            Ethernet_mqtt_sdk_init();
            break;
        //电信other
        case 0x04:
            Ethernet_other_init();
            break;
        default:
            Start_timerEx(NET_INIT_EVT, 1000);
            break;
        }
        break;
    //以太网 结束++++++++++++++++++++++++++++++++++++++++

    //WiFi 开始&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    case 0x04:
        Start_timerEx(NET_INIT_EVT, 1000);
        break;

    //WiFi 结束&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    //USB  开始########################################
    case 0x05:
        Start_timerEx(NET_INIT_EVT, 1000);
        break;
    //USB 结束#########################################
    default:
        Start_timerEx(NET_INIT_EVT, 1000);
        break;
    }
}

/* 与模块通讯的设计思路：共有三个程序向cdma模块发命令和数据：1.初始化cdma模块函cdma_tcp_init；
2.回复平台控制命令函数cmd_reply；3.CDMA向平台上报数据函cdma_tcp_send。接收cdma命令和数据的函数为RxReport2。因此，采用
send_message_type变量记录发送数据的类型：（0x01为初始化cdma模块，0x02为回复平台控制命令，0x03为向平台上报数据，0x04类型为快速回复极码平台的控制指令）。
变量cmd_flg控制cdma_tcp_init初始化模块程序执行的顺序；module_send_flg_flg控制cdma_tcp_send函数执行的顺序；函数cmd_reply只有一条指令
，无需控制执行顺序。at_data_flg标志0x00为发at指令，0x01为数据指令。send_flag=0x01禁止向cdma发送命令和数据
（本指令组合除外），send_flag=0x00允许向cdma发送命令和数据。在接收cdma命令和数据的函数为RxReport2中cdma_ok标志所
发命令和数据是否正确，正确cdma_ok=1；如果cdma返回ERROR，则重复执行一次该指令，如果继续出错，返回cdma初始化。
*/

static void cdma_tcp_init(void)   //电信TCP/IP连接
{
    send_flg = 0x01;
    send_message_type = 0x01;
    switch (cmd_flg)
    {
    /*cmd_flg=0xff,连接完成*/
    case 0x01://系统重新启动
        cmd_flg = 0x02;
        RESET_CTL_ENABLE;
        Start_timerEx(NET_INIT_EVT, 150);//原来150:CDMA
        break;

    case 0x02:
        cmd_flg = 0x05;
        RESET_CTL_DISABLE;
        Start_timerEx(NET_INIT_EVT, 5000);//原来20000（20s）
        break;

    case 0x03:
        cmd_flg = 0x04;//cdma开机
        TERM_ON_CTL_ENABLE;
        Start_timerEx(NET_INIT_EVT, 150);//CDAM:150
        break;
    case 0x04:
        cmd_flg = 0x01;
        TERM_ON_CTL_DISABLE;
        Start_timerEx(NET_INIT_EVT, 5000);//原来20000;20s
        break;

    case 0x05:
        //			WriteDataToBuffer(2,(unsigned char *)"ATE0\r\n",0,6); //关闭回显
        memcpy(USART2SendTCB, (unsigned char *)"ATE0\r\n", 6);
        WriteDataToDMA_BufferTX2(6);
        next_atdata_flg = 0x06;
        Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
        break;

    case 0x06:
        //			WriteDataToBuffer(2,(unsigned char *)"AT^RSSIREP=0\r\n",0,14);//GSM:"AT+CNMI=3,2,0,0,0\r\n",19,800);，GSM主动上报短信
        memcpy(USART2SendTCB, (unsigned char *)"AT^RSSIREP=0\r\n", 14);
        WriteDataToDMA_BufferTX2(14);
        next_atdata_flg = 0x07;
        Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
        break;

    case 0x07:
        //			WriteDataToBuffer(2,(unsigned char *)"AT+IFC=2,2\r\n",0,12);//设置DTE和DCE数据传输控制方式,RTS和CTS
        memcpy(USART2SendTCB, (unsigned char *)"AT+IFC=2,2\r\n", 12);
        WriteDataToDMA_BufferTX2(12);
        next_atdata_flg = 0x08;
        Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
        break;

    case 0x08:
        //			memcpy(USART2SendTCB,(unsigned char *)"AT+CMGD=0,4\r\n",13);//4表示删除所有的短信，包括未读短信;CDMA为0,4
        //      WriteDataToDMA_BufferTX2(13);
        next_atdata_flg = 0x09;
        cmd_flg = 0x09;
        Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
        break;
    case 0x09:
        //			WriteDataToBuffer(2,(unsigned char *)"AT^IPDATMODE=1\r\n",0,16);//CDMA:"AT^IPDATMODE=1\r\n",16,800 当有新的TCP/UP数据到达是否主动上报，1上报
        memcpy(USART2SendTCB, (unsigned char *)"AT^IPDATMODE=1\r\n", 16);
        WriteDataToDMA_BufferTX2(16);
        next_atdata_flg = 0x0A;
        Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
        break;
    case 0x0A:
        //			WriteDataToBuffer(2,(unsigned char *)"AT^RSSIREP=0\r\n",0,14);//TCP/UDP链接初始化；CDMA:"AT^IPINIT=,\"CARD\",\"CARD\"\r\n",26,6000
        memcpy(USART2SendTCB, (unsigned char *)"AT^RSSIREP=0\r\n", 14);
        WriteDataToDMA_BufferTX2(14);
        next_atdata_flg = 0x0B;
        Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
        break;

    case 0x0B:
        //				WriteDataToBuffer(2,(unsigned char *)"AT+CNMI=1,2,0,0,0\r\n",0,19);//建立Profile服务第一条指令
        //cdma收到短信直接上报
        memcpy(USART2SendTCB, (unsigned char *)"AT+CNMI=1,2,0,0,0\r\n", 19);
        WriteDataToDMA_BufferTX2(19);
        next_atdata_flg = 0x0C;
        Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
        break;
    case 0x0C:
        //				WriteDataToBuffer(2,(unsigned char *)"AT^IPINIT=,\"CARD\",\"CARD\"\r\n",0,26);//建立Profile服务第二条指令
        //TCP/UDP 链路初始化 CDMA:"AT^IPINIT=,\"CARD\",\"CARD\"\r\n",26
        memcpy(USART2SendTCB, (unsigned char *)"AT^IPINIT=,\"CARD\",\"CARD\"\r\n", 26);
        WriteDataToDMA_BufferTX2(26);
        next_atdata_flg = 0x0D;
        Start_timerEx(NET_INIT_EVT, 5 * CMD_WAIT_TIME);
        break;
    case 0x0D:   //建立Profile服务第三条指令
    {
        unsigned char _ipopen[48] = { "AT^IPOPEN=1,\"TCP\",\"115.239.134.165\",00502,5002\r\n" }; //30个字节
        memcpy(_ipopen + 19, factory_gateway_set + 31, 15);		//IP地址
        memcpy(_ipopen + 36, factory_gateway_set + 47, 5);//端口号
        memcpy(USART2SendTCB, _ipopen, 48);
        WriteDataToDMA_BufferTX2(48);
        if (factory_gateway_set[1] == 1)next_atdata_flg = 0x15;//MQTT为0x0F,TCP为0x15
        if (factory_gateway_set[1] == 2)next_atdata_flg = 0x0F;//MQTT为0x0F,TCP为0x15
        if (factory_gateway_set[1] == 3)next_atdata_flg = 0x0F;//MQTT为0x0F,TCP为0x15
        if (factory_gateway_set[1] == 4)next_atdata_flg = 0x0F;//MQTT为0x0F,TCP为0x15
        Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
        break;
    }
    //MQTT初始化开始
    case 0x0F:
        cmd_flg = 0x10;
        Start_timerEx(NET_INIT_EVT, DATA_WAIT_TIME);//建立TCP链路需要的等待时间
        break;
    case 0x10:	//MQTT建立连接开始AT指令
        mqtt_connect();//将建立连接的内容送到ReportData2，并计算长度到mqtt_len
        bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);
        send_at_cdma((unsigned char *)message_len_char1, 21);//mqtt数据包长度为41,先发at指令，下面发数据
        next_atdata_flg = 0x11;
        Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
        break;
    case 0x11:   //MQTT建立连接开始DATA指令
        mqtt_connect();//将建立连接的内容送到ReportData2，并计算长度到mqtt_len
        bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);
        send_data_module(ReportData2, (unsigned char *)message_len_char1);//mqtt数据包长度为41
        next_atdata_flg = 0x12;
        Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
        break;
    case 0x12://MQTT 建立回复等待时间
        cmd_flg = 0x13;
        send_flg = 0x00;//允许接收MQTT回复
        send_message_type = 0x00;
        Start_timerEx(NET_INIT_EVT, DATA_WAIT_TIME);
        break;
    //MQTT建立连接结束，订阅主题开始
    case 0x13:
        send_flg = 0x01;
        mqtt_subscribe();		  //订阅主题 ，将订阅主题的内容送到ReportData2，并计算长度到mqtt_len
        bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);
        send_at_cdma((unsigned char *)message_len_char1, 21);//订阅主题数据包长度为:message_len_char；先发at指令，下面发数据
        next_atdata_flg = 0x14;
        Start_timerEx(NET_INIT_EVT, 3 * CMD_WAIT_TIME);
        break;
    case 0x14:
        mqtt_subscribe();		  //订阅主题 ，将订阅主题的内容送到ReportData2，并计算长度到mqtt_len
        bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);
        send_data_module(ReportData2, (unsigned char *)message_len_char1);//订阅主题数据包长度为:message_len_char
        next_atdata_flg = 0x15;
        Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
        break;
    //订阅主题结束
    //MQTT初始化结束
    case 0x15:
        send_flg = 0x00;//允许回复平台控制命令函数cmd_reply；向平台上报数据函数report8；向gsm或cdma模块发命令
        send_message_type = 0x00;
        module_send_flg = 0x01;//TCB协议允许CDMA向平台发送数据执行步骤控制标志
        cmd_flg = 0xFF;//CDMA初始化完成
        net_connect_count = 0;
        send_message_type = 0x00;
        Start_timerEx(SEND_PLATFORM_EVT, DATA_WAIT_TIME);//允许CDMA向平台发送数据
        break;
    default:
        if (cmd_flg != 0xFF)
        {
            while (1);
        }
        break;
    }
}

//static void cdma_other_init(void) //电信other连接
//{
//}

static void gprs_tcp_init(void)   //移动TCP/IP连接
{
    send_flg = 0x01;
    send_message_type = 0x01;
    switch (cmd_flg)
    {
    /*cmd_flg=0xff,重新初始化cdma模块；0xfd，rest开始；0xfc，rest结束(10ms<rest时间<2s)；0xfb CDMA开机使能，0xfa使能结束*/
    case 0x01://系统重新启动
        cmd_flg = 0x01;//gprs模块REST会使模块死机，只有关机，开机才能重新启动
        TERM_ON_CTL_ENABLE;//gprs关机
//			RESET_CTL_ENABLE;
        Start_timerEx(NET_INIT_EVT, 1100);//原来150:CDMA;GSM:50
        break;
    case 0x02:
        cmd_flg = 0x05;
        TERM_ON_CTL_DISABLE;
        //			RESET_CTL_DISABLE;
        Start_timerEx(NET_INIT_EVT, 3000);//原来20000（20s）
        break;
    case 0x03:
        cmd_flg = 0x04;//cdma开机
        TERM_ON_CTL_ENABLE;
        Start_timerEx(NET_INIT_EVT, 1100);//CDAM:150;GSM:1500(>1S,<2.6S)
        break;
    case 0x04:
        cmd_flg = 0x05;
        TERM_ON_CTL_DISABLE;
        Start_timerEx(NET_INIT_EVT, 3000);//原来20s
        break;

    case 0x05:
        memcpy(USART2SendTCB, (unsigned char *)"ATE0\r\n", 6);
        WriteDataToDMA_BufferTX2(6);
        //		  WriteDataToBuffer(2,"ATE0\r\n",0,6); //关闭回显
        next_atdata_flg = 0x06;
        Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
        break;

    case 0x06:
        memcpy(USART2SendTCB, (unsigned char *)"AT+CNMI=3,2,0,0,0\r\n", 19);
        WriteDataToDMA_BufferTX2(19);
        //			WriteDataToBuffer(2,"AT+CNMI=3,2,0,0,0\r\n",0,19);//GSM:"AT+CNMI=3,2,0,0,0\r\n",19,800);，GSM主动上报短信
        next_atdata_flg = 0x07;
        Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
        //CDMA: 0xF8,0xF9,0xF7,"AT^RSSIREP=0\r\n",14,800
        break;

    case 0x07:
        memcpy(USART2SendTCB, (unsigned char *)"AT+IFC=2,2\r\n", 12);
        WriteDataToDMA_BufferTX2(12);
        //			WriteDataToBuffer(2,"AT+IFC=2,2\r\n",0,12);//设置DTE和DCE数据传输控制方式,RTS和CTS
        next_atdata_flg = 0x08;
        Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
        break;

    case 0x08:
        memcpy(USART2SendTCB, (unsigned char *)"AT+CMGD=1,4\r\n", 13);
        WriteDataToDMA_BufferTX2(13);
        //			WriteDataToBuffer(2,"AT+CMGD=1,4\r\n",0,13);//4表示删除所有的短信，包括未读短信;CDMA为0,4;GSM为1,4;其余与GSM相同
        next_atdata_flg = 0x09;
        Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
        break;
    case 0x09:
        memcpy(USART2SendTCB, (unsigned char *)"AT^SICS=0,conType,GPRS0\r\n", 25);
        WriteDataToDMA_BufferTX2(25);
        //			WriteDataToBuffer(2,"AT^SICS=0,conType,GPRS0\r\n",0,25);//CDMA:"AT^IPDATMODE=1\r\n",16,800 当有新的TCP/UP数据到达是否主动上报，1上报
        //GSM:建立连接Profile需要二条指令，这里是第一条指令.当有新的TCP/UP数据到达主动上报是GSM的默认值，无需设定
        next_atdata_flg = 0x0A;
        Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
        break;
    case 0x0A:
        memcpy(USART2SendTCB, (unsigned char *)"AT^SICS=0,apn,jmwg\r\n", 20);
        WriteDataToDMA_BufferTX2(20);
        //			WriteDataToBuffer(2,"AT^SICS=0,apn,jmwg\r\n",0,20);//TCP/UDP链接初始化；CDMA:"AT^IPINIT=,\"CARD\",\"CARD\"\r\n",26,6000
        //GSM:建立连接Profile的第二条指令。"AT^SICS=0,apn,jmwg\r\n",20,800
        next_atdata_flg = 0x0B;
        Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
        break;

    /*与网页平台建立链路*/
    //WriteDataToBuffer(2,"AT^SISC=0\r\n",0,11);//关闭链路1#；CDMA为"AT^IPCLOSE=1\r\n",0,14。GSM为"AT^SISC=1\r\n",0,11

    case 0x0B:
        memcpy(USART2SendTCB, (unsigned char *)"AT^SISS=0,srvType,socket\r\n", 26);
        WriteDataToDMA_BufferTX2(26);
        //				WriteDataToBuffer(2,"AT^SISS=0,srvType,socket\r\n",0,26);//建立Profile服务第一条指令
        next_atdata_flg = 0x0C;
        Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
        break;
    case 0x0C:
        memcpy(USART2SendTCB, (unsigned char *)"AT^SISS=0,conId,0\r\n", 19);
        WriteDataToDMA_BufferTX2(19);
        //				WriteDataToBuffer(2,"AT^SISS=0,conId,0\r\n",0,19);//建立Profile服务第二条指令
        next_atdata_flg = 0x0D;
        Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
        break;
    case 0x0D:   //建立Profile服务第三条指令
    {
        unsigned char _ipopen[53] = { "AT^SISS=0,address,\"socktcp://115.239.134.165:00502\"\r\n" }; //29个字节,GSM命令；CDMA命令全改了
        memcpy(_ipopen + 29, factory_gateway_set + 31, 15);		//IP地址
        memcpy(_ipopen + 45, factory_gateway_set + 47, 5);//端口号
//				memcpy(_ipopen+29,server_ip,20);		//IP地址:端口号"
//				_ipopen[49]='"';
//				_ipopen[50]='\r';_ipopen[51]='\n';
        memcpy(USART2SendTCB, _ipopen, 53);
        WriteDataToDMA_BufferTX2(53);
        //				WriteDataToBuffer(2,_ipopen,0,52);//命令为AT^SISS=0,address,"socktcp://183.056.016.057:09988"
        next_atdata_flg = 0x0E;
        Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
        break;
    }
    case 0x0E:
        memcpy(USART2SendTCB, (unsigned char *)"AT^SISO=0\r\n", 11);//移动使用
        WriteDataToDMA_BufferTX2(11);
        //				WriteDataToBuffer(2,"AT^SISO=0\r\n",0,11);//建立Profile服务第四条指令;打开0#连接
        if (factory_gateway_set[1] == 1)next_atdata_flg = 0x15;//MQTT为0x0F,TCP为0x15
        if (factory_gateway_set[1] == 2)next_atdata_flg = 0x0F;//MQTT为0x0F,TCP为0x15
        if (factory_gateway_set[1] == 3)next_atdata_flg = 0x0F;//MQTT为0x0F,TCP为0x15
        Start_timerEx(NET_INIT_EVT, 2 * CMD_WAIT_TIME);
        break;
    //MQTT初始化开始
    case 0x0F:
        cmd_flg = 0x10;
        Start_timerEx(NET_INIT_EVT, DATA_WAIT_TIME);//建立TCP链路需要的等待时间
        break;
    case 0x10:	//MQTT建立连接开始AT指令
        mqtt_connect();//将建立连接的内容送到ReportData2，并计算长度到mqtt_len
        bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);
        send_at_gprs((unsigned char *)message_len_char1, 15);//mqtt数据包长度为41,先发at指令，下面发数据
        next_atdata_flg = 0x11;
        Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
        break;
    case 0x11:   //MQTT建立连接开始DATA指令
        mqtt_connect();//将建立连接的内容送到ReportData2，并计算长度到mqtt_len
        bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);
        send_data_module(ReportData2, (unsigned char *)message_len_char1);//mqtt数据包长度为41
        next_atdata_flg = 0x12;
        Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
        break;
    case 0x12://MQTT 建立回复等待时间
        cmd_flg = 0x13;
        send_flg = 0x00;//允许接收MQTT回复
        send_message_type = 0x00;
        Start_timerEx(NET_INIT_EVT, DATA_WAIT_TIME);
        break;
    //MQTT建立连接结束，订阅主题开始
    case 0x13:
        send_flg = 0x01;
        mqtt_subscribe();		  //订阅主题 ，将订阅主题的内容送到ReportData2，并计算长度到mqtt_len
        bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);
        send_at_gprs((unsigned char *)message_len_char1, 15);//订阅主题数据包长度为:message_len_char；先发at指令，下面发数据
        next_atdata_flg = 0x14;
        Start_timerEx(NET_INIT_EVT, 3 * CMD_WAIT_TIME);
        break;
    case 0x14:
        mqtt_subscribe();		  //订阅主题 ，将订阅主题的内容送到ReportData2，并计算长度到mqtt_len
        bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);
        send_data_module(ReportData2, (unsigned char *)message_len_char1);//订阅主题数据包长度为:message_len_char
        next_atdata_flg = 0x15;
        Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
        break;
    //		//订阅主题结束
    //		//MQTT初始化结束
    case 0x15:
        send_flg = 0x00;//允许回复平台控制命令函数cmd_reply；向平台上报数据函数report8；向gsm或cdma模块发命令
        send_message_type = 0x00;
        module_send_flg = 0x01;
        cmd_flg = 0xFF;//初始化结束，cmd_flg就为0xFF，在整个运行过程中一直不变
        net_connect_count = 0;
        Start_timerEx(SEND_PLATFORM_EVT, DATA_WAIT_TIME);//向CDMA发数据
        break;
    default:
        if (cmd_flg != 0xFF)
        {
            while (1);
        }
        break;
    }
}

static void gprs_other_init(void)   //移动other连接
{
    Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
}

static void Ethernet_tcp_init(void)   //电信TCP/IP连接
{
    send_flg = 0x00;//允许回复平台控制命令函数cmd_reply；向平台上报数据函数report8；向gsm或cdma模块发命令
    send_message_type = 0x00;
    module_send_flg = 0x01;//TCB协议允许CDMA向平台发送数据执行步骤控制标志
    cmd_flg = 0xFF;//CDMA初始化完成
    net_connect_count = 0;
    Start_timerEx(SEND_PLATFORM_EVT, DATA_WAIT_TIME);//允许CDMA向平台发送数据
}

static void Ethernet_mqtt_sdk_init(void)   //电信mqtt_sdk连接
{
    Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
}

static void Ethernet_other_init(void)   //电信other连接
{
    Start_timerEx(NET_INIT_EVT, CMD_WAIT_TIME);
}

/*向平台发送数据函数类-------------------------------------------------------------------------------------*/
static void send_platform(void)   //向平台发送数据
{
    switch (factory_gateway_set[0])   //网络类型=1 电信，2 移动，3 以太网，4 WiFi，5 USB，0 无连接；
    {
    //无连接
    case 0x00:
        halt_RxReport2++;
        if (factory_gateway_set[1] == 0)
        {
            halt_RxReport2 = 0;
            cmd_flg = 0xFF;
        }
        Start_timerEx(SEND_PLATFORM_EVT, DATA_WAIT_TIME);//允许CDMA向平台发送数据
        break;
    //电信 cdma 开始---------------------------------
    case 0x01:
        switch (factory_gateway_set[1])   //协议类型=1 TCP/IP;2 MQTT; 3 SDK; 4 其它；
        {
        //电信TCP/IP
        case 0x01:
            cdma_tcp_send();
            break;
        //电信MQTT
        case 0x02:
            cdma_mqtt_send();
            break;
        //电信SDK
        case 0x03:
            cdma_sdk_send();
            break;
        //电信other
        case 0x04:
            cdma_sdk_send();
            break;
        default:
            Start_timerEx(SEND_PLATFORM_EVT, DATA_WAIT_TIME);//允许CDMA向平台发送数据
            break;
        }
        break;
    //电信 cdma 结束------------------------------------

    //移动 gprs	开始====================================
    case 0x02:
        switch (factory_gateway_set[1])   //协议类型=1 TCP/IP;2 MQTT; 3 SDK; 4 其它；
        {
        //移动TCP/IP
        case 0x01:
            cdma_tcp_send();//包含移动TCP/IP发送数据 gprs_tcp_send
            break;
        //移动MQTT
        case 0x02:
            cdma_mqtt_send();//包含移动mqtt发送数据 gprs_tcp_send
            break;
        //移动SDK
        case 0x03:
            cdma_sdk_send();//包含移动sdk发送数据 gprs_tcp_send
            break;
        //移动other
        case 0x04:
            gprs_other_send();
            break;
        default:
            Start_timerEx(SEND_PLATFORM_EVT, DATA_WAIT_TIME);//允许CDMA向平台发送数据
            break;
        }
        break;
    //移动 gprs	结束====================================

    //以太网 开始++++++++++++++++++++++++++++++++++++++
    case 0x03:
        switch (factory_gateway_set[1])   //协议类型=1 TCP/IP;2 MQTT; 3 SDK; 4 其它；
        {
        //电信TCP/IP
        case 0x01:
            Ethernet_tcp_send();
            break;
        //电信MQTT
        case 0x02:
            Ethernet_mqtt_send();
            break;
        //电信SDK
        case 0x03:
            Ethernet_sdk_send();
            break;
        //电信other
        case 0x04:
            Ethernet_other_send();
            break;
        default:
            Start_timerEx(SEND_PLATFORM_EVT, DATA_WAIT_TIME);//允许CDMA向平台发送数据
            break;
        }
        break;
    //以太网 结束++++++++++++++++++++++++++++++++++++++++

    //WiFi 开始&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    case 0x04:
        halt_RxReport2++;
        Start_timerEx(SEND_PLATFORM_EVT, DATA_WAIT_TIME);//允许CDMA向平台发送数据
        break;

    //WiFi 结束&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    //USB  开始########################################
    case 0x05:
        halt_RxReport2++;
        Start_timerEx(SEND_PLATFORM_EVT, DATA_WAIT_TIME);//允许CDMA向平台发送数据
        break;
    //USB 结束#########################################
    default:
        halt_RxReport2++;
        Start_timerEx(SEND_PLATFORM_EVT, DATA_WAIT_TIME);//允许CDMA向平台发送数据
        break;
    }
}

/*向平台发送函数--------------------------------------------------------------------------------------------------------*/
/*采集器地址说明：
0x0100-0x011F 0#采集器，0x0120-0x013F 1#采集器，。。。
0x0100-0x01E0 0#-7#采集器；0x0200-0x02E0 8#-15#采集器；0x0300-0x03E0 16#-23#采集器；0x0400-0x04E0 24#-31#采集器；
0x0500-0x05E0 32#-39#控制采集器；0x0600-0x06E0 40#-47#控制采集器；0x0700-0x07E0 40#-47#控制采集器；0x0800-0x08E0 48#-63#控制采集器；
0x0900-0x091F 网关采集器；0x0920-0x0923 断电指示上报
0x0940-0xA3F 网关采集器；0x0940-0xA3F  GPS上报
0x0A40-      网关采集器；0x0A40- 水肥上报
*/

static void cdma_tcp_send(void)   //电信TCP/IP发送数据；最长时间600秒上报一次数据（采集器个数+2*控制器个数）*6秒
{
    u16 reportCrcValue = 0;
    if (send_mess)  	//正在发送短信上报时，等待2s后重新执行
    {
        Start_timerEx(SEND_PLATFORM_EVT, 2000);
        return;
    }

    //	if(send_flg ==0x01||send_flg ==0x04||send_flg ==0x05) //对子站初始化工作完成ZZ_Wired_flag[65]=1；||ZZ_Wireles_flag[65]==1
    // 	{
    //		Start_timerEx(SEND_PLATFORM_EVT,500);
    //		return;
    //	}
    if (send_message_type != 0x03 && send_message_type != 0x00)   //对子站初始化工作完成ZZ_Wired_flag[65]=1；||ZZ_Wireles_flag[65]==1
    {
        Start_timerEx(SEND_PLATFORM_EVT, 500);
        return;
    }
    send_message_type = 0x03;	//在收到GSM的ERROR报告时使用，0x03为向平台上报数据

    /*send_message_type记录向cdma发送命令大类型，0x01为初始化程序，0x02为向平台返回控制命令，0x03为向平台上报数据。send_flg记录
    向cdma发送命令的小类型，0x01为初始化程序的at指令和data指令；0x02为平台上报数据的at指令，0x03为平台上报数据的data指令
    0x04为平台返回控制的at指令，0x05为平台返回控制的data指令；需要向平台发指令就一定有回复,
    在接收程序中用next_atdata_flg控制程序执行顺序，不向平台发指令的，就用module_send_flg控制程序执行。
    */

    switch (module_send_flg)   //0x0100~0x04FF采集器检测参数;0x500~0x08FF控制器检测参数;0x0900~0x091F网关检测参数;0x0920上电指示
    {
    case 0x01:
        send_message_len1 = 6 + 18 + 4 + 3;//6+18+实际字节数值+3
        bytelen_to_asc((unsigned char *)message_len_char1, send_message_len1);//send_message_len全局变量
        if (factory_gateway_set[0] == 1)send_at_cdma((unsigned char *)message_len_char1, 21);//TCB,CDMA	长度为31=6+信息长度=6+25=31；CDMA的AT指令长度为21；GPRS的AT指令长度为15
        if (factory_gateway_set[0] == 2)send_at_gprs((unsigned char *)message_len_char1, 15);//TCB,GPRS
        next_atdata_flg = 0x02;
        send_flg = 0x02; //表示向平台上报的at指令正在执行，下面只能执行data指令
        Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
        break;
    case 0x02:
        real_send[4] = 0x19;//信息长度25,18+实际字节数值+3=18+real_send[22]+3=18+4+3=25
        real_send[20] = 0x20;//0x0920上电指示平台地址低字节;0x0100~0x04FF采集器检测参数
        real_send[21] = 0x09;//0x0920上电指示平台地址高字节

        real_send[22] = 0x04;//实际发送字节数低字节
        real_send[23] = 0x00;//实际发送字节数高字节

        real_send[24] = 0x00;
        real_send[25] = 0x00;
        real_send[26] = 0x01;
        real_send[27] = 0x00;//发送的字节，每个参数4字节，前2个字节均为0；后2个字节为有效值，低字节在前

        reportCrcValue = GetCRC16(real_send + 11, 17);//8(采控器ID)+1(命令)+2(起始地址)+2(字节数长度)=13+4(字节值)=17

        real_send[28] = reportCrcValue & 0x00FF;
        real_send[29] = (reportCrcValue & 0xFF00) >> 8;
        real_send[30] = 0xDD;
        send_message_len1 = 6 + 18 + real_send[22] + 3;//6+18+实际字节数值+3
        bytelen_to_asc((unsigned char *)message_len_char1, send_message_len1);//send_message_len全局变量
        send_data_module(real_send, (unsigned char *)message_len_char1);//TCB
        next_atdata_flg = 0x03;
        send_flg = 0x03;//表示向平台上报的data指令正在执行
        Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
        break;
    case 0x03:
        module_send_flg = 0x04;
        send_flg = 0x00;
        send_message_type = 0x00;
        real_send[4] = 0x25;//信息长度37；18+实际字节数值+3=18+real_send[22]+3=18+16+3=37=0x25
        real_send[20] = 0x00;//平台地址低字节
        real_send[21] = 0x21;//平台地址高字节
        real_send[22] = 0x10;//实际发送字节数低字节,2*2*4(控制点)；1个控制器为一组上报
        real_send[23] = 0x00;//实际发送字节数高字节
        sendnum_mflg = 0x00;
        Start_timerEx(SEND_PLATFORM_EVT, INTER_SEND);
        break;

    //控制器开关状态上报开始
    case 0x04:  //module_send_flg
        //网关系统初始化完成后，控制器子站命令不存在，同时控制器子站ID小于32个（0~31）。下面的发送子站命令跳过;
        while ((ZZ_Wireles_flag[32 + sendnum_mflg] == 0) && (ZZ_Wired_flag[32 + sendnum_mflg] == 0) && (sendnum_mflg < 32))   //CSH_Wired_finish==0|
        {
            sendnum_mflg++;
        }
        if (sendnum_mflg < 32)
        {
            if (send_flg == 0x00)
            {
                //send_at_cdma("094",21);//MQTT长度为94，TCP长度为75（+19个字节），发送主题长度不能变，如果增加，则加相应的字节数
                send_message_len1 = 6 + 18 + real_send[22] + 3;//6+18+实际字节数值+3
                bytelen_to_asc((unsigned char *)message_len_char1, send_message_len1);//send_message_len全局变量
                if (factory_gateway_set[0] == 1)send_at_cdma((unsigned char *)message_len_char1, 21);//TCB,CDMA	长度为31=6+信息长度=6+25=31；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                if (factory_gateway_set[0] == 2)send_at_gprs((unsigned char *)message_len_char1, 15);//TCB,GPRS  TCP 长度为43=6+信息长度=6+37=43；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                send_flg = 0x02;
                next_atdata_flg = 0x04;
                Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
                break;
            }
            if (send_flg == 0x02)
            {
                real_send[20] = 0x00;//平台地址低字节
                real_send[21] = 0x21;//平台地址高字节
                alter_send(0x10, sendnum_mflg);//计算每组控制器上报状态数据的开始地址,修改了real_send的数据地址(2100H,2130H,2160H,...)
                //mqtt_publish(real_send,75);//MQTT 实际发送75个字节,这里的75仅是为了程序可读，没有用。
                //send_data_module(mqtt_real_send,"094");
                send_message_len1 = 6 + 18 + real_send[22] + 3;//6+18+实际字节数值+3
                bytelen_to_asc((unsigned char *)message_len_char1, send_message_len1);//send_message_len全局变量
                send_data_module(real_send, (unsigned char *)message_len_char1);//TCB长度为43=6+信息长度=6+37=43；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                next_atdata_flg = 0x04;
                send_flg = 0x03;
                Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
                break;
            }
            if (send_flg == 0x03)
            {
                module_send_flg = 0x4;//1组数据上报完成，等待5S进入下1组数据上报
                send_flg = 0x00;
                send_message_type = 0x00;
                sendnum_mflg++;
                Start_timerEx(SEND_PLATFORM_EVT, INTER_SEND);
                break;
            }
        }
        else
        {
            module_send_flg = 0x05;
            next_atdata_flg = 0x05;
            send_flg = 0x00;
            send_message_type = 0x00;
            real_send[4] = 0x35;//信息长度45；18+实际字节数值+3=18+real_send[22]+3=18+24+3=45=0x2D
            real_send[20] = 0x00;//平台地址低字节
            real_send[21] = 0x01;//平台地址高字节
            real_send[22] = 0x20;//实际发送字节数低字节2*2*(6+2)(参数个数)=32=0x20
            real_send[23] = 0x00;//实际发送字节数高字节
            sendnum_mflg = 0x00;
            Start_timerEx(SEND_PLATFORM_EVT, INTER_MESS);
            break;
        }

    /*控制器开关状态数据上报结束*/

    //采集器数据、控制器数据及网关采集数据上报开始
    case 0x05:  //module_send_flg
        //网关系统初始化完成后，采集器和控制器子站不存在，同时控制器子站ID小于等于64个（0~65）。下面的发送子站命令跳过;

        while ((ZZ_Wireles_flag[sendnum_mflg] == 0) && (ZZ_Wired_flag[sendnum_mflg] == 0) && (sendnum_mflg <= 64))   //CSH_Wired_finish==0|
        {
            sendnum_mflg++;
        }
        if (sendnum_mflg <= 64)   //该数据组是否已发送完成，没有发送完，继续发送.，sendnum_mflg是记录第几个子站上报
        {
            if (send_flg == 0x00)
            {
                send_message_len1 = 6 + 18 + real_send[22] + 3;//6+18+实际字节数值+3
                bytelen_to_asc((unsigned char *)message_len_char1, send_message_len1);//send_message_len全局变量
                //send_at_cdma("094",21);//MQTT长度为94，TCP长度为75（+19个字节），发送主题长度不能变，如果增加，则加相应的字节数(最长为256)
                if (factory_gateway_set[0] == 1)send_at_cdma((unsigned char *)message_len_char1, 21);//TCB,CDMA	长度为31=6+信息长度=6+25=31；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                if (factory_gateway_set[0] == 2)send_at_gprs((unsigned char *)message_len_char1, 15);//TCB,GPRS  TCB 长度=6+信息长度=6+45=51；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                send_flg = 0x02;
                next_atdata_flg = 0x05;
                Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
                break;
            }
            if (send_flg == 0x02)
            {
                real_send[20] = 0x00;//平台地址低字节
                real_send[21] = 0x01;//平台地址高字节
                chge_coltsnd(sendnum_mflg);//计算每组采集器上报数据的开始地址,修改了real_send的数据地址(0100H,0130H,0160H,...)
                send_message_len1 = 6 + 18 + real_send[22] + 3;//6+18+实际字节数值+3
                bytelen_to_asc((unsigned char *)message_len_char1, send_message_len1);//send_message_len全局变量
                //mqtt_publish(real_send,75);//MQTT 实际发送75个字节,这里的75仅是为了程序可读，没有用.
                //send_data_module(mqtt_real_send,"094");
                send_data_module(real_send, (unsigned char *)message_len_char1);//TCB长度=6+信息长度=6+2*4+45=51+8；
                next_atdata_flg = 0x5;
                send_flg = 0x03;
                Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
                break;
            }
            if (send_flg == 0x03)
            {
                module_send_flg = 0x5;//1组数据上报完成，等待5S进入下1组数据上报
                send_flg = 0x00;
                send_message_type = 0x00;
                sendnum_mflg++;
                Start_timerEx(SEND_PLATFORM_EVT, INTER_SEND);
                break;
            }
        }
        else
        {
            module_send_flg = 0x06;
            send_flg = 0x00;
            send_message_type = 0x00;
            real_send[4] = 0x19;//信息长度25；18+实际字节数值+3=18+real_send[22]+3=18+4+3=25=0x19
            real_send[20] = 0x00;//平台地址低字节
            real_send[21] = 0x25;//平台地址高字节
            real_send[22] = 0x04;//实际发送字节数低字节,2*2*1(自控回路)；1个自控回路为一组上报
            real_send[23] = 0x00;//实际发送字节数高字节
            sendnum_mflg = 0x00;
            Start_timerEx(SEND_PLATFORM_EVT, INTER_SEND);
            break;
        }
    //网关采集数据上报结束

    //自控状态上报开始
    case 0x06:  //module_send_flg
        //网关系统初始化完成后，控制器子站命令不存在，同时控制器子站ID小于32个（0~31）。下面的发送子站命令跳过;
        while ((hand_auto_count[sendnum_mflg] == 0) && (sendnum_mflg < 73))   //CSH_Wired_finish==0|
        {
            sendnum_mflg++;
        }
        if (sendnum_mflg < 73)
        {
            if (send_flg == 0x00)
            {
                //send_at_cdma("094",21);//MQTT长度为94，TCP长度为75（+19个字节），发送主题长度不能变，如果增加，则加相应的字节数
                send_message_len1 = 6 + 18 + real_send[22] + 3;//6+18+实际字节数值+3
                bytelen_to_asc((unsigned char *)message_len_char1, send_message_len1);//send_message_len全局变量
                if (factory_gateway_set[0] == 1)send_at_cdma((unsigned char *)message_len_char1, 21);//TCB,CDMA	长度为31=6+信息长度=6+25=31；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                if (factory_gateway_set[0] == 2)send_at_gprs((unsigned char *)message_len_char1, 15);//TCB,GPRS  TCP 长度为43=6+信息长度=6+37=43；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                send_flg = 0x02;
                next_atdata_flg = 0x06;
                Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
                break;
            }
            if (send_flg == 0x02)
            {
                real_send[20] = 0x00;//平台地址低字节
                real_send[21] = 0x25;//平台地址高字节
                alter_hand_auto(0x04, sendnum_mflg);//计算每组自控回路上报状态数据的开始地址,修改了real_send的数据地址(2500H,2504H,2508H,...)
                //mqtt_publish(real_send,75);//MQTT 实际发送75个字节,这里的75仅是为了程序可读，没有用。
                //send_data_module(mqtt_real_send,"094");
                send_message_len1 = 6 + 18 + real_send[22] + 3;//6+18+实际字节数值+3
                bytelen_to_asc((unsigned char *)message_len_char1, send_message_len1);//send_message_len全局变量
                send_data_module(real_send, (unsigned char *)message_len_char1);//TCB长度为43=6+信息长度=6+37=43；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                next_atdata_flg = 0x6;
                send_flg = 0x03;
                Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
                break;
            }
            if (send_flg == 0x03)
            {
                module_send_flg = 0x6;//1组数据上报完成，等待5S进入下1组数据上报
                send_flg = 0x00;
                send_message_type = 0x00;
                if (hand_auto_count[sendnum_mflg] >= 1)
                {
                    hand_auto_count[sendnum_mflg]--;
                }
                sendnum_mflg++;
                Start_timerEx(SEND_PLATFORM_EVT, INTER_MESS);
                break;
            }
        }
        else
        {
            if (fertigation_flg == 1)
            {
                module_send_flg = 0x07;
                next_atdata_flg = 0x07;
            }
            else
            {
                module_send_flg = 0x01;   //取消延时时间，6秒发一次数据；如果需要延时时间为0xFF
            }

            send_flg = 0x00;
            send_message_type = 0x00;
            real_send[4] = 0x35;//信息长度45；18+实际字节数值+3=18+real_send[22]+3=18+24+3=45=0x2D
            real_send[20] = 0x40;//平台地址低字节
            real_send[21] = 0x0A;//平台地址高字节
            real_send[22] = 0x20;//实际发送字节数低字节2*2*(6+2)(参数个数)=32=0x20
            real_send[23] = 0x00;//实际发送字节数高字节
            sendnum_mflg = 0x00;
            Start_timerEx(SEND_PLATFORM_EVT, INTER_MESS);
            break;
        }
    //水肥参数上报
    case 0x07:  //module_send_flg
        //子站0x52 浮点数放在u8 fertigation52.param_int[6][4] 整型数放在 u8 param_fertigation52[36]中有18字节数据
        //起始平台地址为0x0A40 u8 reportfertigation52[3][16] 三组数据 每组16字节，8个参数
        //u8 fertigation52databuf[12]里存放着浮点数转u16再转2*u8的数据，保留小数点后两位，要除以100使用

        memcpy(reportfertigation52[0], fertigation52databuf, 12);
        memcpy(reportfertigation52[1], param_fertigation52, 16);
        memcpy(reportfertigation52[2], param_fertigation52 + 17, 16); //将浮点数和整型数放到reportfertigation52[3][16]中
        //总共48个参数 浮点数6*2 + 整型数18*2 =48

        if (sendnum_mflg < 3)
        {
            if (send_flg == 0x00)
            {
                //send_at_cdma("094",21);//MQTT长度为94，TCP长度为75（+19个字节），发送主题长度不能变，如果增加，则加相应的字节数
                send_message_len1 = 6 + 18 + real_send[22] + 3;//6+18+实际字节数值+3
                bytelen_to_asc((unsigned char *)message_len_char1, send_message_len1);//send_message_len全局变量
                if (factory_gateway_set[0] == 1)send_at_cdma((unsigned char *)message_len_char1, 21);//TCB,CDMA	长度为31=6+信息长度=6+25=31；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                if (factory_gateway_set[0] == 2)send_at_gprs((unsigned char *)message_len_char1, 15);//TCB,GPRS  TCP 长度为43=6+信息长度=6+37=43；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                send_flg = 0x02;
                next_atdata_flg = 0x07;
                Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
                break;
            }
            if (send_flg == 0x02)
            {
                real_send[20] = 0x40;//平台地址低字节
                real_send[21] = 0x0A;//平台地址高字节
                chge_fertisnd(sendnum_mflg);//计算每组自控回路上报状态数据的开始地址,修改了real_send的数据地址(2500H,2504H,2508H,...)

                //mqtt_publish(real_send,75);//MQTT 实际发送75个字节,这里的75仅是为了程序可读，没有用。
                //send_data_module(mqtt_real_send,"094");
                send_message_len1 = 6 + 18 + real_send[22] + 3;//6+18+实际字节数值+3
                bytelen_to_asc((unsigned char *)message_len_char1, send_message_len1);//send_message_len全局变量
                send_data_module(real_send, (unsigned char *)message_len_char1);//TCB长度为43=6+信息长度=6+37=43；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                next_atdata_flg = 0x7;
                send_flg = 0x03;
                Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
                break;
            }
            if (send_flg == 0x03)
            {
                module_send_flg = 0x7;//1组数据上报完成，等待5S进入下1组数据上报
                send_flg = 0x00;
                send_message_type = 0x00;
                sendnum_mflg++;
                Start_timerEx(SEND_PLATFORM_EVT, INTER_SEND);
                break;
            }
        }
        else
        {
            module_send_flg = 0x01;//取消延时时间，6秒发一次数据；如果需要延时时间为0xFF
            send_flg = 0x00;
            send_message_type = 0x00;
            sendnum_mflg = 0x00;
            Start_timerEx(SEND_PLATFORM_EVT, INTER_MESS);
            break;
        }

    /*水肥上报结束*/
    default:
        Start_timerEx(SEND_PLATFORM_EVT, DATA_WAIT_TIME);//允许CDMA向平台发送数据
        break;
    }
}

static void cdma_mqtt_send(void)   //电信mqtt发送数据
{
    u16 reportCrcValue = 0;
    if (send_mess)  	//正在发送短信上报时，等待2s后重新执行
    {
        Start_timerEx(SEND_PLATFORM_EVT, 2000);
        return;
    }

    //	if(send_flg ==0x01||send_flg ==0x04||send_flg ==0x05) //对子站初始化工作完成Wired_Wireles_flag[65]=1；||Wired_Wireles_flag[65]==0
    // 	{
    //		Start_timerEx(SEND_PLATFORM_EVT,500);
    //		return;
    //	}
    if (send_message_type != 0x03 && send_message_type != 0x00)   //对子站初始化工作完成ZZ_Wired_flag[65]=1；||ZZ_Wireles_flag[65]==1
    {
        Start_timerEx(SEND_PLATFORM_EVT, 500);
        return;
    }
    send_message_type = 0x03;	//在收到GSM的ERROR报告时使用，0x03为向平台上报数据

    /*send_message_type记录向cdma发送命令大类型，0x01为初始化程序，0x02为向平台返回控制命令，0x03为向平台上报数据。send_flg记录
    向cdma发送命令的小类型，0x01为初始化程序的at指令和data指令；0x02为平台上报数据的at指令，0x03为平台上报数据的data指令
    0x04为平台返回控制的at指令，0x05为平台返回控制的data指令；需要向平台发指令就一定有回复,
    在接收程序中用next_atdata_flg控制程序执行顺序，不向平台发指令的，就用module_send_flg控制程序执行。
    */

    switch (module_send_flg)   //0x0100~0x04FF采集器检测参数;0x500~0x08FF控制器检测参数;0x0900~0x091F网关检测参数;0x0920上电指示
    {
    case 0x01:
        real_send[4] = 0x19;//信息长度25,18+实际字节数值+3=18+real_send[22]+3=18+4+3=25
        real_send[20] = 0x20;//0x0920上电指示平台地址低字节;0x0100~0x04FF采集器检测参数
        real_send[21] = 0x09;//0x0920上电指示平台地址高字节

        real_send[22] = 0x04;//实际发送字节数低字节
        real_send[23] = 0x00;//实际发送字节数高字节

        real_send[24] = 0x00;
        real_send[25] = 0x00;
        real_send[26] = 0x01;
        real_send[27] = 0x00;//发送的字节，每个参数4字节，前2个字节均为0；后2个字节为有效值，低字节在前
        reportCrcValue = GetCRC16(real_send + 11, 17);//8(采控器ID)+1(命令)+2(起始地址)+2(字节数长度)=13+4(字节值)=17
        real_send[28] = reportCrcValue & 0x00FF;
        real_send[29] = (reportCrcValue & 0xFF00) >> 8;
        real_send[30] = 0xDD;
        send_message_len1 = 6 + 18 + real_send[22] + 3;//6+18+实际字节数值+3
        mqtt_publish(real_send, send_message_len1);//MQTT 实际发送75个字节(不包括推送主题)。
        bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);//mqtt_len全局变量；在mqtt_publish函数中赋值
        if (factory_gateway_set[0] == 1)send_at_cdma((unsigned char *)message_len_char1, 21);//TCB,CDMA	长度为31=6+信息长度=6+25=31；CDMA的AT指令长度为21；GPRS的AT指令长度为15
        if (factory_gateway_set[0] == 2)send_at_gprs((unsigned char *)message_len_char1, 15);//TCB,GPRS  TCP 长度为43=6+信息长度=6+37=43；CDMA的AT指令长度为21；GPRS的AT指令长度为15
        next_atdata_flg = 0x02;
        send_flg = 0x02; //表示向平台上报的at指令正在执行，下面只能执行data指令
        Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
        break;
    case 0x02:
        send_message_len1 = 6 + 18 + real_send[22] + 3;//6+18+实际字节数值+3
        mqtt_publish(real_send, send_message_len1);//MQTT 实际发送75个字节(不包括推送主题)。
        bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);//send_message_len全局变量
        send_data_module(ReportData2, (unsigned char *)message_len_char1);
        //		  send_data_module(real_send,(unsigned char *)message_len_char);//TCB
        next_atdata_flg = 0x03;
        send_flg = 0x03;//表示向平台上报的data指令正在执行
        Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
        break;
    case 0x03:
        module_send_flg = 0x04;
        send_flg = 0x00;
        send_message_type = 0x00;
        real_send[4] = 0x25;//信息长度37；18+实际字节数值+3=18+real_send[22]+3=18+16+3=37=0x25
        real_send[20] = 0x00;//平台地址低字节
        real_send[21] = 0x21;//平台地址高字节
        real_send[22] = 0x10;//实际发送字节数低字节,2*2*4(控制点)；1个控制器为一组上报
        real_send[23] = 0x00;//实际发送字节数高字节
        sendnum_mflg = 0x00;
        Start_timerEx(SEND_PLATFORM_EVT, INTER_SEND);
        break;

    //控制器开关状态数据上报开始
    case 0x04:  //module_send_flg
        //网关系统初始化完成后，控制器子站命令不存在，同时控制器子站ID小于32个（0~31）。下面的发送子站命令跳过;
        while ((ZZ_Wireles_flag[32 + sendnum_mflg] == 0) && (ZZ_Wired_flag[32 + sendnum_mflg] == 0) && (sendnum_mflg < 32))   //CSH_Wired_finish==0|
        {
            sendnum_mflg++;
        }
        if (sendnum_mflg < 32)
        {
            if (send_flg == 0x00)
            {
                send_message_len1 = 6 + 18 + real_send[22] + 3;//6+18+实际字节数值+3
                mqtt_publish(real_send, send_message_len1);//MQTT 实际发送75个字节(不包括推送主题)。
                bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);//send_message_len全局变量
                if (factory_gateway_set[0] == 1)send_at_cdma((unsigned char *)message_len_char1, 21);//TCB,CDMA	长度为31=6+信息长度=6+25=31；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                if (factory_gateway_set[0] == 2)send_at_gprs((unsigned char *)message_len_char1, 15);//TCB,GPRS  TCP 长度为43=6+信息长度=6+37=43；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                send_flg = 0x02;
                next_atdata_flg = 0x04;
                Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
                break;
            }
            if (send_flg == 0x02)
            {
                real_send[20] = 0x00;//平台地址低字节
                real_send[21] = 0x21;//平台地址高字节
                alter_send(0x10, sendnum_mflg);//计算每组控制器上报状态数据的开始地址,修改了real_send的数据地址(2100H,2130H,2160H,...)
                send_message_len1 = 6 + 18 + real_send[22] + 3;//6+18+实际字节数值+3
                mqtt_publish(real_send, send_message_len1);//MQTT 实际发送75个字节(不包括推送主题)。
                bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);//send_message_len全局变量
                send_data_module(ReportData2, (unsigned char *)message_len_char1);
                //	    send_data_module(real_send,(unsigned char *)message_len_char);//TCB长度为43=6+信息长度=6+37=43；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                next_atdata_flg = 0x04;
                send_flg = 0x03;
                Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
                break;
            }
            if (send_flg == 0x03)
            {
                module_send_flg = 0x4;//1组数据上报完成，等待5S进入下1组数据上报
                send_flg = 0x00;
                send_message_type = 0x00;
                sendnum_mflg++;
                Start_timerEx(SEND_PLATFORM_EVT, INTER_SEND);
                break;
            }
        }
        else
        {
            module_send_flg = 0x5;
            next_atdata_flg = 0x05;
            send_flg = 0x00;
            send_message_type = 0x00;
            real_send[4] = 0x35;//信息长度45；18+实际字节数值+3=18+real_send[22]+3=18+24+3=45=0x2D
            real_send[20] = 0x00;//平台地址低字节
            real_send[21] = 0x01;//平台地址高字节
            real_send[22] = 0x20;//实际发送字节数低字节2*2*(6+2)(参数个数)=32=0x20
            real_send[23] = 0x00;//实际发送字节数高字节
            sendnum_mflg = 0x00;
            Start_timerEx(SEND_PLATFORM_EVT, INTER_MESS);
            break;
        }

    /*控制器开关状态数据上报结束*/

    //采集器数据、控制器数据及网关采集数据上报开始
    case 0x05:  //module_send_flg
        //网关系统初始化完成后，采集器和控制器子站命令不存在，同时控制器子站ID小于等于64个（0~65）。下面的发送子站命令跳过;

        while ((ZZ_Wireles_flag[sendnum_mflg] == 0) && (ZZ_Wired_flag[sendnum_mflg] == 0) && (sendnum_mflg <= 64))   //CSH_Wired_finish==0|
        {
            sendnum_mflg++;
        }
        if (sendnum_mflg <= 64)   //该数据组是否已发送完成，没有发送完，继续发送.，sendnum_mflg是记录第几个子站上报
        {
            if (send_flg == 0x00)
            {
                send_message_len1 = 6 + 18 + real_send[22] + 3;//6+18+实际字节数值+3
                mqtt_publish(real_send, send_message_len1);//MQTT 实际发送75个字节(不包括推送主题)。
                bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);//send_message_len全局变量
                if (factory_gateway_set[0] == 1)send_at_cdma((unsigned char *)message_len_char1, 21);//TCB,CDMA	长度为31=6+信息长度=6+25=31；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                if (factory_gateway_set[0] == 2)send_at_gprs((unsigned char *)message_len_char1, 15);//TCB,GPRS  TCP 长度为43=6+信息长度=6+37=43；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                send_flg = 0x02;
                next_atdata_flg = 0x05;
                Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
                break;
            }
            if (send_flg == 0x02)
            {
                real_send[20] = 0x00;//平台地址低字节
                real_send[21] = 0x01;//平台地址高字节
                chge_coltsnd(sendnum_mflg);//计算每组采集器上报数据的开始地址,修改了real_send的数据地址(0100H,0130H,0160H,...)
                send_message_len1 = 6 + 18 + real_send[22] + 3;//6+18+实际字节数值+3
                mqtt_publish(real_send, send_message_len1);//MQTT 实际发送75个字节(不包括推送主题)。
                bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);//send_message_len全局变量
                send_data_module(ReportData2, (unsigned char *)message_len_char1);
                //	    send_data_module(real_send,(unsigned char *)message_len_char);//TCB长度=6+信息长度=6+2*4+45=51+8；
                next_atdata_flg = 0x5;
                send_flg = 0x03;
                Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
                break;
            }
            if (send_flg == 0x03)
            {
                module_send_flg = 0x5;//1组数据上报完成，等待5S进入下1组数据上报
                send_flg = 0x00;
                send_message_type = 0x00;
                sendnum_mflg++;
                Start_timerEx(SEND_PLATFORM_EVT, INTER_SEND);
                break;
            }
        }
        else
        {
            module_send_flg = 0x06;
            send_flg = 0x00;
            send_message_type = 0x00;
            real_send[4] = 0x19;//信息长度25；18+实际字节数值+3=18+real_send[22]+3=18+4+3=25=0x19
            real_send[20] = 0x00;//平台地址低字节
            real_send[21] = 0x25;//平台地址高字节
            real_send[22] = 0x04;//实际发送字节数低字节,2*2*4(控制点)；1个控制器为一组上报
            real_send[23] = 0x00;//实际发送字节数高字节
            sendnum_mflg = 0x00;
            Start_timerEx(SEND_PLATFORM_EVT, INTER_SEND);
            break;
        }
    //网关采集数据上报结束
    //自控回路手动-自动状态上报开始
    case 0x06:  //module_send_flg
        //网关系统初始化完成后，控制器子站命令不存在，同时控制器子站ID小于32个（0~31）。下面的发送子站命令跳过;
        while ((hand_auto_count[sendnum_mflg] == 0) && (sendnum_mflg < 73))   //CSH_Wired_finish==0|
        {
            sendnum_mflg++;
        }
        if (sendnum_mflg < 73)
        {
            if (send_flg == 0x00)
            {
                send_message_len1 = 6 + 18 + real_send[22] + 3;//6+18+实际字节数值+3
                mqtt_publish(real_send, send_message_len1);//MQTT 实际发送75个字节(不包括推送主题)。
                bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);//send_message_len全局变量
                if (factory_gateway_set[0] == 1)send_at_cdma((unsigned char *)message_len_char1, 21);//TCB,CDMA	长度为31=6+信息长度=6+25=31；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                if (factory_gateway_set[0] == 2)send_at_gprs((unsigned char *)message_len_char1, 15);//TCB,GPRS  TCP 长度为43=6+信息长度=6+37=43；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                send_flg = 0x02;
                next_atdata_flg = 0x06;
                Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
                break;
            }
            if (send_flg == 0x02)
            {
                real_send[20] = 0x00;//平台地址低字节
                real_send[21] = 0x25;//平台地址高字节
                alter_hand_auto(0x04, sendnum_mflg);//计算每组控制器上报状态数据的开始地址,修改了real_send的数据地址(2100H,2130H,2160H,...)
                send_message_len1 = 6 + 18 + real_send[22] + 3;//6+18+实际字节数值+3
                mqtt_publish(real_send, send_message_len1);//MQTT 实际发送75个字节(不包括推送主题)。
                bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);//send_message_len全局变量
                send_data_module(ReportData2, (unsigned char *)message_len_char1);
                //	    send_data_module(real_send,(unsigned char *)message_len_char);//TCB长度为43=6+信息长度=6+37=43；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                next_atdata_flg = 0x6;
                send_flg = 0x03;
                Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
                break;
            }
            if (send_flg == 0x03)
            {
                module_send_flg = 0x6;//1组数据上报完成，等待5S进入下1组数据上报
                send_flg = 0x00;
                send_message_type = 0x00;
                if (hand_auto_count[sendnum_mflg] >= 1)
                {
                    hand_auto_count[sendnum_mflg]--;
                }
                sendnum_mflg++;
                Start_timerEx(SEND_PLATFORM_EVT, INTER_MESS);
                break;
            }
        }
        else
        {
            module_send_flg = 0x01;//取消延时时间，6秒发一次数据；如果需要延时时间为0xFF
            send_flg = 0x00;
            sendnum_mflg = 0x00;
            Start_timerEx(SEND_PLATFORM_EVT, INTER_MESS);
            break;
        }
    /*自控回路手动-自动状态上报结束*/
    default:
        Start_timerEx(SEND_PLATFORM_EVT, DATA_WAIT_TIME);//允许CDMA向平台发送数据
        break;
    }
}

static void cdma_sdk_send(void)   //电信sdk发送数据（极码平台）
{
    //	u16 reportCrcValue = 0;
    if (send_mess)  	//正在发送短信上报时，等待2s后重新执行
    {
        Start_timerEx(SEND_PLATFORM_EVT, 2000);
        return;
    }

    if (send_message_type != 0x03 && send_message_type != 0x00)   //对子站初始化工作完成ZZ_Wired_flag[65]=1；||ZZ_Wireles_flag[65]==1
    {
        Start_timerEx(SEND_PLATFORM_EVT, 500);
        return;
    }
    send_message_type = 0x03;	//在收到GSM的ERROR报告时使用，0x03为向平台上报数据

    /*send_message_type记录向cdma发送命令大类型，0x01为初始化程序，0x02为向平台返回控制命令，0x03为向平台上报数据。send_flg记录
    向cdma发送命令的小类型，0x01为初始化程序的at指令和data指令；0x02为平台上报数据的at指令，0x03为平台上报数据的data指令
    0x04为平台返回控制的at指令，0x05为平台返回控制的data指令；需要向平台发指令就一定有回复,
    在接收程序中用next_atdata_flg控制程序执行顺序，不向平台发指令的，就用module_send_flg控制程序执行。
    */

    switch (module_send_flg)   //0x0100~0x04FF采集器检测参数;0x500~0x08FF控制器检测参数;0x0900~0x091F网关检测参数;0x0920上电指示
    {
        u8 len, temp_date[2];
    case 0x01:
        temp_date[0] = 0x01;
        temp_date[1] = 0x00;
        len = made_keyX_value(0x0920, temp_date, 1, real_send);//0x0920上电指示地址，值0x0001,1一个参数，返回real_send[]={30393230:30303031};len=11个字节
        if (factory_gateway_set[1] == 4)len = made_keyX_value4(0x0920, temp_date, 1, real_send);
        mqtt_publish(real_send, len);
        bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);//mqtt_len为全局变量，在mqtt_publish11函数中赋值
        if (factory_gateway_set[0] == 1)send_at_cdma((unsigned char *)message_len_char1, 21);//TCB,CDMA	长度为31=6+信息长度=6+25=31；CDMA的AT指令长度为21；GPRS的AT指令长度为15
        if (factory_gateway_set[0] == 2)send_at_gprs((unsigned char *)message_len_char1, 15);//TCB,GPRS  TCP 长度为43=6+信息长度=6+37=43；CDMA的AT指令长度为21；GPRS的AT指令长度为15
        next_atdata_flg = 0x02;
        send_flg = 0x02; //表示向平台上报的at指令正在执行，下面只能执行data指令
        Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
        break;
    case 0x02:
        send_data_module(ReportData2, (unsigned char *)message_len_char1);//ReportData2为实际发送内容，在mqtt_publish函数中包装
        next_atdata_flg = 0x03;
        send_flg = 0x03;//表示向平台上报的data指令正在执行
        Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
        break;
    case 0x03:
        module_send_flg = 0x04;
        send_flg = 0x00;
        send_message_type = 0x00;
        sendnum_mflg = 0x00;
        Start_timerEx(SEND_PLATFORM_EVT, INTER_SEND);
        break;

    //控制器开关状态数据上报开始
    case 0x04:  //module_send_flg
        //网关系统初始化完成后，控制器子站命令不存在，同时控制器子站ID小于32个（0~31）。下面的发送子站命令跳过;
        while ((ZZ_Wireles_flag[32 + sendnum_mflg] == 0) && (ZZ_Wired_flag[32 + sendnum_mflg] == 0) && (sendnum_mflg < 32))   //CSH_Wired_finish==0|
        {
            sendnum_mflg++;
        }
        if (sendnum_mflg < 32)
        {
            if (send_flg == 0x00)
            {
                len = made_keyX_value(0x2100 + sendnum_mflg * 4 * 4, Controllers[sendnum_mflg], 4, real_send);//0x2100控制器控制状态地址，一次上报1个控制器4个状态参数
                if (factory_gateway_set[1] == 4)len = made_keyX_value4(0x2100 + sendnum_mflg * 4 * 4, Controllers[sendnum_mflg], 4, real_send);
                mqtt_publish(real_send, len);//该推送函数为SDK特别指定的，因为规定了控制器状态上报的推送主题
                bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);//mqtt_len为全局变量，在mqtt_publish11函数中赋值
                if (factory_gateway_set[0] == 1)send_at_cdma((unsigned char *)message_len_char1, 21);//TCB,CDMA	长度为31=6+信息长度=6+25=31；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                if (factory_gateway_set[0] == 2)send_at_gprs((unsigned char *)message_len_char1, 15);//TCB,GPRS  TCP 长度为43=6+信息长度=6+37=43；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                send_flg = 0x02;
                next_atdata_flg = 0x04;
                Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
                break;
            }
            if (send_flg == 0x02)
            {
                len = made_keyX_value(0x2100 + sendnum_mflg * 4 * 4, Controllers[sendnum_mflg], 4, real_send);//0x2100控制器控制状态地址，上报1个控制器4个状态参数
                if (factory_gateway_set[1] == 4)len = made_keyX_value4(0x2100 + sendnum_mflg * 4 * 4, Controllers[sendnum_mflg], 4, real_send);
                mqtt_publish(real_send, len);//该推送函数为SDK特别指定的，因为规定了控制器状态上报的推送主题
                bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);//mqtt_len为全局变量，在mqtt_publish11函数中赋值
                send_data_module(ReportData2, (unsigned char *)message_len_char1);//ReportData2为实际发送内容，在mqtt_publish函数中包装
                next_atdata_flg = 0x04;
                send_flg = 0x03;
                Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
                break;
            }
            if (send_flg == 0x03)
            {
                module_send_flg = 0x4;//1组数据上报完成，等待5S进入下1组数据上报
                send_flg = 0x00;
                send_message_type = 0x00;
                sendnum_mflg++;
                Start_timerEx(SEND_PLATFORM_EVT, INTER_SEND);
                break;
            }
        }
        else
        {
            module_send_flg = 0x5;
            next_atdata_flg = 0x05;
            send_flg = 0x00;
            send_message_type = 0x00;
            sendnum_mflg = 0x00;
            Start_timerEx(SEND_PLATFORM_EVT, INTER_MESS);
            break;
        }

    /*控制器开关状态数据上报结束*/

    //采集器数据、控制器数据及网关采集数据上报开始
    case 0x05:  //module_send_flg
        //网关系统初始化完成后，采集器和控制器子站命令不存在，同时控制器子站ID小于等于64个（0~65）。下面的发送子站命令跳过;

        while ((ZZ_Wireles_flag[sendnum_mflg] == 0) && (ZZ_Wired_flag[sendnum_mflg] == 0) && (sendnum_mflg <= 64))   //CSH_Wired_finish==0|
        {
            sendnum_mflg++;
        }
        if (sendnum_mflg <= 64)   //该数据组是否已发送完成，没有发送完，继续发送.，sendnum_mflg是记录第几个子站上报
        {
            if (send_flg == 0x00)
            {
                memset(real_send, 0, sizeof(real_send));
                len = made_keyX_valueF(0x0100 + sendnum_mflg * 4 * 8, Collectors[sendnum_mflg], zero_rang.k_b_float[sendnum_mflg], 4 * send_count, real_send);
                //0x0100为采集器和控制器采集数据地址，上报1个控制器8个采集参数；分2次上报，4*send_count为第2次上报的开始参数地址
                if (factory_gateway_set[1] == 4)len = made_keyX_valueF4(0x0100 + sendnum_mflg * 4 * 8, Collectors[sendnum_mflg], zero_rang.k_b_float[sendnum_mflg], 4 * send_count, real_send);
                mqtt_publish(real_send, len);//该推送函数为通用推送函数
                bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);//mqtt_len为全局变量，在mqtt_publish11函数中赋值
                send_flg = 0x02;
                if (factory_gateway_set[0] == 1)send_at_cdma((unsigned char *)message_len_char1, 21);//TCB,CDMA	长度为31=6+信息长度=6+25=31；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                if (factory_gateway_set[0] == 2)send_at_gprs((unsigned char *)message_len_char1, 15);//TCB,GPRS  TCP 长度为43=6+信息长度=6+37=43；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                next_atdata_flg = 0x05;
                Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
                break;
            }
            if (send_flg == 0x02)
            {
                len = made_keyX_valueF(0x0100 + sendnum_mflg * 4 * 8, Collectors[sendnum_mflg], zero_rang.k_b_float[sendnum_mflg], 4 * send_count, real_send);
                //0x0100为采集器和控制器采集数据地址，上报1个控制器8个采集参数
                if (factory_gateway_set[1] == 4)len = made_keyX_valueF4(0x0100 + sendnum_mflg * 4 * 8, Collectors[sendnum_mflg], zero_rang.k_b_float[sendnum_mflg], 4 * send_count, real_send);
                mqtt_publish(real_send, len);//该推送函数为通用推送函数
                bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);//mqtt_len为全局变量，在mqtt_publish11函数中赋值
                send_data_module(ReportData2, (unsigned char *)message_len_char1);//ReportData2为实际发送内容，在mqtt_publish函数中包装
                if (send_count == 0)
                {
                    send_count = 1;
                    module_send_flg = 0x5;
                    next_atdata_flg = 0x5;
                    send_flg = 0x00;
                    send_message_type = 0x00;
                    Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);//8个参数分2次连续上报；300ms
                    break;
                }
                else
                    send_count = 0;
                module_send_flg = 0x5;
                next_atdata_flg = 0x5;
                send_flg = 0x03;
                Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
                break;
            }

            if (send_flg == 0x03)
            {
                module_send_flg = 0x5;//1组数据上报完成，等待5S进入下1组数据上报
                send_flg = 0x00;
                send_message_type = 0x00;
                send_count = 0;
                sendnum_mflg++;
                Start_timerEx(SEND_PLATFORM_EVT, INTER_SEND);
                break;
            }
        }
        else
        {
            module_send_flg = 0x06;
            send_flg = 0x00;
            send_message_type = 0x00;
            sendnum_mflg = 0x00;
            Start_timerEx(SEND_PLATFORM_EVT, INTER_SEND);
            break;
        }
    //网关采集数据上报结束
    //自控回路手动-自动状态数据上报开始
    case 0x06:  //module_send_flg
        //网关系统初始化完成后，控制器子站命令不存在，同时控制器子站ID小于32个（0~31）。下面的发送子站命令跳过;
        while ((hand_auto_count[sendnum_mflg] == 0) && (sendnum_mflg < 73))   //CSH_Wired_finish==0|
        {
            sendnum_mflg++;
        }
        if (sendnum_mflg < 73)
        {
            if (send_flg == 0x00)
            {
                len = made_keyX_value(0x2500 + sendnum_mflg * 1 * 4, hand_auto_flg[sendnum_mflg], 1, real_send);//0x2500自控回路手自动状态地址，1次上报1个手动-自动状态
                if (factory_gateway_set[1] == 4)len = made_keyX_value4(0x2500 + sendnum_mflg * 1 * 4, hand_auto_flg[sendnum_mflg], 1, real_send);
                mqtt_publish(real_send, len);//该推送函数为SDK特别指定的，因为规定了控制器状态上报的推送主题
                bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);//mqtt_len为全局变量，在mqtt_publish11函数中赋值
                if (factory_gateway_set[0] == 1)send_at_cdma((unsigned char *)message_len_char1, 21);//TCB,CDMA	长度为31=6+信息长度=6+25=31；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                if (factory_gateway_set[0] == 2)send_at_gprs((unsigned char *)message_len_char1, 15);//TCB,GPRS  TCP 长度为43=6+信息长度=6+37=43；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                send_flg = 0x02;
                next_atdata_flg = 0x06;
                Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
                break;
            }
            if (send_flg == 0x02)
            {
                len = made_keyX_value(0x2500 + sendnum_mflg * 1 * 4, hand_auto_flg[sendnum_mflg], 1, real_send);//0x2500自控回路手自动状态地址，1次上报1个手动-自动状态
                if (factory_gateway_set[1] == 4)len = made_keyX_value4(0x2500 + sendnum_mflg * 1 * 4, hand_auto_flg[sendnum_mflg], 1, real_send);
                mqtt_publish(real_send, len);//该推送函数为SDK特别指定的，因为规定了控制器状态上报的推送主题
                bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);//mqtt_len为全局变量，在mqtt_publish11函数中赋值
                send_data_module(ReportData2, (unsigned char *)message_len_char1);//ReportData2为实际发送内容，在mqtt_publish函数中包装
                next_atdata_flg = 0x06;
                send_flg = 0x03;
                Start_timerEx(SEND_PLATFORM_EVT, CMD_WAIT_TIME);
                break;
            }
            if (send_flg == 0x03)
            {
                module_send_flg = 0x4;//1组数据上报完成，等待5S进入下1组数据上报
                send_flg = 0x00;
                send_message_type = 0x00;
                sendnum_mflg++;
                Start_timerEx(SEND_PLATFORM_EVT, INTER_SEND);
                break;
            }
        }
        else
        {
            module_send_flg = 0x01;//取消延时时间，6秒发一次数据；如果需要延时时间为0xFF
            send_flg = 0x00;
            send_message_type = 0x00;
            sendnum_mflg = 0x00;
            Start_timerEx(SEND_PLATFORM_EVT, INTER_MESS);
            break;
        }

    /*控制器开关状态数据上报结束*/
    default:
        Start_timerEx(SEND_PLATFORM_EVT, DATA_WAIT_TIME);//允许CDMA向平台发送数据
        break;
    }
}

//static void cdma_other_send(void) //电信other发送数据
//{
//}

static void gprs_other_send(void)   //移动other发送数据
{
    Start_timerEx(SEND_PLATFORM_EVT, DATA_WAIT_TIME);//允许CDMA向平台发送数据
}

static void Ethernet_tcp_send(void)   //电信TCP/IP发送数据
{
    u16 reportCrcValue = 0;
    if (send_mess)  	//正在发送短信上报时，等待2s后重新执行
    {
        Start_timerEx(SEND_PLATFORM_EVT, 2000);
        return;
    }

    if (send_message_type != 0x03 && send_message_type != 0x00)   //对子站初始化工作完成ZZ_Wired_flag[65]=1；||ZZ_Wireles_flag[65]==1
    {
        Start_timerEx(SEND_PLATFORM_EVT, 500);
        return;
    }
    send_message_type = 0x03;	//在收到GSM的ERROR报告时使用，0x03为向平台上报数据

    /*send_message_type记录向cdma发送命令大类型，0x01为初始化程序，0x02为向平台返回控制命令，0x03为向平台上报数据，0x04类型为快速回复极码平台的控制指令。send_flg记录
    向cdma发送命令的小类型，0x01为初始化程序的at指令和data指令；0x02为平台上报数据的at指令，0x03为平台上报数据的data指令
    0x04为平台返回控制的at指令，0x05为平台返回控制的data指令；需要向平台发指令就一定有回复,
    在接收程序中用next_atdata_flg控制程序执行顺序，不向平台发指令的，就用module_send_flg控制程序执行。
    */

    switch (module_send_flg)   //0x0100~0x04FF采集器检测参数;0x500~0x08FF控制器检测参数;0x0900~0x091F网关检测参数;0x0920上电指示
    {
    case 0x01:
        module_send_flg = 0x02;
        send_flg = 0x02; //表示向平台上报的at指令正在执行，下面只能执行data指令
        Start_timerEx(SEND_PLATFORM_EVT, Ethernet_WAIT_TIME);
        break;
    case 0x02:
        real_send[4] = 0x19;//信息长度25,18+实际字节数值+3=18+real_send[22]+3=18+4+3=25
        real_send[20] = 0x20;//0x0920上电指示平台地址低字节;0x0100~0x04FF采集器检测参数
        real_send[21] = 0x09;//0x0920上电指示平台地址高字节

        real_send[22] = 0x04;//实际发送字节数低字节
        real_send[23] = 0x00;//实际发送字节数高字节

        real_send[24] = 0x00;
        real_send[25] = 0x00;
        real_send[26] = 0x01;
        real_send[27] = 0x00;//发送的字节，每个参数4字节，前2个字节均为0；后2个字节为有效值，低字节在前

        reportCrcValue = GetCRC16(real_send + 11, 17);//8(采控器ID)+1(命令)+2(起始地址)+2(字节数长度)=13+4(字节值)=17

        real_send[28] = reportCrcValue & 0x00FF;
        real_send[29] = (reportCrcValue & 0xFF00) >> 8;
        real_send[30] = 0xDD;
        send_message_len1 = 6 + 18 + real_send[22] + 3;//6+18+实际字节数值+3
        bytelen_to_asc((unsigned char *)message_len_char1, send_message_len1);//send_message_len全局变量
        send_Ethernet_module(real_send, (unsigned char *)message_len_char1);//TCB
        module_send_flg = 0x03;
        send_flg = 0x03;//表示向平台上报的data指令正在执行
        Start_timerEx(SEND_PLATFORM_EVT, Ethernet_WAIT_TIME);
        break;
    case 0x03:
        module_send_flg = 0x04;
        send_flg = 0x00;
        send_message_type = 0x00;
        real_send[4] = 0x25;//信息长度37；18+实际字节数值+3=18+real_send[22]+3=18+16+3=37=0x25
        real_send[20] = 0x00;//平台地址低字节
        real_send[21] = 0x21;//平台地址高字节
        real_send[22] = 0x10;//实际发送字节数低字节,2*2*4(控制点)；1个控制器为一组上报
        real_send[23] = 0x00;//实际发送字节数高字节
        sendnum_mflg = 0x00;
        Start_timerEx(SEND_PLATFORM_EVT, INTER_SEND);
        break;

    //控制器开关状态上报开始
    case 0x04:  //module_send_flg
        //网关系统初始化完成后，控制器子站命令不存在，同时控制器子站ID小于32个（0~31）。下面的发送子站命令跳过;
        while ((ZZ_Wireles_flag[32 + sendnum_mflg] == 0) && (ZZ_Wired_flag[32 + sendnum_mflg] == 0) && (sendnum_mflg < 32))   //CSH_Wired_finish==0|
        {
            sendnum_mflg++;
        }
        if (sendnum_mflg < 32)
        {
            if (send_flg == 0x00)
            {
                send_flg = 0x02;
                module_send_flg = 0x04;
                Start_timerEx(SEND_PLATFORM_EVT, Ethernet_WAIT_TIME);
                break;
            }
            if (send_flg == 0x02)
            {
                real_send[20] = 0x00;//平台地址低字节
                real_send[21] = 0x21;//平台地址高字节
                alter_send(0x10, sendnum_mflg);//计算每组控制器上报状态数据的开始地址,修改了real_send的数据地址(2100H,2130H,2160H,...)
                //mqtt_publish(real_send,75);//MQTT 实际发送75个字节,这里的75仅是为了程序可读，没有用。
                //send_data_module(mqtt_real_send,"094");
                send_message_len1 = 6 + 18 + real_send[22] + 3;//6+18+实际字节数值+3
                bytelen_to_asc((unsigned char *)message_len_char1, send_message_len1);//send_message_len全局变量
                send_Ethernet_module(real_send, (unsigned char *)message_len_char1);//TCB长度为43=6+信息长度=6+37=43；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                module_send_flg = 0x4;
                send_flg = 0x03;
                Start_timerEx(SEND_PLATFORM_EVT, Ethernet_WAIT_TIME);
                break;
            }
            if (send_flg == 0x03)
            {
                module_send_flg = 0x4;//1组数据上报完成，等待5S进入下1组数据上报
                send_flg = 0x00;
                send_message_type = 0x00;
                sendnum_mflg++;
                Start_timerEx(SEND_PLATFORM_EVT, INTER_SEND);
                break;
            }
        }
        else
        {
            module_send_flg = 0x5;
            send_flg = 0x00;
            send_message_type = 0x00;
            real_send[4] = 0x35;//信息长度45；18+实际字节数值+3=18+real_send[22]+3=18+24+3=45=0x2D
            real_send[20] = 0x00;//平台地址低字节
            real_send[21] = 0x01;//平台地址高字节
            real_send[22] = 0x20;//实际发送字节数低字节2*2*(6+2)(参数个数)=32=0x20
            real_send[23] = 0x00;//实际发送字节数高字节
            sendnum_mflg = 0x00;
            Start_timerEx(SEND_PLATFORM_EVT, INTER_MESS);
            break;
        }

    /*控制器开关状态数据上报结束*/

    //采集器数据、控制器数据及网关采集数据上报开始
    case 0x05:  //module_send_flg
        //网关系统初始化完成后，采集器和控制器子站命令不存在，同时控制器子站ID小于等于64个（0~65）。下面的发送子站命令跳过;

        while ((ZZ_Wireles_flag[sendnum_mflg] == 0) && (ZZ_Wired_flag[sendnum_mflg] == 0) && (sendnum_mflg <= 64))   //CSH_Wired_finish==0|
        {
            sendnum_mflg++;
        }
        if (sendnum_mflg <= 64)   //该数据组是否已发送完成，没有发送完，继续发送.，sendnum_mflg是记录第几个子站上报
        {
            if (send_flg == 0x00)
            {
                send_flg = 0x02;
                module_send_flg = 0x05;
                Start_timerEx(SEND_PLATFORM_EVT, Ethernet_WAIT_TIME);
                break;
            }
            if (send_flg == 0x02)
            {
                real_send[20] = 0x00;//平台地址低字节
                real_send[21] = 0x01;//平台地址高字节
                chge_coltsnd(sendnum_mflg);//计算每组采集器上报数据的开始地址,修改了real_send的数据地址(0100H,0130H,0160H,...)
                send_message_len1 = 6 + 18 + real_send[22] + 3;//6+18+实际字节数值+3
                bytelen_to_asc((unsigned char *)message_len_char1, send_message_len1);//send_message_len全局变量
                //mqtt_publish(real_send,75);//MQTT 实际发送75个字节,这里的75仅是为了程序可读，没有用.
                //send_data_module(mqtt_real_send,"094");
                send_Ethernet_module(real_send, (unsigned char *)message_len_char1);//TCB长度=6+信息长度=6+2*4+45=51+8；
                module_send_flg = 0x5;
                send_flg = 0x03;
                Start_timerEx(SEND_PLATFORM_EVT, Ethernet_WAIT_TIME);
                break;
            }
            if (send_flg == 0x03)
            {
                module_send_flg = 0x5;//1组数据上报完成，等待5S进入下1组数据上报
                send_flg = 0x00;
                send_message_type = 0x00;
                sendnum_mflg++;
                Start_timerEx(SEND_PLATFORM_EVT, INTER_SEND);
                break;
            }
        }
        else
        {
            module_send_flg = 0x06;
            send_flg = 0x00;
            send_message_type = 0x00;
            real_send[4] = 0x19;//信息长度25；18+实际字节数值+3=18+real_send[22]+3=18+4+3=25=0x19
            real_send[20] = 0x00;//平台地址低字节
            real_send[21] = 0x25;//平台地址高字节
            real_send[22] = 0x04;//实际发送字节数低字节,2*2*1(自控回路)；1个自控回路为一组上报
            real_send[23] = 0x00;//实际发送字节数高字节
            sendnum_mflg = 0x00;
            Start_timerEx(SEND_PLATFORM_EVT, INTER_SEND);
            break;
        }
    //网关采集数据上报结束

    //自控状态上报开始
    case 0x06:  //module_send_flg
        //网关系统初始化完成后，控制器子站命令不存在，同时控制器子站ID小于32个（0~31）。下面的发送子站命令跳过;
        while ((hand_auto_count[sendnum_mflg] == 0) && (sendnum_mflg < 73))   //CSH_Wired_finish==0|
        {
            sendnum_mflg++;
        }
        if (sendnum_mflg < 73)
        {
            if (send_flg == 0x00)
            {
                send_flg = 0x02;
                module_send_flg = 0x06;
                Start_timerEx(SEND_PLATFORM_EVT, Ethernet_WAIT_TIME);
                break;
            }
            if (send_flg == 0x02)
            {
                real_send[20] = 0x00;//平台地址低字节
                real_send[21] = 0x25;//平台地址高字节
                alter_hand_auto(0x04, sendnum_mflg);//计算每组自控回路上报状态数据的开始地址,修改了real_send的数据地址(2500H,2504H,2508H,...)
                //mqtt_publish(real_send,75);//MQTT 实际发送75个字节,这里的75仅是为了程序可读，没有用。
                //send_data_module(mqtt_real_send,"094");
                send_message_len1 = 6 + 18 + real_send[22] + 3;//6+18+实际字节数值+3
                bytelen_to_asc((unsigned char *)message_len_char1, send_message_len1);//send_message_len全局变量
                send_Ethernet_module(real_send, (unsigned char *)message_len_char1);//TCB长度为43=6+信息长度=6+37=43；CDMA的AT指令长度为21；GPRS的AT指令长度为15
                module_send_flg = 0x6;
                send_flg = 0x03;
                Start_timerEx(SEND_PLATFORM_EVT, Ethernet_WAIT_TIME);
                break;
            }
            if (send_flg == 0x03)
            {
                module_send_flg = 0x6;//1组数据上报完成，等待5S进入下1组数据上报
                send_flg = 0x00;
                send_message_type = 0x00;
                if (hand_auto_count[sendnum_mflg] >= 1)
                {
                    hand_auto_count[sendnum_mflg]--;
                }
                sendnum_mflg++;
                Start_timerEx(SEND_PLATFORM_EVT, INTER_MESS);
                break;
            }
        }
        else
        {
            module_send_flg = 0x01;//取消延时时间，6秒发一次数据；如果需要延时时间为0xFF
            send_flg = 0x00;
            send_message_type = 0x00;
            sendnum_mflg = 0x00;
            Start_timerEx(SEND_PLATFORM_EVT, INTER_MESS);
            break;
        }

    /*自控状态数据上报结束*/
    default:
        Start_timerEx(SEND_PLATFORM_EVT, DATA_WAIT_TIME);//允许CDMA向平台发送数据
        break;
    }
}
static void Ethernet_mqtt_send(void)   //电信mqtt发送数据
{
    Start_timerEx(SEND_PLATFORM_EVT, DATA_WAIT_TIME);//允许CDMA向平台发送数据
}

static void Ethernet_sdk_send(void)   //通过以太网（sdk）向极码平台发送数据（极码平台）
{
    if (send_mess)  	//正在发送短信上报时，等待2s后重新执行
    {
        Start_timerEx(SEND_PLATFORM_EVT, 2000);
        return;
    }

    if (send_message_type != 0x03 && send_message_type != 0x00)   //对子站初始化工作完成ZZ_Wired_flag[65]=1；||ZZ_Wireles_flag[65]==1
    {
        Start_timerEx(SEND_PLATFORM_EVT, 500);
        return;
    }
    send_message_type = 0x03;	//在收到GSM的ERROR报告时使用，0x03为向平台上报数据

    /*send_message_type记录向cdma发送命令大类型，0x01为初始化程序，0x02为向平台返回控制命令，0x03为向平台上报数据。send_flg记录
    向cdma发送命令的小类型，0x01为初始化程序的at指令和data指令；0x02为平台上报数据的at指令，0x03为平台上报数据的data指令
    0x04为平台返回控制的at指令，0x05为平台返回控制的data指令；需要向平台发指令就一定有回复,
    在接收程序中用next_atdata_flg控制程序执行顺序，不向平台发指令的，就用module_send_flg控制程序执行。
    */

    switch (module_send_flg)   //0x0100~0x04FF采集器检测参数;0x500~0x08FF控制器检测参数;0x0900~0x091F网关检测参数;0x0920上电指示
    {
        u8 len, temp_date[2];
    case 0x01:
        module_send_flg = 0x02;
        send_flg = 0x02; //表示向平台上报的at指令正在执行，下面只能执行data指令
        Start_timerEx(SEND_PLATFORM_EVT, Ethernet_WAIT_TIME);
        break;
    case 0x02:
        memset(real_send, 0, sizeof(real_send));
        temp_date[0] = 0x01;
        temp_date[1] = 0x00;
        len = made_keyX_value(0x0920, temp_date, 1, real_send);//0x0920上电指示地址，值0x0001,1一个参数，返回real_send[]={30393230:30303031};len=11个字节
        if (factory_gateway_set[1] == 4)len = made_keyX_value4(0x0920, temp_date, 1, real_send);
        mqtt_publish(real_send, len);
        bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);//mqtt_len为全局变量，在mqtt_publish11函数中赋值
        send_Ethernet_module(ReportData2, (unsigned char *)message_len_char1);//ReportData2为实际发送内容，在mqtt_publish函数中包装
        module_send_flg = 0x03;
        send_flg = 0x03;//表示向平台上报的data指令正在执行
        Start_timerEx(SEND_PLATFORM_EVT, Ethernet_WAIT_TIME);
        break;
    case 0x03:
        module_send_flg = 0x04;
        send_flg = 0x00;
        send_message_type = 0x00;
        sendnum_mflg = 0x00;
        Start_timerEx(SEND_PLATFORM_EVT, INTER_SEND);
        break;

    //控制器开关状态数据上报开始
    case 0x04:  //module_send_flg
        //网关系统初始化完成后，控制器子站命令不存在，同时控制器子站ID小于32个（0~31）。下面的发送子站命令跳过;
        while ((ZZ_Wireles_flag[32 + sendnum_mflg] == 0) && (ZZ_Wired_flag[32 + sendnum_mflg] == 0) && (sendnum_mflg < 32))   //CSH_Wired_finish==0|
        {
            sendnum_mflg++;
        }
        if (sendnum_mflg < 32)
        {
            if (send_flg == 0x00)
            {
                send_flg = 0x02;
                module_send_flg = 0x04;
                Start_timerEx(SEND_PLATFORM_EVT, Ethernet_WAIT_TIME);
                break;
            }
            if (send_flg == 0x02)
            {
                memset(real_send, 0, sizeof(real_send));
                len = made_keyX_value(0x2100 + sendnum_mflg * 4 * 4, Controllers[sendnum_mflg], 4, real_send);//0x2100控制器控制状态地址，上报1个控制器4个状态参数
                if (factory_gateway_set[1] == 4)len = made_keyX_value4(0x2100 + sendnum_mflg * 4 * 4, Controllers[sendnum_mflg], 4, real_send);
                mqtt_publish(real_send, len);//该推送函数为SDK特别指定的，因为规定了控制器状态上报的推送主题
                bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);//mqtt_len为全局变量，在mqtt_publish11函数中赋值
                send_Ethernet_module(ReportData2, (unsigned char *)message_len_char1);//ReportData2为实际发送内容，在mqtt_publish函数中包装
                module_send_flg = 0x4;
                send_flg = 0x03;
                Start_timerEx(SEND_PLATFORM_EVT, Ethernet_WAIT_TIME);
                break;
            }
            if (send_flg == 0x03)
            {
                module_send_flg = 0x4;//1组数据上报完成，等待5S进入下1组数据上报
                send_flg = 0x00;
                send_message_type = 0x00;
                sendnum_mflg++;
                Start_timerEx(SEND_PLATFORM_EVT, INTER_SEND);
                break;
            }
        }
        else
        {
            module_send_flg = 0x5;
            next_atdata_flg = 0x05;
            send_flg = 0x00;
            send_message_type = 0x00;
            sendnum_mflg = 0x00;
            Start_timerEx(SEND_PLATFORM_EVT, INTER_MESS);
            break;
        }

    /*控制器开关状态数据上报结束*/

    //采集器数据、控制器数据及网关采集数据上报开始
    case 0x05:  //module_send_flg
        //网关系统初始化完成后，采集器和控制器子站命令不存在，同时控制器子站ID小于等于64个（0~65）。下面的发送子站命令跳过;

        while ((ZZ_Wireles_flag[sendnum_mflg] == 0) && (ZZ_Wired_flag[sendnum_mflg] == 0) && (sendnum_mflg <= 64))   //CSH_Wired_finish==0|
        {
            sendnum_mflg++;
        }
        if (sendnum_mflg <= 64)   //该数据组是否已发送完成，没有发送完，继续发送.，sendnum_mflg是记录第几个子站上报
        {
            if (send_flg == 0x00)
            {
                send_flg = 0x02;
                module_send_flg = 0x05;
                Start_timerEx(SEND_PLATFORM_EVT, Ethernet_WAIT_TIME);
                break;
            }
            if (send_flg == 0x02)
            {
                memset(real_send, 0, sizeof(real_send));
                len = made_keyX_valueF(0x0100 + sendnum_mflg * 4 * 8, Collectors[sendnum_mflg], zero_rang.k_b_float[sendnum_mflg], 4 * send_count, real_send);
                //0x0100为采集器和控制器采集数据地址，上报1个控制器8个采集参数
                if (factory_gateway_set[1] == 4)len = made_keyX_valueF4(0x0100 + sendnum_mflg * 4 * 8, Collectors[sendnum_mflg], zero_rang.k_b_float[sendnum_mflg], 4 * send_count, real_send);
                mqtt_publish(real_send, len);//该推送函数为通用推送函数
                bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);//mqtt_len为全局变量，在mqtt_publish11函数中赋值
                send_Ethernet_module(ReportData2, (unsigned char *)message_len_char1);//ReportData2为实际发送内容，在mqtt_publish函数中包装
                if (send_count == 0)
                {
                    send_count = 1;
                    module_send_flg = 0x5;
                    send_flg = 0x00;
                    send_message_type = 0x00;
                    Start_timerEx(SEND_PLATFORM_EVT, Ethernet_WAIT_TIME);//8个参数分2次连续上报；300ms
                    break;
                }
                else
                {
                    send_count = 0;
                    module_send_flg = 0x5;
                    send_flg = 0x03;
                    Start_timerEx(SEND_PLATFORM_EVT, Ethernet_WAIT_TIME);
                    break;
                }
            }

            if (send_flg == 0x03)
            {
                module_send_flg = 0x5;//1组数据上报完成，等待5S进入下1组数据上报
                send_flg = 0x00;
                send_message_type = 0x00;
                send_count = 0;
                sendnum_mflg++;
                Start_timerEx(SEND_PLATFORM_EVT, Ethernet_WAIT_TIME);
                break;
            }
        }
        else
        {
            module_send_flg = 0x06;
            send_flg = 0x00;
            send_message_type = 0x00;
            sendnum_mflg = 0x00;
            Start_timerEx(SEND_PLATFORM_EVT, INTER_SEND);
            break;
        }
    //网关采集数据上报结束
    //自控回路手动-自动状态数据上报开始
    case 0x06:  //module_send_flg
        //网关系统初始化完成后，控制器子站命令不存在，同时控制器子站ID小于32个（0~31）。下面的发送子站命令跳过;
        while ((hand_auto_count[sendnum_mflg] == 0) && (sendnum_mflg < 73))   //CSH_Wired_finish==0|
        {
            sendnum_mflg++;
        }
        if (sendnum_mflg < 73)
        {
            if (send_flg == 0x00)
            {
                send_flg = 0x02;
                module_send_flg = 0x06;
                Start_timerEx(SEND_PLATFORM_EVT, Ethernet_WAIT_TIME);
                break;
            }
            if (send_flg == 0x02)
            {
                memset(real_send, 0, sizeof(real_send));
                len = made_keyX_value(0x2500 + sendnum_mflg * 1 * 4, hand_auto_flg[sendnum_mflg], 1, real_send);//0x2500自控回路手自动状态地址，1次上报1个手动-自动状态
                if (factory_gateway_set[1] == 4)len = made_keyX_value4(0x2500 + sendnum_mflg * 1 * 4, hand_auto_flg[sendnum_mflg], 1, real_send);
                mqtt_publish(real_send, len);//该推送函数为SDK特别指定的，因为规定了控制器状态上报的推送主题
                bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);//mqtt_len为全局变量，在mqtt_publish11函数中赋值
                send_Ethernet_module(ReportData2, (unsigned char *)message_len_char1);//ReportData2为实际发送内容，在mqtt_publish函数中包装
                next_atdata_flg = 0x06;
                send_flg = 0x03;
                Start_timerEx(SEND_PLATFORM_EVT, Ethernet_WAIT_TIME);
                break;
            }
            if (send_flg == 0x03)
            {
                module_send_flg = 0x06;//1组数据上报完成，等待5S进入下1组数据上报
                send_flg = 0x00;
                send_message_type = 0x00;
                sendnum_mflg++;
                Start_timerEx(SEND_PLATFORM_EVT, INTER_SEND);
                break;
            }
        }
        else
        {
            module_send_flg = 0x01;//取消延时时间，6秒发一次数据；如果需要延时时间为0xFF
            send_flg = 0x00;
            send_message_type = 0x00;
            sendnum_mflg = 0x00;
            Start_timerEx(SEND_PLATFORM_EVT, INTER_MESS);
            break;
        }

    /*控制器开关状态数据上报结束*/
    default:
        Start_timerEx(SEND_PLATFORM_EVT, DATA_WAIT_TIME);//允许CDMA向平台发送数据
        break;
    }
}

static void Ethernet_other_send(void)   //电信other发送数据
{
    Start_timerEx(SEND_PLATFORM_EVT, DATA_WAIT_TIME);//允许CDMA向平台发送数据
}

/*Ethernet或WiFi平台接收函数类（串口1）-----------------------------------------------------------------------------------------------*/
static void RxReport1(u8 len, u8 *pData)   //从平台（CDMA或GPRS）接收数据处理
{
    switch (factory_gateway_set[0])   //网络类型=3 以太网，4 WiFi
    {
    //以太网开始---------------------------------
    case 0x03:	//以太网
        switch (factory_gateway_set[1])   //协议类型=1 TCP/IP;2 MQTT; 3 SDK; 4 其它；
        {
        case 0x01:   //以太网TCP/IP
            Ethernet_tcp_receive(len, pData);
            break;

        case 0x02:  //以太网MQTT
            Ethernet_mqtt_receive(len, pData);
            break;

        case 0x03:   //以太网SDK
            Ethernet_sdk_receive(len, pData);
            break;

        case 0x04:  //以太网other
            Ethernet_other_receive(len, pData);
            break;
        default:
            break;
        }

        break;
    //以太网 结束------------------------------------

    //WiFi	开始====================================
    case 0x04:
        switch (factory_gateway_set[1])   //协议类型=1 TCP/IP;2 MQTT; 3 SDK; 4 其它；
        {
        //WiFi TCP/IP
        case 0x01:
            //				gprs_tcp_receive(len,pData);
            break;
        //WiFi MQTT
        case 0x02:
            //				gprs_mqtt_receive(len,pData);
            break;
        //WiFi SDK
        case 0x03:
            //				gprs_sdk_receive(len,pData);
            break;
        //WiFi other
        case 0x04:
            //				gprs_other_receive(len,pData);
            break;
        default:
            break;
        }
        break;
    //WiFi	结束====================================
    default:
        break;
    }
}

/*CDMA或GPRS平台接收函数类（串口2）-----------------------------------------------------------------------------------------------*/
static void RxReport2(u8 len, u8 *pData)   //从平台（CDMA或GPRS）接收数据处理
{
    switch (factory_gateway_set[0])   //网络类型=1 电信，2 移动
    {
    //电信 cdma 开始---------------------------------
    case 0x01:	//电信 cdma
        switch (factory_gateway_set[1])   //协议类型=1 TCP/IP;2 MQTT; 3 SDK; 4 其它；
        {
        case 0x01:   //电信TCP/IP
            cdma_cmd_receive(len, pData);//RxReport2_step_flg是为了减少CPU运行负担
            if (RxReport2_step_flg == 0)
            {
                cdma_tcp_receive(len, pData);   //RxReport2_step_flg=1表示链路函数处理过cdma_cmd_receive
            }
            if (RxReport2_step_flg == 0)
            {
                cdma_SMS_receive(len, pData);   //RxReport2_step_flg=2表示协议函数处理过cdma_tcp_receive或cdma_sdk_receive等
            }
            break;

        case 0x02:  //电信MQTT
            cdma_cmd_receive(len, pData);
            if (RxReport2_step_flg == 0)
            {
                cdma_mqtt_receive(len, pData);
            }
            if (RxReport2_step_flg == 0)
            {
                cdma_SMS_receive(len, pData);
            }
            break;

        case 0x03:   //电信SDK
            cdma_cmd_receive(len, pData);
            if (RxReport2_step_flg == 0)
            {
                cdma_sdk_receive(len, pData);
            }
            if (RxReport2_step_flg == 0)
            {
                cdma_SMS_receive(len, pData);
            }
            break;

        case 0x04:  //电信other
            cdma_cmd_receive(len, pData);
            if (RxReport2_step_flg == 0)
            {
                cdma_sdk_receive(len, pData);
            }
            if (RxReport2_step_flg == 0)
            {
                cdma_SMS_receive(len, pData);
            }
            break;
        default:
            break;
        }

        break;
    //电信 cdma 结束------------------------------------

    //移动 gprs	开始====================================
    case 0x02:
        switch (factory_gateway_set[1])   //协议类型=1 TCP/IP;2 MQTT; 3 SDK; 4 其它；
        {
        //移动TCP/IP
        case 0x01:
            gprs_cmd_receive(len, pData);//RxReport2_step_flg是为了减少CPU运行负担
            if (RxReport2_step_flg == 0)
            {
                cdma_tcp_receive(len, pData);   //RxReport2_step_flg=1表示链路函数处理过cdma_cmd_receive；包含了 移动TCP/IP接收平台数据 gprs_tcp_receive
            }
            if (RxReport2_step_flg == 0)
            {
                cdma_SMS_receive(len, pData);   //RxReport2_step_flg=2表示协议函数处理过cdma_tcp_receive或cdma_sdk_receive等	包含了移动接收短信处理函数gprs_SMS_receive
            }
            break;
        //移动MQTT
        case 0x02:
            gprs_cmd_receive(len, pData);//RxReport2_step_flg是为了减少CPU运行负担
            if (RxReport2_step_flg == 0)
            {
                cdma_mqtt_receive(len, pData);   //包含了 移动mqtt接收平台数据 gprs_mqtt_receive
            }
            if (RxReport2_step_flg == 0)
            {
                cdma_SMS_receive(len, pData);   //包含了移动接收短信处理函数gprs_SMS_receive
            }
            break;
        //移动SDK
        case 0x03:
            gprs_cmd_receive(len, pData);//RxReport2_step_flg是为了减少CPU运行负担
            if (RxReport2_step_flg == 0)
            {
                cdma_sdk_receive(len, pData);   //包含了 移动sdk接收平台数据 gprs_sdk_receive
            }
            if (RxReport2_step_flg == 0)
            {
                cdma_SMS_receive(len, pData);   //包含了移动接收短信处理函数gprs_SMS_receive
            }
            break;
        //移动other
        case 0x04:
            gprs_other_receive(len, pData);
            break;
        default:
            break;
        }
        break;
    //移动 gprs	结束====================================
    default:
        break;
    }
}
/*CDMA或GPRS接收函数-----------------------------------------------------------------------------------*/
static void cdma_cmd_receive(u8 len, u8 *pBuf)   //电信TCP/IP接收CDMA命令返回处理函数
{
    //   memcpy(test,pBuf,100);
    if ((mqtt_bcmd = strstr((char *)pBuf, "+CSQ:")) != NULL)   //说明CDMA工作正常,wakeup_flg==0x01表示发唤醒包期间
    {
        //		gsm_halt_test=0x00;
        //		 Start_timerEx(GSM_AT_READBUF,10000);//10S检测一次，三次没有正常回复则认为CDMA有问题，在cycle_cmd()判断
        RxReport2_step_flg = 1;
        return;
    }

    if (strstr((char *)pBuf, "ERROR") != NULL)   //网关系统重新启动，调试后恢复
    {
        if (send_message_type == 0x01 || send_message_type == 0x02 || send_message_type == 0x03 || send_message_type == 0x04)   //0x02为回复平台控制指令；0x03为平台上报数据
        {
            if (error_num < MAX_ERROR_NUM)   //出现AT指令错误，重复执行该指令，次数由MAX_ERROR_NUM决定
            {
                if (send_message_type == 0x01 && send_flg == 0x01)
                {
                    Start_timerEx(NET_INIT_EVT, 1000);	//重新执行该指令，reprot_flg没有改变
                }
                if (send_message_type == 0x02 && send_flg == 0x04)
                {
                    send_flg = 0x00;
                    send_message_type = 0x00;
                    Start_timerEx(WG_REPLY_EVT, 100);
                }
                if (send_message_type == 0x02 && send_flg == 0x05)
                {
                    send_flg = 0x04;
                    Start_timerEx(WG_REPLY_EVT, 100);
                }

                if (send_message_type == 0x03 && send_flg == 0x02)
                {
                    send_flg = 0x00;
                    send_message_type = 0x00;
                    Start_timerEx(SEND_PLATFORM_EVT, 100);
                }
                if (send_message_type == 0x03 && send_flg == 0x03)
                {
                    send_flg = 0x02;
                    Start_timerEx(SEND_PLATFORM_EVT, 100);
                }
                if (send_message_type == 0x04 && send_flg == 0x04)
                {
                    send_flg = 0x00;
                    send_message_type = 0x00;
                    Start_timerEx(JM_PLATFORM_REPLY_EVT, 100);
                }
                if (send_message_type == 0x04 && send_flg == 0x05)
                {
                    send_flg = 0x04;
                    Start_timerEx(JM_PLATFORM_REPLY_EVT, 100);
                }
                send_message_type = 0x00;
                error_num++;
                RxReport2_step_flg = 1;
                return;
            }
            else
            {
                cmd_flg = 0x01;//LINK_SERVER=0xE9
                send_flg = 0x00;
                error_num = 0x00;
                next_atdata_flg = 0;
                Stop_timerEx(WG_REPLY_EVT);
                Stop_timerEx(SEND_PLATFORM_EVT);
                Start_timerEx(NET_INIT_EVT, 500);
                RxReport2_step_flg = 1;
                return;
            }
        }
    }

    if (strstr((char *)pBuf, "\r\nOK\r\n") != NULL)
    {
        //0x01为cdma初始化；0x02为回复平台控制指令；0x03为平台上报数据
        error_num = 0x00;
        if (send_message_type == 0x01 && send_flg == 0x01)
        {
            cmd_flg = next_atdata_flg;
            Start_timerEx(NET_INIT_EVT, 500);
            RxReport2_step_flg = 1;
            return;
        }
        if (send_message_type == 0x03 && send_flg == 0x02)
        {
            module_send_flg = next_atdata_flg;
            Start_timerEx(SEND_PLATFORM_EVT, 100);//GSM为10，CDMA为100,发送AT指令
            RxReport2_step_flg = 1;
            return;
        }
        if (send_message_type == 0x03 && send_flg == 0x03)
        {
            module_send_flg = next_atdata_flg;
            Start_timerEx(SEND_PLATFORM_EVT, 150);//GSM为10ms,CDMA为50ms;发数据指令
            RxReport2_step_flg = 1;
            return;
        }
        if (send_message_type == 0x02 && send_flg == 0x04)
        {
            Start_timerEx(WG_REPLY_EVT, 100);//GSM为10，CDMA为100;发AT指令
            RxReport2_step_flg = 1;
            return;
        }
        if (send_message_type == 0x02 && send_flg == 0x05)
        {
            Start_timerEx(WG_REPLY_EVT, 150);//GSM为10ms；CDMA为50ms；发DATA指令
            RxReport2_step_flg = 1;
            return;
        }
        if (send_message_type == 0x04 && send_flg == 0x04)
        {
            Start_timerEx(JM_PLATFORM_REPLY_EVT, 100);//GSM为10，CDMA为100;发AT指令
            RxReport2_step_flg = 1;
            return;
        }
        if (send_message_type == 0x04 && send_flg == 0x05)
        {
            Start_timerEx(JM_PLATFORM_REPLY_EVT, 150);//GSM为10ms；CDMA为50ms；发DATA指令
            RxReport2_step_flg = 1;
            return;
        }
        RxReport2_step_flg = 1;
        return;
    }

    if ((mqtt_bcmd = strstr((char *)pBuf, "^IPSTATE: 1,0,remote close")) != NULL)   //远程服务器关闭了链路,CDMA模块指令
    {
        cmd_flg = 0x01;
        send_flg = 0x01;
        Stop_timerEx(SEND_PLATFORM_EVT);
        Start_timerEx(NET_INIT_EVT, 500);
        return;
    }
    if ((mqtt_bcmd = strstr((char *)pBuf, "\r\n^DSDORMANT: 1\r\n")) != NULL)   //远程服务器或CDMA模块发起进入休眠状态
    {
        cmd_flg = 0x01;
        send_flg = 0x01;
        Stop_timerEx(SEND_PLATFORM_EVT);
        Start_timerEx(NET_INIT_EVT, 500);
        return;
    }
}

static void cdma_tcp_receive(u8 len, u8 *pBuf)   //电信TCP/IP接收平台数据
{
    u8 k = 0, first_k = 0;

    if (strstr((char *)pBuf, "^IPDATA:1,") != NULL && factory_gateway_set[0] == 1)first_k = 13; //CDMA：^IPDATA:1 ;GSM:^SISR: 0,注意空格；
    if (strstr((char *)pBuf, "^SISR: 0,") != NULL && factory_gateway_set[0] == 2)first_k = 13; //CDMA：^IPDATA:1 ;GSM:^SISR: 0,注意空格；
    if (first_k != 0)
    {
        /*平台控制指令*/
        for (k = 13; k < len; k++)
            //mqtt-CDMA在0xCC前面有43个字节，k=43（订阅主题长度为15个字节不变,增加或减少订阅主题字节数，则相应修改k值）
            //TCP-CDMA 在0xCC前面有AT指令11个字节+2个字节（长度），k=13 ;这样可以保证前面的字符串中有0xCC也没有关系。
            //mqtt-GSM在0xCC前面有43个字节，k=43（订阅主题长度为15个字节不变,增加或减少订阅主题字节数，则相应修改k值）
            //TCP-GSM 在0xCC前面AT指令有13个字节（其中2个字节是长度），k=13 ;这样可以保证前面的字符串中有0xCC也没有关系。
        {
            if (pBuf[k] == 0xCC)   //上报平台协议规定的帧头
            {
                if (pBuf[k - 3] == real_send[7] && pBuf[k - 2] == real_send[8] && pBuf[k - 1] == real_send[9])   //采控器ID
                {
                    handlecmd(pBuf + k - 10, (((u16)pBuf[k - 5]) << 8) + pBuf[k - 6] + 6);
                    //处理平台下发的控制指令，以0xCC为标准得到完整的平台指令（0x01,0x00,...,0xDD）,长度为完整指令长度
                    RxReport2_step_flg = 2;
                    return;
                }
            }
        }
    }
}

static void cdma_mqtt_receive(u8 len, u8 *pBuf)   //电信mqtt接收平台数据处理函数
{
    if ((mqtt_bcmd = strstr((char *)pBuf, "^IPDATA:1,4,")) != NULL || (mqtt_bcmd = strstr((char *)pBuf, "^SISR: 0,4\r\n")) != NULL)
        //mqtt-CDMA ^IPDATA:1,4, 为12个字节，mqtt主题中不能出现“OK”和“remote close”

        //if((mqtt_bcmd=strstr((char *)pBuf,"^SISR: 0,4\r\n"))!=NULL) return; //GSM
        //mqtt-GSM ^SISR: 0,4\r\n 为12个字节，mqtt主题中不能出现“OK”和“remote close”
        //5E 53 49 53 52 3A 20 30 2C 34 0D 0A;注意空格
    {
        mqtt_bcmdxb = (u8 *)mqtt_bcmd - pBuf;//计算下标

        if (pBuf[mqtt_bcmdxb + 12] == 0x20 && pBuf[mqtt_bcmdxb + 13] == 0x02 && pBuf[mqtt_bcmdxb + 14] == 0x00 && pBuf[mqtt_bcmdxb + 15] == 0x00)
            //CDMA从链路1收到4个字节的mqtt-connect回复值：正常为0x20,0x02,0x00,0x00(0x01~0x04不正常)；publish没有回复值
        {
            RxReport2_step_flg = 2;
            return;
        }
    }

    if ((mqtt_bcmd = strstr((char *)pBuf, "^IPDATA:1,5,")) != NULL || (mqtt_bcmd = strstr((char *)pBuf, "^SISR: 0,5\r\n")) != NULL)   //MQTT-CDMA;AT命令长度12个字节
        //if((mqtt_bcmd=strstr((char *)pBuf,"^SISR: 0,5\r\n"))!=NULL) return; //GSM处理
        //MQTT-GSM;AT命令长度12个字节；注意空格
    {
        mqtt_bcmdxb = (u8 *)mqtt_bcmd - pBuf;

        if (pBuf[mqtt_bcmdxb + 12] == 0x90 && pBuf[mqtt_bcmdxb + 13] == 0x03 && pBuf[mqtt_bcmdxb + 14] == 0x00 && pBuf[mqtt_bcmdxb + 15] == 0x01)
            //CDMA从链路1收到5个字节的mqtt-subscribe回复值：0x90,0x03(后面字节长度)，0x00,0x01(msgid二字节)，0x00(状态位)
        {
            RxReport2_step_flg = 2;
            return;
        }
    }
    if (strstr((char *)pBuf, "^IPDATA:1,") != NULL || strstr((char *)pBuf, "^SISR: 0,") != NULL)   //CDMA：^IPDATA:1 ;GSM:^SISR: 0,注意空格；
    {
        /*平台控制指令*/
        u8 k;
        k = 28 + factory_gateway_set[83];//订阅主题长度
        for (; k < factory_gateway_set[83] + len; k++)
            //mqtt-CDMA在0xCC前面有43个字节，k=43（订阅主题长度为15个字节不变,增加或减少订阅主题字节数，则相应修改k值）
            //TCP-CDMA 在0xCC前面有AT指令11个字节+2个字节（长度），k=13 ;这样可以保证前面的字符串中有0xCC也没有关系。
            //mqtt-GSM在0xCC前面有43个字节，k=43（订阅主题长度为15个字节不变,增加或减少订阅主题字节数，则相应修改k值）
            //TCP-GSM 在0xCC前面AT指令有13个字节（其中2个字节是长度），k=13 ;这样可以保证前面的字符串中有0xCC也没有关系。
        {
            if (pBuf[k] == 0xCC)   //上报平台协议规定的帧头
            {
                if (pBuf[k - 3] == real_send[7] && pBuf[k - 2] == real_send[8] && pBuf[k - 1] == real_send[9])   //采控器ID
                {
                    handlecmd(pBuf + k - 10, (((u16)pBuf[k - 5]) << 8) + pBuf[k - 6] + 6);//pBuf+k-10为0x01的地址；pBuf[k-5])<<8)+pBuf[k-6]+6为完整指令的长度
                    //处理平台下发的控制指令，以0xCC为标准得到完整的平台指令（0x01,0x00,...,0xDD）,长度为完整指令长度
                    RxReport2_step_flg = 2;
                    return;
                }
            }
        }
    }
}
static void cdma_sdk_receive(u8 len, u8 *pBuf)   //电信sdk接收平台数据
{
    // 下面说明是MQTT指令回复处理

    if ((mqtt_bcmd = strstr((char *)pBuf, "^IPDATA:1,4,")) != NULL || (mqtt_bcmd = strstr((char *)pBuf, "^SISR: 0,4\r\n")) != NULL)
        //mqtt-CDMA ^IPDATA:1,4, 为12个字节，mqtt主题中不能出现“OK”和“remote close”

        //if((mqtt_bcmd=strstr(pBuf,"^SISR: 0,4\r\n"))!=NULL) return; //GSM
        //mqtt-GSM ^SISR: 0,4\r\n 为12个字节，mqtt主题中不能出现“OK”和“remote close”
        //5E 53 49 53 52 3A 20 30 2C 34 0D 0A;注意空格
    {
        mqtt_bcmdxb = (u8 *)mqtt_bcmd - pBuf;//计算下标

        if (pBuf[mqtt_bcmdxb + 12] == 0x20 && pBuf[mqtt_bcmdxb + 13] == 0x02 && pBuf[mqtt_bcmdxb + 14] == 0x00 && pBuf[mqtt_bcmdxb + 15] == 0x00)
            //CDMA从链路1收到4个字节的mqtt-connect回复值：正常为0x20,0x02,0x00,0x00(0x01~0x0不正常)；publish没有回复值
        {
            RxReport2_step_flg = 2;
            return;
        }
    }

    if ((mqtt_bcmd = strstr((char *)pBuf, "^IPDATA:1,5,")) != NULL || (mqtt_bcmd = strstr((char *)pBuf, "^SISR: 0,5\r\n")) != NULL)   //MQTT-CDMA;AT命令长度12个字节
        //if((mqtt_bcmd=strstr((char *)pBuf,"^SISR: 0,5\r\n"))!=NULL) return; //GSM处理
        //MQTT-GSM;AT命令长度12个字节；注意空格
    {
        mqtt_bcmdxb = (u8 *)mqtt_bcmd - pBuf;

        if (pBuf[mqtt_bcmdxb + 12] == 0x90 && pBuf[mqtt_bcmdxb + 13] == 0x03 && pBuf[mqtt_bcmdxb + 14] == 0x00 && pBuf[mqtt_bcmdxb + 15] == 0x01)
            //CDMA从链路1收到5个字节的mqtt-subscribe回复值：0x90,0x03(后面字节长度)，0x00,0x01(msgid二字节)，0x00(状态位)
        {
            RxReport2_step_flg = 2;
            return;
        }
    }
    //		memcpy(test,pBuf,100);
    mqtt_bcmdxb = match_str(pBuf, len, (unsigned char *)"{\"method\":\"", 11);
    if (factory_gateway_set[1] == 4)mqtt_bcmdxb = match_str(pBuf, len, (unsigned char *)"{\"command\":\"", 12);//测试
    if (mqtt_bcmdxb != 0)   //CDMA：^IPDATA:1 ;GSM:^SISR: 0,注意空格；
    {
        memcpy(ctrl_key, pBuf + mqtt_bcmdxb + 1, 4);
        memcpy(ctrl_value, pBuf + mqtt_bcmdxb + 1 + 16, 4);
        //	 if(factory_gateway_set[1]==4)memcpy(ctrl_value,pBuf+mqtt_bcmdxb+16,4);//测试
        /*平台控制指令 {"method":"2100","params":"0001"}*/
        ctrl_adrr = dword_asc_hex(ctrl_key);//将4个字节的ASC码转换成十六进制数0x2100
        ctrl_cmd = dword_asc_hex(ctrl_value);
        if (ctrl_adrr >= 0x2100 && ctrl_adrr < 0x2300)
        {
            offset_addrX = (ctrl_adrr - 0x2100) / 16;
            offset_addrY = (ctrl_adrr & 0x000F) / 2;
            Controllers[offset_addrX][offset_addrY] = ctrl_cmd & 0x00FF;
            Controllers[offset_addrX][offset_addrY + 1] = (ctrl_cmd >> 8) & 0x00FF;
            crtl_cmd_num[offset_addrX][offset_addrY / 2] = 50;//子站有线发送记录，允许重复发送3次，收到清0，不再继续发送。
            crtl_cmd_numWX[offset_addrX][offset_addrY / 2] = 50;//子站无线发送记录，允许重复发送3次，收到清0，不再继续发送。
            memset(sdk_ctrl_reply, 0, sizeof(sdk_ctrl_reply));//上一次出现错误ERROR或其它返回值有可能保留在数组中，所以需要清零
            sdk_ctrl_reply[0] = len;
            memcpy(sdk_ctrl_reply + 1, pBuf, len);
            Start_timerEx(WG_REPLY_EVT, 150);
            RxReport2_step_flg = 2;
            return;
        }
        if (ctrl_adrr >= 0x2300 && ctrl_adrr < 0x2500)
        {
            return;
        }
        if (ctrl_adrr >= 0x2500 && ctrl_adrr < 0x2620)
        {
            offset_addrX = (ctrl_adrr - 0x2500) / 4;
            hand_auto_flg[offset_addrX][0] = ctrl_cmd & 0x00FF;
            hand_auto_flg[offset_addrX][1] = (ctrl_cmd >> 8) & 0x00FF;
            hand_auto_count[offset_addrX] = 5;//向平台发送手动-自动状态记录，允许重复发送5次。
            memset(sdk_ctrl_reply, 0, sizeof(sdk_ctrl_reply));//上一次出现错误ERROR或其它返回值有可能保留在数组中，所以需要清零
            sdk_ctrl_reply[0] = len;
            memcpy(sdk_ctrl_reply + 1, pBuf, len);
            Start_timerEx(WG_REPLY_EVT, 150);
            RxReport2_step_flg = 2;
            return;
        }
    } //如果收到错误信息则不做处理
}
//static void cdma_other_receive(u8 len,u8 *pBuf)//电信other接收平台数据
//{
//}

static void cdma_SMS_receive(u8 len, u8 *pBuf)   //电信TCP/IP接收CDMA命令返回处理函数
{
    u8 _cnt = 0;
    u8 is_mess = 0;
    //	memcpy(test,pBuf,len);
    if (factory_gateway_set[0] == 1)is_mess = match_str(pBuf, len, (unsigned char *)"HCMT", 4);//CDMA短信以HCMT开头;GSM为+CMT
    if (factory_gateway_set[0] == 2)is_mess = match_str(pBuf, len, (unsigned char *)"+CMT", 4);//CDMA短信以HCMT开头;GSM为+CMT
    /*判断接收到的短信指令*/
    if (is_mess)   //判断短信
    {
        u8 i, j;
        u8 is_wakeup = match_str(pBuf, len, (unsigned char *)"HXCX", 4);
        u8 jmwgrest = match_str(pBuf, len, (unsigned char *)"JMWGREST", 8);
        u8 is_main = match_str(pBuf, len, main_call, 11);
        u8 is_voice = match_str(pBuf, len, voice_call, 11);
        u8 is_third = match_str(pBuf, len, third_call, 11);

        u8 mess_sta = 0, mess_len = 0, len_strt = 0;

        unsigned char _cmd[] = { "AT+CNMA\r\n" }; //gsm和cdma相同

        _cnt = 0;

        //		WriteDataToBuffer(2,_cmd,0,9);	//确认收到短信
        memcpy(USART2SendTCB, _cmd, 9);
        WriteDataToDMA_BufferTX2(9);

        for (i = is_mess + 1; i < len; i++)  	//^HCMT:13634171664,2012,06,19,21,23,03,0,1,6,0,0,0 PT8888 ；PT8888为实际所发的信息
        {
            if (pBuf[i] == 0x2C)  		//判断到逗号，逗号的ASCII码为0x2C
            {
                _cnt++;
                switch (_cnt)
                {
                case 1:
                    //call_len=i-is_mess-2;
                    break;
                case 9:
                    len_strt = i + 1;//短信有效长度的数组下标，这个例子6为实际所发的信息长度
                    break;
                case 10:
                    mess_len = 0;
                    for (j = len_strt; j < i; j++)
                    {
                        if (j + 1 == i)
                        {
                            mess_len += pBuf[j] - 0x30;
                        }
                        else
                        {
                            mess_len += (pBuf[j] - 0x30) * 10 * (i - j - 1);//mess_len实际信息长度
                        }
                    }
                    break;
                case 12:
                    mess_sta = i + 4;//短信开始，第一个字节，mess_sta为实际信息开始的下标
                    i = len;
                    break;
                default:
                    break;
                }
            }
        }

        if (is_wakeup)    //CDMA初始化
        {
            cmd_flg = 0x01;
            Start_timerEx(NET_INIT_EVT, 150);
        }
        else if (is_main || is_voice || is_third)
        {
            if (jmwgrest)   //当网关收到"JMWGREST"短信，系统进入死循环，系统重新启动
            {
                __set_FAULTMASK(1);//关闭所有的中断
                NVIC_SystemReset();//系统复位
                while (1);
            }
            if (pBuf[mess_sta] == 'I'&&pBuf[mess_sta + 1] == 'P')   //'IP'
            {
                if (len >= mess_sta + 23 && mess_len == 23 && is_number(pBuf + mess_sta + 2, 21))    //修改IP
                {
                    factory_gateway_set[30] = 21;
                    memcpy(factory_gateway_set + 31, pBuf + mess_sta + 2, 21);
                    Flash_Write(0x0807B000, factory_gateway_set, 255);
                }
            }

            else if (pBuf[mess_sta] == 0x44 && pBuf[mess_sta + 1] == 0x42)   //'D','B'
            {
                if (is_main)     			//修改voice_call,我的移动号码
                {
                    if (len >= mess_sta + 13 && mess_len == 13 && is_number(pBuf + mess_sta + 2, 11))
                    {
                        memcpy(factory_gateway_set + 221, pBuf + mess_sta + 2, 11);
                        Flash_Write(0x0807B000, factory_gateway_set, 255);
                    }
                }
                else if (is_voice)     //我的电信号码
                {
                    if (len >= mess_sta + 13 && mess_len == 13 && is_number(pBuf + mess_sta + 2, 11))
                    {
                        memcpy(factory_gateway_set + 232, pBuf + mess_sta + 2, 11);
                        Flash_Write(0x0807B000, factory_gateway_set, 255);
                    }
                }
            }
            else if (pBuf[mess_sta] == 0x44 && pBuf[mess_sta + 1] == 0x43)     //'D','C'
            {
                if (is_third)
                {
                    for (i = 0; i < 11; i++)
                    {
                        third_call[i] = 0;
                    }
                    memcpy(factory_gateway_set + 243, third_call, 11);
                    Flash_Write(0x0807B000, factory_gateway_set, 255);
                }
            }

            else if (pBuf[mess_sta] == 'X'&&pBuf[mess_sta + 1] == 'Y')  	//'XY'修改协议类型。
            {
                if (len >= mess_sta + 5 && mess_len == 5 && is_number(pBuf + mess_sta + 2, 3))
                {
                    factory_gateway_set[1] = (pBuf[mess_sta + 2] - 0x30) * 100 + (pBuf[mess_sta + 3] - 0x30) * 10 + (pBuf[mess_sta + 4] - 0x30);
                    Flash_Write(0x0807B000, factory_gateway_set, 255);
                }
            }

            else if (pBuf[mess_sta] == 'W'&&pBuf[mess_sta + 1] == 'G'&&pBuf[mess_sta + 2] <= 0x32 && pBuf[mess_sta + 2] >= 0x30)   //'WG'修改网关ID 002002004254
            {
                u8 _tmp = 0;
                if (len >= mess_sta + 14 && mess_len == 14 && is_number(pBuf + mess_sta + 2, 12))
                {
                    for (i = mess_sta + 2; i < mess_sta + 12; i = i + 3)
                    {
                        _tmp = (pBuf[i] - 0x30) * 100 + (pBuf[i + 1] - 0x30) * 10 + (pBuf[i + 2] - 0x30);
                        factory_gateway_set[2 + (i - mess_sta - 2) / 3] = _tmp;
                    }
                    Flash_Write(0x0807B000, factory_gateway_set, 255);
                }
            }
            else if (pBuf[mess_sta] == 'C'&&pBuf[mess_sta + 1] == 'K'&&pBuf[mess_sta + 2] <= 0x32 && pBuf[mess_sta + 2] >= 0x30)     //'CK'修改采控器ID 004005006009
            {
                u8 _tmp = 0;
                if (len >= mess_sta + 14 && mess_len == 14 && is_number(pBuf + mess_sta + 2, 12))
                {
                    for (i = mess_sta + 2; i < mess_sta + 12; i = i + 3)
                    {
                        _tmp = (pBuf[i] - 0x30) * 100 + (pBuf[i + 1] - 0x30) * 10 + (pBuf[i + 2] - 0x30);
                        factory_gateway_set[6 + (i - mess_sta - 2) / 3] = _tmp;
                    }
                    Flash_Write(0x0807B000, factory_gateway_set, 255);
                }
            }
            else if (pBuf[mess_sta] == 'K'&&pBuf[mess_sta + 1] == 'H'&&pBuf[mess_sta + 2] == 'J'&&pBuf[mess_sta + 3] == 'M'&&pBuf[mess_sta + 4] == 'I'&&pBuf[mess_sta + 5] == 'D')
                //'KHJMID'  修改客户端ID
            {
                if (len >= mess_sta + mess_len)
                {
                    factory_gateway_set[52] = mess_len - 6;
                    memcpy(factory_gateway_set + 53, pBuf + mess_sta + 6, mess_len - 6);
                    Flash_Write(0x0807B000, factory_gateway_set, 255);
                }
            }
            else if (pBuf[mess_sta] == 'D'&&pBuf[mess_sta + 1] == 'Y'&&pBuf[mess_sta + 2] == 'J'&&pBuf[mess_sta + 3] == 'M'&&pBuf[mess_sta + 4] == 'Z'&&pBuf[mess_sta + 5] == 'T')
                //'DYJMZT'修改订阅主题
            {
                if (len >= mess_sta + mess_len)
                {
                    factory_gateway_set[83] = mess_len - 6;
                    memcpy(factory_gateway_set + 84, pBuf + mess_sta + 6, mess_len - 6);
                    Flash_Write(0x0807B000, factory_gateway_set, 255);
                }
            }
            else if (pBuf[mess_sta] == 'T'&&pBuf[mess_sta + 1] == 'S'&&pBuf[mess_sta + 2] == 'J'&&pBuf[mess_sta + 3] == 'M'&&pBuf[mess_sta + 4] == 'Z'&&pBuf[mess_sta + 5] == 'T')
                //'TSJMZT'修改推送主题
            {
                if (len >= mess_sta + mess_len)
                {
                    factory_gateway_set[124] = mess_len - 6;
                    memcpy(factory_gateway_set + 125, pBuf + mess_sta + 6, mess_len - 6);
                    Flash_Write(0x0807B000, factory_gateway_set, 255);
                }
            }
            else if (pBuf[mess_sta] == 'Y'&&pBuf[mess_sta + 1] == 'H'&&pBuf[mess_sta + 2] == 'J'&&pBuf[mess_sta + 3] == 'M'&&pBuf[mess_sta + 4] == 'M'&&pBuf[mess_sta + 5] == 'Z')
                //'YHJMMZ'修改用户名
            {
                if (len >= mess_sta + mess_len)
                {
                    factory_gateway_set[155] = mess_len - 6;
                    memcpy(factory_gateway_set + 156, pBuf + mess_sta + 6, mess_len - 6);
                    Flash_Write(0x0807B000, factory_gateway_set, 255);
                }
            }
            else if (pBuf[mess_sta] == 'M'&&pBuf[mess_sta + 1] == 'M'&&pBuf[mess_sta + 2] == 'J'&&pBuf[mess_sta + 3] == 'M'&&pBuf[mess_sta + 4] == 'Y'&&pBuf[mess_sta + 5] == 'H')
                //'MMJMYH'修改用户密码
            {
                if (len >= mess_sta + mess_len)
                {
                    factory_gateway_set[188] = mess_len - 6;
                    memcpy(factory_gateway_set + 189, pBuf + mess_sta + 6, mess_len - 6);
                    Flash_Write(0x0807B000, factory_gateway_set, 255);
                }
            }
        }
    }
    return;
}

static void gprs_cmd_receive(u8 len, u8 *pBuf)   //移动TCP/IP接收CDMA命令返回处理函数
{
    if ((mqtt_bcmd = strstr((char *)pBuf, "^SISR: 0,0\r\n")) != NULL)
        //"^SISR: 0,0"表示gsm缓存区已读空，停止读取缓存区
    {
        RxReport2_step_flg = 1;
        return;
    }
    //	if((mqtt_bcmd=strstr((const char *)pBuf,"^SISR: 0,1\r\n"))!=NULL) //检测平台是否发指令,注意一定要有空格;\r\n为0x0D 0x0A;"^SISR: 0,1\r\n"表示有数据可读
    //   {
    //     if(readbuf_fast_flg==0x00)
    //		 {
    //			 readbuf_fast_flg=0x01;
    //			 //控制gsm模块缓存区读取速度，过快读取程序执行混乱。以300ms速度连续读。3ms是第一次读取速度。只有读空gsm 缓冲区才能readbuf_fast_flg=0x00
    //			 Start_timerEx(GSM_AT_READBUF,3);//向GSM模块读取平台发送的数据
    //		 }
    //    return;
    //   }
    if (strstr((char *)pBuf, "^SHUTDOWN") != NULL)while (1);  //GSM:"^SHUTDOWN"
    if (strstr((char *)pBuf, "^SIS: 0,0,") != NULL || strstr((char *)pBuf, "^SIS: 0,2,") != NULL)   //GSM:internet服务时发生了问题
    {
        cmd_flg = 0x01;//REST
        send_flg = 0x01;
        Stop_timerEx(WG_REPLY_EVT);
        Stop_timerEx(SEND_PLATFORM_EVT);
        Start_timerEx(NET_INIT_EVT, 500);
        RxReport2_step_flg = 1;
        return;
    }
    if (strstr((char *)pBuf, "ERROR") != NULL)   //网关系统重新启动，调试后恢复
    {
        if (send_message_type == 0x01 || send_message_type == 0x02 || send_message_type == 0x03 || send_message_type == 0x04)   //0x02为回复平台控制指令；0x03为平台上报数据
        {
            if (error_num < MAX_ERROR_NUM)   //出现AT指令错误，重复执行该指令，次数由MAX_ERROR_NUM决定
            {
                if (send_message_type == 0x01 && send_flg == 0x01)
                {
                    Start_timerEx(NET_INIT_EVT, 1000);	//重新执行该指令，reprot_flg没有改变
                }
                if (send_message_type == 0x02 && send_flg == 0x04)
                {
                    send_flg = 0x00;
                    send_message_type = 0x00;
                    Start_timerEx(WG_REPLY_EVT, 100);
                }
                if (send_message_type == 0x02 && send_flg == 0x05)
                {
                    send_flg = 0x04;
                    Start_timerEx(WG_REPLY_EVT, 100);
                }

                if (send_message_type == 0x03 && send_flg == 0x02)
                {
                    send_flg = 0x00;
                    send_message_type = 0x00;
                    Start_timerEx(SEND_PLATFORM_EVT, 100);
                }
                if (send_message_type == 0x03 && send_flg == 0x03)
                {
                    send_flg = 0x02;
                    Start_timerEx(SEND_PLATFORM_EVT, 100);
                }
                if (send_message_type == 0x04 && send_flg == 0x04)
                {
                    send_flg = 0x00;
                    send_message_type = 0x00;
                    Start_timerEx(JM_PLATFORM_REPLY_EVT, 100);
                }
                if (send_message_type == 0x04 && send_flg == 0x05)
                {
                    send_flg = 0x04;
                    Start_timerEx(JM_PLATFORM_REPLY_EVT, 100);
                }
                send_message_type = 0x00;
                error_num++;
                RxReport2_step_flg = 1;
                return;
            }
            else
            {
                cmd_flg = 0x01;//LINK_SERVER=0xE9
                send_flg = 0x00;
                error_num = 0x00;
                next_atdata_flg = 0;
                Stop_timerEx(WG_REPLY_EVT);
                Stop_timerEx(SEND_PLATFORM_EVT);
                Start_timerEx(NET_INIT_EVT, 500);
                RxReport2_step_flg = 1;
                return;
            }
        }
    }
    if (strstr((char *)pBuf, "^SISW: 0,1") != NULL)   //防止一条指令重复执行二遍，例如AT^SISO指令，输出OK及^SISW: 0,1,1460
    {
        RxReport2_step_flg = 1;
        return;
    }
    if (strstr((char *)pBuf, "\r\nOK\r\n") != NULL || strstr((char *)pBuf, "^SISW: 0,"))  	//"^SISW: 0,1"表示GSM模块成功接收
    {
        //0x01为cdma初始化；0x02为回复平台控制指令；0x03为平台上报数据
        error_num = 0x00;
        if (send_message_type == 0x01 && send_flg == 0x01)
        {
            cmd_flg = next_atdata_flg;
            Start_timerEx(NET_INIT_EVT, 500);
            RxReport2_step_flg = 1;
            return;
        }
        if (send_message_type == 0x03 && send_flg == 0x02)
        {
            module_send_flg = next_atdata_flg;
            Start_timerEx(SEND_PLATFORM_EVT, 10);//GSM为10，CDMA为100,发送AT指令
            RxReport2_step_flg = 1;
            return;
        }
        if (send_message_type == 0x03 && send_flg == 0x03)
        {
            module_send_flg = next_atdata_flg;
            Start_timerEx(SEND_PLATFORM_EVT, 30);//GSM为10ms,CDMA为50ms;发数据指令
            RxReport2_step_flg = 1;
            return;
        }
        if (send_message_type == 0x02 && send_flg == 0x04)
        {
            Start_timerEx(WG_REPLY_EVT, 10);//GSM为10，CDMA为100;发AT指令
            RxReport2_step_flg = 1;
            return;
        }
        if (send_message_type == 0x02 && send_flg == 0x05)
        {
            Start_timerEx(WG_REPLY_EVT, 30);//GSM为10ms；CDMA为50ms；发DATA指令
            RxReport2_step_flg = 1;
            return;
        }
        if (send_message_type == 0x04 && send_flg == 0x04)
        {
            Start_timerEx(JM_PLATFORM_REPLY_EVT, 10);//GSM为10，CDMA为100;发AT指令
            RxReport2_step_flg = 1;
            return;
        }
        if (send_message_type == 0x04 && send_flg == 0x05)
        {
            Start_timerEx(JM_PLATFORM_REPLY_EVT, 30);//GSM为10ms；CDMA为50ms；发DATA指令
            RxReport2_step_flg = 1;
            return;
        }
        RxReport2_step_flg = 1;
        return;
    }
}

static void gprs_other_receive(u8 len, u8 *pBuf)   //移动other接收平台数据
{
}

static void Ethernet_tcp_receive(u8 len, u8 *pBuf)   //移动TCP/IP接收平台数据
{
    u8 k = 0;

    /*平台控制指令*/
    for (k = 13; k < len; k++)
        //mqtt-CDMA在0xCC前面有43个字节，k=43（订阅主题长度为15个字节不变,增加或减少订阅主题字节数，则相应修改k值）
        //TCP-CDMA 在0xCC前面有AT指令11个字节+2个字节（长度），k=13 ;这样可以保证前面的字符串中有0xCC也没有关系。
        //mqtt-GSM在0xCC前面有43个字节，k=43（订阅主题长度为15个字节不变,增加或减少订阅主题字节数，则相应修改k值）
        //TCP-GSM 在0xCC前面AT指令有13个字节（其中2个字节是长度），k=13 ;这样可以保证前面的字符串中有0xCC也没有关系。
    {
        if (pBuf[k] == 0xCC)   //上报平台协议规定的帧头
        {
            if (pBuf[k - 3] == real_send[7] && pBuf[k - 2] == real_send[8] && pBuf[k - 1] == real_send[9])   //采控器ID
            {
                handlecmd(pBuf + k - 10, (((u16)pBuf[k - 5]) << 8) + pBuf[k - 6] + 6);
                //处理平台下发的控制指令，以0xCC为标准得到完整的平台指令（0x01,0x00,...,0xDD）,长度为完整指令长度
                RxReport2_step_flg = 2;
                return;
            }
        }
    }
}
static void Ethernet_mqtt_receive(u8 len, u8 *pBuf)   //移动mqtt接收平台数据
{
}
static void Ethernet_sdk_receive(u8 len, u8 *pBuf)   //移动sdk接收平台数据
{
    mqtt_bcmdxb = match_str(pBuf, len, (unsigned char *)"{\"method\":\"", 11);
    if (factory_gateway_set[1] == 4)mqtt_bcmdxb = match_str(pBuf, len, (unsigned char *)"{\"command\":\"", 12);//测试
    if (mqtt_bcmdxb != 0)
    {
        memcpy(ctrl_key, pBuf + mqtt_bcmdxb + 1, 4);
        memcpy(ctrl_value, pBuf + mqtt_bcmdxb + 1 + 16, 4);
        //	 if(factory_gateway_set[1]==4)memcpy(ctrl_value,pBuf+mqtt_bcmdxb+16,4);//测试
        /*平台控制指令 {"method":"2100","params":"0001"}*/
        ctrl_adrr = dword_asc_hex(ctrl_key);//将4个字节的ASC码转换成十六进制数0x2100
        ctrl_cmd = dword_asc_hex(ctrl_value);
        if (ctrl_adrr >= 0x2100 && ctrl_adrr < 0x2300)
        {
            offset_addrX = (ctrl_adrr - 0x2100) / 16;
            offset_addrY = (ctrl_adrr & 0x000F) / 2;
            Controllers[offset_addrX][offset_addrY] = ctrl_cmd & 0x00FF;
            Controllers[offset_addrX][offset_addrY + 1] = (ctrl_cmd >> 8) & 0x00FF;
            crtl_cmd_num[offset_addrX][offset_addrY / 2] = 50;//子站有线发送记录，允许重复发送3次，收到清0，不再继续发送。
            crtl_cmd_numWX[offset_addrX][offset_addrY / 2] = 50;//子站无线发送记录，允许重复发送3次，收到清0，不再继续发送。
            memset(sdk_ctrl_reply, 0, sizeof(sdk_ctrl_reply));//上一次出现错误ERROR或其它返回值有可能保留在数组中，所以需要清零
            sdk_ctrl_reply[0] = len;
            memcpy(sdk_ctrl_reply + 1, pBuf, len);
            Start_timerEx(WG_REPLY_EVT, 150);
            RxReport2_step_flg = 2;
            return;
        }
        if (ctrl_adrr >= 0x2300 && ctrl_adrr < 0x2500)
        {
            return;
        }
        if (ctrl_adrr >= 0x2500 && ctrl_adrr < 0x2620)
        {
            offset_addrX = (ctrl_adrr - 0x2500) / 4;
            hand_auto_flg[offset_addrX][0] = ctrl_cmd & 0x00FF;
            hand_auto_flg[offset_addrX][1] = (ctrl_cmd >> 8) & 0x00FF;
            hand_auto_count[offset_addrX] = 5;//向平台发送手动-自动状态记录，允许重复发送5次。
            memset(sdk_ctrl_reply, 0, sizeof(sdk_ctrl_reply));//上一次出现错误ERROR或其它返回值有可能保留在数组中，所以需要清零
            sdk_ctrl_reply[0] = len;
            memcpy(sdk_ctrl_reply + 1, pBuf, len);
            Start_timerEx(WG_REPLY_EVT, 150);
            RxReport2_step_flg = 2;
            return;
        }
    }
}
static void Ethernet_other_receive(u8 len, u8 *pBuf)   //移动other接收平台数据
{
}

//回复平台控制命令及查询
static void wg_reply_cmd(void)   //回复平台控制命令及查询
{
    switch (factory_gateway_set[0])   //网络类型=1 电信，2 移动，3 以太网，4 WiFi，5 USB，6 无连接；
    {
    //电信 cdma 开始---------------------------------
    case 0x01:
        switch (factory_gateway_set[1])   //协议类型=1 TCP/IP;2 MQTT; 3 SDK; 4 其它；
        {
        //电信TCP/IP
        case 0x01:
            cdma_tcp_reply();
            break;
        //电信MQTT
        case 0x02:
            cdma_mqtt_reply();
            break;
        //电信SDK
        case 0x03:
            cdma_sdk_reply();
            break;
        //电信other
        case 0x04:
            cdma_other_reply();
            break;
        default:
            break;
        }
        break;
    //电信 cdma 结束------------------------------------

    //移动 gprs	开始====================================
    case 0x02:
        switch (factory_gateway_set[1])   //协议类型=1 TCP/IP;2 MQTT; 3 SDK; 4 其它；
        {
        //移动TCP/IP
        case 0x01:
            cdma_tcp_reply();//包含gprs_tcp_reply
            break;
        //移动MQTT
        case 0x02:
            cdma_mqtt_reply();//包含gprs_mqtt_reply
            break;
        //移动SDK
        case 0x03:
            cdma_sdk_reply();//包含gprs_sdk_reply
            break;
        //移动other
        case 0x04:
            gprs_other_reply();
            break;
        default:
            break;
        }
        break;
    //移动 gprs	结束====================================

    //Ethernet 开始---------------------------------
    case 0x03:
        switch (factory_gateway_set[1])   //协议类型=1 TCP/IP;2 MQTT; 3 SDK; 4 其它；
        {
        //Ethernet TCP/IP
        case 0x01:
            Ethernet_tcp_reply();
            break;
        //Ethernet MQTT
        case 0x02:
            Ethernet_mqtt_reply();
            break;
        //Ethernet SDK
        case 0x03:
            Ethernet_sdk_reply();
            break;
        //Ethernet other
        case 0x04:
            Ethernet_other_reply();
            break;
        default:
            break;
        }
        break;
    //Ethernet 结束------------------------------------

    //WiFi 开始&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    case 0x04:
        break;

    //WiFi 结束&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    //USB  开始########################################
    case 0x05:

        break;
    //USB 结束#########################################
    default:
        break;
    }
}

static void cdma_tcp_reply(void)   //电信TCP/IP回复平台控制命令及查询
{
    if (send_mess)  		//正在发送短信上报，等待50ms后执行
    {
        Start_timerEx(WG_REPLY_EVT, 50);
        return;
    }
    //	if(send_flg ==0x01||send_flg ==0x02||send_flg ==0x03)
    // 	{
    //		Start_timerEx(WG_REPLY_EVT,500);
    //		return;
    //	}
    if (send_message_type != 0x02 && send_message_type != 0x00)   //对子站初始化工作完成ZZ_Wired_flag[65]=1；||ZZ_Wireles_flag[65]==1
    {
        Start_timerEx(WG_REPLY_EVT, 500);
        return;
    }
    send_message_type = 0x02;	//在收到GSM的ERROR报告时使用，0x02类型为回复平台的控制指令
    if (send_flg == 0x00)
    {
        //send_at_cdma("046");//发送主题长度不能变，如果增加，则加相应的字节数(最长为99);publish打包结束的数字为mqtt_real_send
        if (factory_gateway_set[0] == 1)send_at_cdma((unsigned char *)"027", 21);//TCB,CDMA	长度为31=6+信息长度=6+25=31；CDMA的AT指令长度为21；GPRS的AT指令长度为15
        if (factory_gateway_set[0] == 2)send_at_gprs((unsigned char *)"027", 15);//TCB,GPRS  TCP 长度为43=6+信息长度=6+37=43；CDMA的AT指令长度为21；GPRS的AT指令长度为15
        //TCB	长度为31=6+信息长度=6+21=27（特别说明：其它程序都为29）；CDMA的AT指令长度为21；GPRS的AT指令长度为15（特别说明：其它程序都为29）
        send_flg = 0x04;
        Start_timerEx(WG_REPLY_EVT, CMD_WAIT_TIME);
        return;
    }
    if (send_flg == 0x04)
    {
        //mqtt_publish(reply_xiafa_data,27);//mqtt数据包长度为46，MQTT头有19个字节，控制返回命令(下发的控制命令)27个字节（特别说明：其它程序都为29）
        //send_data_module(mqtt_real_send,"046");//（特别说明：其它程序都为29）
        send_data_module(reply_xiafa_data, (unsigned char *)"027");//TCB（特别说明：其它程序都为29）
        send_flg = 0x05;
        Start_timerEx(WG_REPLY_EVT, CMD_WAIT_TIME);
        return;
    }
    if (send_flg == 0x05)
    {
        send_flg = 0x00;
        send_message_type = 0x00;
        return;
    }
    send_flg = 0x00;
    send_message_type = 0x00;
    return;
}

static void cdma_mqtt_reply(void)   //电信mqtt回复平台控制命令及查询
{
    if (send_mess)  		//正在发送短信上报，等待50ms后执行
    {
        Start_timerEx(WG_REPLY_EVT, 50);
        return;
    }
    //	if(send_flg ==0x01||send_flg ==0x02||send_flg ==0x03)
    // 	{
    //		Start_timerEx(WG_REPLY_EVT,500);
    //		return;
    //	}
    if (send_message_type != 0x02 && send_message_type != 0x00)   //对子站初始化工作完成ZZ_Wired_flag[65]=1；||ZZ_Wireles_flag[65]==1
    {
        Start_timerEx(WG_REPLY_EVT, 500);
        return;
    }
    send_message_type = 0x02;	//在收到GSM的ERROR报告时使用，0x02类型为回复平台的控制指令
    if (send_flg == 0x00)
    {
        send_message_len2 = 6 + 18 + 3;//6+18+3
        mqtt_publish(reply_xiafa_data, send_message_len2);//MQTT 实际发送75个字节(不包括推送主题)。
        bytelen_to_asc((unsigned char *)message_len_char2, mqtt_len);//mqtt_len全局变量；在mqtt_publish函数中赋值
        if (factory_gateway_set[0] == 1)send_at_cdma((unsigned char *)message_len_char2, 21);//TCB,CDMA	长度为31=6+信息长度=6+25=31；CDMA的AT指令长度为21；GPRS的AT指令长度为15
        if (factory_gateway_set[0] == 2)send_at_gprs((unsigned char *)message_len_char2, 15);//TCB,GPRS  TCP 长度为43=6+信息长度=6+37=43；CDMA的AT指令长度为21；GPRS的AT指令长度为15
//send_at_cdma("046");//发送主题长度不能变，如果增加，则加相应的字节数(最长为99);publish打包结束的数字为mqtt_real_send
//	 send_at_cdma((unsigned char *)"027",21);//TCB	长度为31=6+信息长度=6+21=27（特别说明：其它程序都为29）；CDMA的AT指令长度为21；GPRS的AT指令长度为15（特别说明：其它程序都为29）
        send_flg = 0x04;
        Start_timerEx(WG_REPLY_EVT, CMD_WAIT_TIME);
        return;
    }
    if (send_flg == 0x04)
    {
        send_message_len2 = 6 + 18 + 3;//6+18+3=27 收到的控制命令长度
        mqtt_publish(reply_xiafa_data, send_message_len2);//MQTT 实际发送75个字节(不包括推送主题)。
        bytelen_to_asc((unsigned char *)message_len_char2, mqtt_len);//mqtt_len全局变量；在mqtt_publish函数中赋值
        //mqtt数据包长度为46，MQTT头有19个字节，控制返回命令(下发的控制命令)27个字节（特别说明：其它程序都为29）
        send_data_module(ReportData2, (unsigned char *)message_len_char2);//（特别说明：其它程序都为29）
// send_data_module(reply_xiafa_data,(unsigned char *)"027");//TCB（特别说明：其它程序都为29）
        send_flg = 0x05;
        Start_timerEx(WG_REPLY_EVT, CMD_WAIT_TIME);
        return;
    }
    if (send_flg == 0x05)
    {
        send_flg = 0x00;
        send_message_type = 0x00;
        return;
    }
    send_flg = 0x00;
    send_message_type = 0x00;
    return;
}

static void cdma_sdk_reply(void)   //电信sdk回复平台控制命令及查询
{
    if (send_mess)  		//正在发送短信上报，等待50ms后执行
    {
        Start_timerEx(WG_REPLY_EVT, 50);
        return;
    }
    //	if(send_flg ==0x01||send_flg ==0x02||send_flg ==0x03)
    // 	{
    //		Start_timerEx(WG_REPLY_EVT,500);
    //		return;
    //	}
    if (send_message_type != 0x02 && send_message_type != 0x00)   //对子站初始化工作完成ZZ_Wired_flag[65]=1；||ZZ_Wireles_flag[65]==1
    {
        Start_timerEx(WG_REPLY_EVT, 500);
        return;
    }
    send_message_type = 0x02;	//在收到GSM的ERROR报告时使用，0x02类型为回复平台的控制指令
    if (send_flg == 0x00)
    {
        u8 sdk_bcmdxb1;
        //		sdk_len=made_keyX_value(0x2100+offset_addrX*4*4,Controllers[offset_addrX],4,real_send);
        //0x2100控制器控制状态地址，上报1个控制器4个状态参数 ;offset_addrX在cdma_sdk_receive函数中设置
        sdk_bcmdxb = match_str(sdk_ctrl_reply, sdk_ctrl_reply[0], (unsigned char *)"{\"method\":", 10);//匹配返回的地址是‘：’的地址；所以下面源地址加2
        memcpy(jm_reply_cmd + 1, sdk_ctrl_reply + sdk_bcmdxb + 1, 5);//{"method":"2100","params":"0009"},为极码平台命令回复配置
        memcpy(jm_reply_cmd + 7, sdk_ctrl_reply + sdk_bcmdxb + 18, 4);//回复命令"{H2100:0009}"，为极码平台命令回复配置
        sdk_bcmdxb1 = match_str(sdk_ctrl_reply + sdk_bcmdxb, sdk_ctrl_reply[0] - sdk_bcmdxb, (unsigned char *)"}", 1);
        sdk_len = sdk_bcmdxb1 + 10;//sdk_bcmdxb1为命令有效字节数；10为{"method":的字节数
        memset(real_send, 0, sizeof(real_send));//上次返回值有可能保留在数组中，所以需要清零
        memcpy(real_send, sdk_ctrl_reply + sdk_bcmdxb - 10, sdk_len);

        /*以下为publish响应命令主题的提取和修改*/
        sdk_bcmdxb = match_str(sdk_ctrl_reply, sdk_ctrl_reply[0], (unsigned char *)"v1/devices/me/rpc/request/", 26);
        //sdk_ctrl_reply[0]接收命令长度，sdk_ctrl_reply[1]后面是命令帧
        memset(SDK_topicString_pub1 + 27, 0, 128 - 27);//上次返回值有可能保留在数组中，所以需要清零
        memcpy(SDK_topicString_pub1 + 27, sdk_ctrl_reply + sdk_bcmdxb, sdk_ctrl_reply[sdk_bcmdxb - 26 - 1] - 26);//sdk_ctrl_reply[sdk_bcmdxb-1]推送主题长度
        //27为 v1/devices/me/rpc/response/长度；26为v1/devices/me/rpc/request/长度；主题为v1/devices/me/rpc/request/1061 其中1061为requestID，长度可变
        mqtt_publish1(real_send, sdk_len);//该推送函数为SDK特别指定的，因为规定了控制器状态上报的推送主题
        bytelen_to_asc((unsigned char *)message_len_char2, mqtt_len);//mqtt_len为全局变量，在mqtt_publish1函数中赋值
        if (factory_gateway_set[0] == 1)send_at_cdma((unsigned char *)message_len_char2, 21);//TCB,CDMA	长度为31=6+信息长度=6+25=31；CDMA的AT指令长度为21；GPRS的AT指令长度为15
        if (factory_gateway_set[0] == 2)send_at_gprs((unsigned char *)message_len_char2, 15);//TCB,GPRS  TCP 长度为43=6+信息长度=6+37=43；CDMA的AT指令长度为21；GPRS的AT指令长度为15
        send_flg = 0x04;
        Start_timerEx(WG_REPLY_EVT, CMD_WAIT_TIME);
        return;
    }
    if (send_flg == 0x04)
    {
        //		sdk_len=made_keyX_value(0x2100+offset_addrX*4*4,Controllers[offset_addrX],4,real_send);
        //0x2100控制器控制状态地址，上报1个控制器4个状态参数 ;offset_addrX在cdma_sdk_receive函数中设置
        mqtt_publish1(real_send, sdk_len);//该推送函数为SDK特别指定的，因为规定了控制器状态上报的推送主题
        bytelen_to_asc((unsigned char *)message_len_char2, mqtt_len);//mqtt_len为全局变量，在mqtt_publish1函数中赋值
        send_data_module(ReportData2, (unsigned char *)message_len_char2);//ReportData2为实际发送内容，在mqtt_publish函数中包装
        send_flg = 0x05;
        Start_timerEx(WG_REPLY_EVT, CMD_WAIT_TIME);
        return;
    }
    if (send_flg == 0x05)
    {
        send_flg = 0x00;
        send_message_type = 0x00;
        Start_timerEx(JM_PLATFORM_REPLY_EVT, 300);
        return;
    }
    send_flg = 0x00;
    send_message_type = 0x00;
    return;
}

static void jm_platform_reply(void)   //极码平台快速返回控制命令
{
    if (send_mess)  		//正在发送短信上报，等待50ms后执行
    {
        Start_timerEx(JM_PLATFORM_REPLY_EVT, 50);
        return;
    }

    if (send_message_type != 0x04 && send_message_type != 0x00)   //对子站初始化工作完成ZZ_Wired_flag[65]=1；||ZZ_Wireles_flag[65]==1
    {
        Start_timerEx(JM_PLATFORM_REPLY_EVT, 500);
        return;
    }
    send_message_type = 0x04;	//在收到GSM的ERROR报告时使用，0x04类型为快速回复极码平台的控制指令
    if (send_flg == 0x00)
    {
        //		  memset(real_send, 0,sizeof(real_send));//上次返回值有可能保留在数组中，所以需要清零
        //		  memcpy(real_send,jm_reply_cmd,12);//回复命令"{H2100:0009}"
        mqtt_publish(jm_reply_cmd, 12);
        bytelen_to_asc((unsigned char *)message_len_char1, mqtt_len);//mqtt_len为全局变量，在mqtt_publish11函数中赋值
        if (factory_gateway_set[0] == 1)send_at_cdma((unsigned char *)message_len_char1, 21);//TCB,CDMA	长度为31=6+信息长度=6+25=31；CDMA的AT指令长度为21；GPRS的AT指令长度为15
        if (factory_gateway_set[0] == 2)send_at_gprs((unsigned char *)message_len_char1, 15);//TCB,GPRS  TCP 长度为43=6+信息长度=6+37=43；CDMA的AT指令长度为21；GPRS的AT指令长度为15
        send_flg = 0x04; //表示向平台上报的at指令正在执行，下面只能执行data指令
        Start_timerEx(JM_PLATFORM_REPLY_EVT, CMD_WAIT_TIME);
        return;
    }
    if (send_flg == 0x04)
    {
        mqtt_publish(jm_reply_cmd, 12);//该推送函数为SDK特别指定的，因为规定了控制器状态上报的推送主题
        bytelen_to_asc((unsigned char *)message_len_char2, mqtt_len);//mqtt_len为全局变量，在mqtt_publish函数中赋值
        send_data_module(ReportData2, (unsigned char *)message_len_char2);//ReportData2为实际发送内容，在mqtt_publish函数中包装
        send_flg = 0x05;
        Start_timerEx(JM_PLATFORM_REPLY_EVT, CMD_WAIT_TIME);
        return;
    }
    if (send_flg == 0x05)
    {
        send_flg = 0x00;
        send_message_type = 0x00;
        return;
    }
    send_flg = 0x00;
    send_message_type = 0x00;
    return;
}

static void cdma_other_reply(void)   //电信other回复平台控制命令及查询
{
}

static void gprs_other_reply(void)   //移动other回复平台控制命令及查询
{
}

static void Ethernet_tcp_reply(void)   //Ethernet TCP/IP回复平台控制命令及查询
{
    if (send_mess)  		//正在发送短信上报，等待50ms后执行
    {
        Start_timerEx(WG_REPLY_EVT, 50);
        return;
    }

    if (send_message_type != 0x02 && send_message_type != 0x00)   //对子站初始化工作完成ZZ_Wired_flag[65]=1；||ZZ_Wireles_flag[65]==1
    {
        Start_timerEx(WG_REPLY_EVT, 500);
        return;
    }
    send_message_type = 0x02;	//在收到GSM的ERROR报告时使用，0x02类型为回复平台的控制指令
    if (send_flg == 0x00)
    {
        send_flg = 0x04;
        Start_timerEx(WG_REPLY_EVT, Ethernet_WAIT_TIME);
        return;
    }
    if (send_flg == 0x04)
    {
        //mqtt_publish(reply_xiafa_data,27);//mqtt数据包长度为46，MQTT头有19个字节，控制返回命令(下发的控制命令)27个字节（特别说明：其它程序都为29）
        //send_data_module(mqtt_real_send,"046");//（特别说明：其它程序都为29）
        send_Ethernet_module(reply_xiafa_data, (unsigned char *)"027");//TCB（特别说明：其它程序都为29）
        send_flg = 0x05;
        Start_timerEx(WG_REPLY_EVT, Ethernet_WAIT_TIME);
        return;
    }
    if (send_flg == 0x05)
    {
        send_flg = 0x00;
        send_message_type = 0x00;
        return;
    }
    send_flg = 0x00;
    send_message_type = 0x00;
    return;
}
static void Ethernet_mqtt_reply(void)   //Ethernet mqtt回复平台控制命令及查询
{
}
static void Ethernet_sdk_reply(void)   //Ethernet sdk回复平台控制命令及查询
{
    if (send_mess)  		//正在发送短信上报，等待50ms后执行
    {
        Start_timerEx(WG_REPLY_EVT, 50);
        return;
    }

    if (send_message_type != 0x04 && send_message_type != 0x00)   //对子站初始化工作完成ZZ_Wired_flag[65]=1；||ZZ_Wireles_flag[65]==1
    {
        Start_timerEx(WG_REPLY_EVT, 500);
        return;
    }
    send_message_type = 0x04;	//在收到GSM的ERROR报告时使用，0x04类型为快速回复极码平台的控制指令
    if (send_flg == 0x00)
    {
        //		  memset(real_send, 0,sizeof(real_send));//上次返回值有可能保留在数组中，所以需要清零
        //		  memcpy(real_send,jm_reply_cmd,12);//回复命令"{H2100:0009}"
        send_flg = 0x04; //表示向平台上报的at指令正在执行，下面只能执行data指令
        Start_timerEx(WG_REPLY_EVT, Ethernet_WAIT_TIME);
        return;
    }
    if (send_flg == 0x04)
    {
        mqtt_publish(jm_reply_cmd, 12);//该推送函数为SDK特别指定的，因为规定了控制器状态上报的推送主题
        bytelen_to_asc((unsigned char *)message_len_char2, mqtt_len);//mqtt_len为全局变量，在mqtt_publish函数中赋值
        send_Ethernet_module(ReportData2, (unsigned char *)message_len_char2);//ReportData2为实际发送内容，在mqtt_publish函数中包装
        send_flg = 0x05;
        Start_timerEx(JM_PLATFORM_REPLY_EVT, Ethernet_WAIT_TIME);
        return;
    }
    if (send_flg == 0x05)
    {
        send_flg = 0x00;
        send_message_type = 0x00;
        return;
    }
    send_flg = 0x00;
    send_message_type = 0x00;
    return;
}
static void Ethernet_other_reply(void)   //Ethernet other回复平台控制命令及查询
{
}

/*向平台发送数据函数定义*/
static void send_at_cdma(unsigned char send_data_len[], u8 at_cmd_len)  	//发送数据的at指令到CDMA;CDMA为at_cmd_len=21;GSM为at_cmd_len=15
{
    at_send_cdma[16] = send_data_len[0];//CDMA为at_send[16]~[18];GSM为at_send[10]~[12]
    at_send_cdma[17] = send_data_len[1];
    at_send_cdma[18] = send_data_len[2];
    //	       WriteDataToBuffer(2,at_send,0,at_cmd_len); //发送AT指令;CDMA为at_cmd_len=21;GSM为at_cmd_len=15
    memcpy(USART2SendTCB, at_send_cdma, at_cmd_len);
    WriteDataToDMA_BufferTX2(at_cmd_len);
}
static void send_at_gprs(unsigned char send_data_len[], u8 at_cmd_len)  	//发送数据的at指令到通讯模块，CDMA：at_cmd_len=21，GPRS:at_cmd_len=15
{
    at_send_gprs[10] = send_data_len[0];//CDMA为at_send[16]~[18]；
    at_send_gprs[11] = send_data_len[1];
    at_send_gprs[12] = send_data_len[2];
    memcpy(USART2SendTCB, at_send_gprs, at_cmd_len);
    WriteDataToDMA_BufferTX2(at_cmd_len);
}

static void send_data_module(unsigned char send_data[], unsigned char send_data_len[])  	//发送数据到CDMA
{
    u8 temp_len;
    temp_len = (send_data_len[0] - 0x30) * 100 + (send_data_len[1] - 0x30) * 10 + (send_data_len[2] - 0x30);//将发送长度ASC码转换成16进制数
//	       WriteDataToBuffer(2,send_data,0,temp_len); //发送数据
    memcpy(USART2SendTCB, send_data, temp_len);//ReportData2
    WriteDataToDMA_BufferTX2(temp_len);
}

static void send_Ethernet_module(unsigned char send_data[], unsigned char send_data_len[])  	//发送数据到Ethernet模块;
{
    u8 temp_len;
    temp_len = (send_data_len[0] - 0x30) * 100 + (send_data_len[1] - 0x30) * 10 + (send_data_len[2] - 0x30);//将发送长度ASC码转换成16进制数
    memcpy(USART1SendTCB, send_data, temp_len);//ReportData2
    WriteDataToDMA_BufferTX1(temp_len);
}

static void alter_send(u8 n, u8 flg)   //alter_hand_auto(0x04,sendnum_mflg)
{
    u16 CRC_Value;
    u8 i = 0;

    u16 _lastaddr = real_send[20] + (((u16)real_send[21]) << 8);
    _lastaddr = _lastaddr + (n*flg);//上报时，2*2*4=16，每个电流传感器数据为4个字节，前2个字节为0
    real_send[20] = _lastaddr & 0xFF;
    real_send[21] = _lastaddr >> 8;

    for (i = 0; i < n / 2; i = i + 2)  	//01 00 00 00 65 00 02 02 02 03 cc 01 02 03 04 05 06 07 64 04 00 21 50 00 ........cr cr dd
    {
        real_send[24 + i * 2] = 0x00;
        real_send[25 + i * 2] = 0x00;
        real_send[26 + i * 2] = Controllers[flg][i];//8(采控器ID)+1(命令)+2(起始地址)+2(字节数长度)=13+16(字节数)=29
        real_send[27 + i * 2] = Controllers[flg][i + 1];
    }

    CRC_Value = GetCRC16(real_send + 11, 13 + n);//8(采控器ID)+1(命令)+2(起始地址)+2(字节数长度)=13+16(值的字节数)=29

    real_send[24 + n] = CRC_Value & 0x00FF;//24+16(值的字节数)=40
    real_send[25 + n] = (CRC_Value >> 8);
    real_send[26 + n] = 0xDD;
    real_send[27 + n] = flg;
}

static void alter_hand_auto(u8 n, u8 flg)
{
    u16 CRC_Value;
    u8 i = 0;

    u16 _lastaddr = real_send[20] + (((u16)real_send[21]) << 8);
    _lastaddr = _lastaddr + (n*flg);//上报时，2*2*4=16，每个电流传感器数据为4个字节，前2个字节为0
    real_send[20] = _lastaddr & 0xFF;
    real_send[21] = _lastaddr >> 8;

    for (i = 0; i < n / 2; i = i + 2)  	//01 00 00 00 65 00 02 02 02 03 cc 01 02 03 04 05 06 07 64 04 00 21 50 00 ........cr cr dd
    {
        real_send[24 + i * 2] = 0x00;
        real_send[25 + i * 2] = 0x00;
        real_send[26 + i * 2] = hand_auto_flg[flg][i];//8(采控器ID)+1(命令)+2(起始地址)+2(字节数长度)=13+16(字节数)=29
        real_send[27 + i * 2] = hand_auto_flg[flg][i + 1];
    }

    CRC_Value = GetCRC16(real_send + 11, 13 + n);//8(采控器ID)+1(命令)+2(起始地址)+2(字节数长度)=13+16(值的字节数)=29

    real_send[24 + n] = CRC_Value & 0x00FF;//24+16(值的字节数)=40
    real_send[25 + n] = (CRC_Value >> 8);
    real_send[26 + n] = 0xDD;
    real_send[27 + n] = flg;
}

/*特别说明
每个采集器有6组参数，共12个字节，余下4个字节保留；控制器有8个采集参数，共16个字节
控制器8个参数：1号卷膜器电流，1号卷膜器最大电流，2号卷膜器电流，2号卷膜器最大电流，。。。
*/

static void chge_coltsnd(u8 flg)     //75字节
{
    u16 i = 0;
    u16 CRC_Value;

    u16 _lastaddr = real_send[20] + (((u16)real_send[21]) << 8);
    _lastaddr = _lastaddr + (0x0020 * flg);//上报时，2*2*8=32，每个采集数据为4个字节，前2个字节为0
    real_send[20] = _lastaddr & 0xFF;
    real_send[21] = _lastaddr >> 8;

    for (i = 0; i < 16; i = i + 2)   //1个采集器8个参数16个字节转换为上报的32个字节
    {
        real_send[24 + 2 * i] = 0x00;
        real_send[25 + 2 * i] = 0x00;
        real_send[26 + 2 * i] = Collectors[flg][i];
        real_send[27 + 2 * i] = Collectors[flg][i + 1];
    }

    CRC_Value = GetCRC16(real_send + 11, 45);//8(采控器ID)+1(命令)+2(起始地址)+2(字节数长度)=13+24+8(字节值)=45
    real_send[56] = CRC_Value & 0x00FF;//24+24(值的字节数)=48
    real_send[57] = ((CRC_Value & 0xFF00) >> 8);
    real_send[58] = 0xDD;
    real_send[59] = flg;
}

static void chge_fertisnd(u8 flg)     //75字节
{
    u16 i = 0;
    u16 CRC_Value;

    u16 _lastaddr = real_send[20] + (((u16)real_send[21]) << 8);
    _lastaddr = _lastaddr + (0x0020 * flg);//上报时，2*2*8=32，每个采集数据为4个字节，前2个字节为0
    real_send[20] = _lastaddr & 0xFF;
    real_send[21] = _lastaddr >> 8;

    for (i = 0; i < 16; i = i + 2)   //1个采集器8个参数16个字节转换为上报的32个字节
    {
        real_send[24 + 2 * i] = 0x00;
        real_send[25 + 2 * i] = 0x00;
        real_send[26 + 2 * i] = reportfertigation52[flg][i];
        real_send[27 + 2 * i] = reportfertigation52[flg][i + 1];
    }

    CRC_Value = GetCRC16(real_send + 11, 45);//8(采控器ID)+1(命令)+2(起始地址)+2(字节数长度)=13+24+8(字节值)=45
    real_send[56] = CRC_Value & 0x00FF;//24+24(值的字节数)=48
    real_send[57] = ((CRC_Value & 0xFF00) >> 8);
    real_send[58] = 0xDD;
    real_send[59] = flg;
}

static void handlecmd(u8 *reply_xiafa_cmd, u16 len)
//处理平台下发的控制指令，reply_xiafa_cmd是一个完整的平台指令（0x01,0x00,...,0xDD）,len长度为完整指令长度
{
    u16 CRC_Value = 0x0000;

    if (reply_xiafa_cmd[len - 1] != 0xDD) 		return;
    switch (reply_xiafa_cmd[19])   //平台控制命令0x06
    {
    case 0x06://平台下发控制指令
        CRC_Value = GetCRC16(reply_xiafa_cmd + 11, len - 14);
        if (reply_xiafa_cmd[len - 3] == ((CRC_Value & 0x00FF)) && reply_xiafa_cmd[len - 2] == ((CRC_Value & 0xFF00) >> 8))
        {
            set_ctrl_data(reply_xiafa_cmd + 20, reply_xiafa_cmd + 22, 2);//给控制器发送数据的数组进行设置
            memcpy(reply_xiafa_data, reply_xiafa_cmd, len);
            Start_timerEx(WG_REPLY_EVT, 150);
        }
        break;
    default:
        break;
    }
    return;
}

/*set_ctrl_data(平台下发控制指令地址，指令内容，2)，例如：地址为2100H(00,21);内容为0001H(01,00)
每个控制器有4个通道及4个限流参数设定，每个通道占4个字节,如:第1个控制器的控制命令地址为:2100H,2104H,2108H,210CH;
第2个控制器命令为:2110H,2114H,2118H,211CH；
控制器编号为0~31，每个控制器的起始地址为：2100H+10H*控制器编号
2100H~22FCH平台控制命令
2300H-24FCH为4个通道的限流参数；暂时不配置（本软件无此功能）
2500H-25FCH为64个单自控回路；2600H-2620H为9个批量自控回路。
*/
static void set_ctrl_data(u8 *_addr, u8 *_data, u8 len)  	//len==2
{
    u16 PT_address, dis = 0, rnd = 0, rem = 0;//2100H~22FCH平台控制命令
    PT_address = _addr[0] + ((u16)_addr[1] << 8);
    if (PT_address >= 0x2100 && PT_address < 0x2300)
    {
        dis = ((PT_address - 0x2100) >> 2);//dis=以控制器起始地址2100h开始计算偏移值/4

        rnd = dis >> 2;						//rnd=dis/2;rnd为控制器的统一编号,0~31
        rem = (dis % 4) << 1;		//rem=(dis%8)*2; /rem为rnd编号的卷膜控制器内部参数编号(0~3)*2,(0~3为控制命令)

        if (rnd < 32 && rem <= 8)
        {
            Controllers[rnd][rem] = _data[0];//控制指令最低位（如风机开指令编码：00 01,发送排列为：最低 01 00 最高）
            Controllers[rnd][rem + 1] = _data[1];
            crtl_cmd_num[rnd][rem / 2] = 50;//子站有线发送记录，允许重复发送3次，收到清0，不允许继续发送。
            crtl_cmd_numWX[rnd][rem / 2] = 50;//子站无线发送记录，允许重复发送3次，收到清0，不允许继续发送。
        }
        return;
    }
    if (PT_address >= 0x2300 && PT_address < 0x2500)
    {
        return;
    }
    if (PT_address >= 0x2500 && PT_address < 0x2620)
    {
        dis = ((PT_address - 0x2500) >> 2);//dis=以自控回路起始地址2500h开始计算偏移值/4
        hand_auto_flg[dis][0] = _data[0];
        hand_auto_flg[dis][1] = _data[1];
        hand_auto_count[dis] = 5;//手自动状态发生变化，5次回复平台
        return;
    }
}

//短信函数定义
/*如果数组里全为数字，返回真*/
static bool is_number(u8 *sorce, u8 len)
{
    u8 i, temp_flase_true;
    //  memcpy(test,sorce,len);
    for (i = 0; i < len; i++)
    {
        temp_flase_true = (sorce[i] != 0x2E && sorce[i] != 0x3A && sorce[i] != 0x2C);
        if ((sorce[i] < 0x30 || sorce[i] > 0x39) && temp_flase_true)
        {
            return false;
        }
    }
    return true;
}

/*如果匹配返回相同段的最后一位的地址，否则返回0*/
static u8 match_str(u8 *dst, u8 dst_len, u8 *sor, u8 sor_len)
{
    u8 _lastSame = 0;
    u8 i = 0, j = 0;
    bool _same = false;
    for (i = 0; i < dst_len; i++)
    {
        if (dst[i] == sor[0])
        {
            _same = true;
            for (j = 1; j < sor_len; j++)
            {
                if (dst[i + j] != sor[j])
                {
                    _same = false;
                    j = sor_len;
                }
            }
            if (_same)
            {
                _lastSame = i + sor_len;
                i = dst_len;
            }
        }
    }
    return _lastSame;
}

//mqtt函数
void  mqtt_connect(void)
{
    char char_string_clientID[40];
    char char_string_username[40];
    char char_string_password[40];
    char char_string_topicName[50];
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    memset(char_string_clientID, 0, sizeof(char_string_clientID));
    memcpy(char_string_clientID, factory_gateway_set + 53, factory_gateway_set[52]); //客户端ID
    data.clientID.cstring = char_string_clientID;
    data.keepAliveInterval = 900;
    data.cleansession = 1;
    data.willFlag = 0;
    memset(char_string_username, 0, sizeof(char_string_username));
    memcpy(char_string_username, factory_gateway_set + 156, factory_gateway_set[155]); //用户名
    data.username.cstring = char_string_username;
    if (factory_gateway_set[188] != 0)
    {
        memset(char_string_password, 0, sizeof(char_string_password));
        memcpy(char_string_password, factory_gateway_set + 189, factory_gateway_set[188]);	//使用密码
        data.password.cstring = char_string_password; //密码
    }
    if (factory_gateway_set[1] == 4)
    {
        //    	memset(char_string_username,0,sizeof(char_string_username));// 测试
        //			memcpy(char_string_username,"testCode1-DTU-20170925-000068",29);	//使用用户名；测试
        //		  data.username.cstring =char_string_username;//测试
        memset(char_string_password, 0, sizeof(char_string_password));
        memcpy(char_string_password, md5_password_ascii, 32);	//使用密码
        data.password.cstring = char_string_password; //密码
        data.willFlag = 1;
        memset(char_string_topicName, 0, sizeof(char_string_topicName));
        //			memcpy(char_string_topicName,"/testCode1-DTU-20170925-000068/lastwill",39);	//使用topicName;用户名；测试
        memcpy(char_string_topicName, "/", 1);	//使用topicName
        memcpy(char_string_topicName + 1, factory_gateway_set + 156, factory_gateway_set[155]);	//使用topicName；用户名
        memcpy(char_string_topicName + 1 + factory_gateway_set[155], "/lastwill", 9);	//使用topicName="/${deviceId}/lastwill"
        data.will.topicName.cstring = char_string_topicName;
        //		memset(char_string,0,sizeof(char_string));
        //		memcpy(char_string,"\"devid\":\"111123132dsa\"",22);	//使用will.message
        //		data.will.message.cstring=char_string;
    }
    mqtt_len = MQTTSerialize_connect(ReportData2, sizeof(ReportData2), &data);
}

void mqtt_subscribe(void)
{
    // subscribe                配置接收状态  --订阅主题
    char char_string[40];
    MQTTString topicString = MQTTString_initializer;
    volatile int len;
    int msgid = 1;
    int req_qos = 0;
    memset(char_string, 0, sizeof(char_string));
    memcpy(char_string, factory_gateway_set + 84, factory_gateway_set[83]); //factory_gateway_set[83]为订阅主题长度，[84]~[113]最多	30个字符
    if (factory_gateway_set[1] == 4)
    {
        memset(char_string, 0, sizeof(char_string));
        //		memcpy(char_string,"/testCode1-DTU-20170925-000068/control",38);	//订阅主题；测试
        memcpy(char_string, "/", 1);	//使用topicName
        memcpy(char_string + 1, factory_gateway_set + 156, factory_gateway_set[155]);	//使用topicName；用户名
        memcpy(char_string + 1 + factory_gateway_set[155], "/control", 8);	//使用topicName="/${deviceId}/lastwill"
    }
    topicString.cstring = char_string;
    mqtt_len = MQTTSerialize_subscribe(ReportData2, sizeof(ReportData2), 0, msgid, 1, &topicString, &req_qos);
}

void mqtt_publish(unsigned char *real, int pub_len)   //推送主题
{
    //volatile int len;
    char char_string[40];
    MQTTString topicString = MQTTString_initializer;
    memset(char_string, 0, sizeof(char_string));
    memcpy(char_string, factory_gateway_set + 125, factory_gateway_set[124]); //factory_gateway_set[124]为推送主题长度，[125]~[154]最多	30个字符

    if (factory_gateway_set[1] == 4)
    {
        memset(char_string, 0, sizeof(char_string));
        //			memcpy(char_string,"/testCode1-DTU-20170925-000068/status",37);	//使用topicName;用户名；测试
        memcpy(char_string, "/", 1);	//使用topicName
        memcpy(char_string + 1, factory_gateway_set + 156, factory_gateway_set[155]);	//使用topicName；用户名
        memcpy(char_string + 1 + factory_gateway_set[155], "/status", 7);	//使用topicName="/${deviceId}/lastwill"
    }
    topicString.cstring = char_string;
    mqtt_len = MQTTSerialize_publish(ReportData2, sizeof(ReportData2), 0, 0, 0, 0, topicString, (unsigned char*)real, pub_len);
}

/*以下推送主题函数是为SDK协议推送控制器状态使用*/
void mqtt_publish1(unsigned char *real, int pub_len)   //推送主题1
{
    MQTTString topicString = MQTTString_initializer;
    topicString.cstring = SDK_topicString_pub1;
    mqtt_len = MQTTSerialize_publish(ReportData2, sizeof(ReportData2), 0, 0, 0, 0, topicString, (unsigned char*)real, pub_len);
}

//SDK函数
/*该函数将要发送平台的2字节起始地址(keyX_addr)、2字节数据数组(value)及参数个数n转换为：key1:value1，key2:value2，...
   keyn:valuen 的ASC码,并封装成{key1:value1，key2:value2，... keyn:valuen}送到key_data_send数组中*/
static u8 made_keyX_value(u16 keyX_addr, u8 *value, u8 n, u8 *key_data_send)
{
    u8 i, keyX_asc[5], value_asc[4];//u8 8位无符号数
    s8 j;            //s8 8位有符号数
    u16 temp_addr, temp_value;

    for (i = 0; i < n; i++)
    {
        temp_addr = keyX_addr + i * 4;
        temp_value = (value[2 * i + 1] << 8) | value[2 * i];
        for (j = 4; j >= 1; j--)
        {
            keyX_asc[j] = temp_addr % 16;//把2字节16位的地址转换成ASC码；即key值
            if (keyX_asc[j] <= 9)
            {
                keyX_asc[j] = keyX_asc[j] + '0';
            }
            else
            {
                keyX_asc[j] = keyX_asc[j] + 0x37;
            }
            keyX_asc[0] = 'H';

            value_asc[j - 1] = temp_value % 16;//把2字节16位的数据转换成ASC码；即上报平台的数据
            if (value_asc[j - 1] <= 9)
            {
                value_asc[j - 1] = value_asc[j - 1] + '0';
            }
            else
            {
                value_asc[j - 1] = value_asc[j - 1] + 0x37;
            }

            temp_addr = temp_addr / 16;
            temp_value = temp_value / 16;
        }
        memcpy(key_data_send + i * (6 + 5) + 1, keyX_asc, 5);//H+4字节+‘:’;共6字节
        key_data_send[i*(6 + 5) + 1 + 5] = ':';
        memcpy(key_data_send + i * (6 + 5) + 7, value_asc, 4);//4字节+‘，’;共5字节
        key_data_send[i*(6 + 5) + 7 + 4] = ',';
    }
    key_data_send[0] = '{';
    key_data_send[n*(6 + 5)] = '}';
    return	 n * (6 + 5) + 1;//最后一个字节 ',' 被  '}'  替换了，所以只加1，每个参数需要11个字节
}

/*该函数将要发送平台的2字节起始地址(keyX_addr)、2字节数据数组(value)及参数个数n转换为：{"key1":value1}，{"key2":value2}，...
	{"keyn":valuen} 的ASC码,并封装成[{"key1":value1}，{"key2":value2}，... {"keyn":valuen}]送到key_data_send数组中*/
static u8 made_keyX_value4(u16 keyX_addr, u8 *value, u8 n, u8 *key_data_send)
{
    u8 i, keyX_asc[4], value_asc[4];//u8 8位无符号数
    s8 j;            //s8 8位有符号数
    u16 temp_addr, temp_value;

    for (i = 0; i < n; i++)
    {
        temp_addr = keyX_addr + i * 4;
        temp_value = (value[2 * i + 1] << 8) | value[2 * i];
        for (j = 3; j >= 0; j--)
        {
            keyX_asc[j] = temp_addr % 16;//把2字节16位的地址转换成ASC码；即key值
            if (keyX_asc[j] <= 9)
            {
                keyX_asc[j] = keyX_asc[j] + '0';
            }
            else
            {
                keyX_asc[j] = keyX_asc[j] + 0x37;
            }

            value_asc[j] = temp_value % 16;//把2字节16位的数据转换成ASC码；即上报平台的数据
            if (value_asc[j] <= 9)
            {
                value_asc[j] = value_asc[j] + '0';
            }
            else
            {
                value_asc[j] = value_asc[j] + 0x37;
            }

            temp_addr = temp_addr / 16;
            temp_value = temp_value / 16;
        }
        key_data_send[i * 14 + 1 + 0] = '{';
        key_data_send[i * 14 + 1 + 1] = '"';
        memcpy(key_data_send + i * 14 + 1 + 2, keyX_asc, 4);//{"4字节":4字节},  共14字节(包括,)
        key_data_send[i * 14 + 1 + 6] = '"';
        key_data_send[i * 14 + 1 + 7] = ':';
        memcpy(key_data_send + i * 14 + 1 + 8, value_asc, 4);//{"4字节":4字节},  共14字节(包括,)
        key_data_send[i * 14 + 1 + 8 + 4] = '}';
        key_data_send[i * 14 + 1 + 8 + 5] = ',';
    }
    key_data_send[0] = '[';
    key_data_send[n * 14] = ']';
    return	 n * 14 + 1;//最后一个字节 ',' 被  '}'  替换了，所以只加1，每个参数需要14个字节
}

/*该函数将要发送平台的2字节起始地址(keyX_addr)、2字节的数据int型数组(valueN)转换成4字节数据数组(valueF)及参数个数n转换为：key1:valueF1，key2:valueF2，...
   keyn:valueFn 的ASC码,并封装成{key1:valueF1，key2:valueF2，... keyn:valueFn}送到key_data_send数组中;valueFN的值为XXXX.XX*/
static u8 made_keyX_valueF(u16 keyX_addr, u8 *value, float *valueF, u8 n, u8 *key_data_send)
{
    u8 i, keyX_asc[5], value_asc[10];//u8 8位无符号数
    u8 valueF_asc_len = 0, valueF_asc_len1 = 0;
    s8 j;                                 //s8 8位有符号数
    float unit_valueF[8];
    u16 temp_addr, temp_value;

    for (i = n; i < n + 4; i++)
    {
        temp_addr = keyX_addr + i * 4;

        temp_value = (value[2 * i + 1] << 8) | value[2 * i];
        if (valueF[2 * i] <= 0.1&&valueF[2 * i] >= -0.1)
        {
            valueF[2 * i] = 655.35;
            valueF[2 * i + 1] = 0;
        }
        if (valueF[2 * i + 1] > 99999.99 || valueF[2 * i + 1] < -99999.99)
        {
            valueF[2 * i] = 655.35;
            valueF[2 * i + 1] = 0;
        }
        unit_valueF[i] = temp_value / valueF[2 * i] + valueF[2 * i + 1];//float_to_string(float data, u8 *str)
        valueF_asc_len = float_to_string(unit_valueF[i], value_asc);

        for (j = 4; j >= 1; j--)
        {
            keyX_asc[j] = temp_addr % 16;//把2字节16位的地址转换成ASC码；即key值
            if (keyX_asc[j] <= 9)
            {
                keyX_asc[j] = keyX_asc[j] + '0';
            }
            else
            {
                keyX_asc[j] = keyX_asc[j] + 0x37;
            }
            temp_addr = temp_addr / 16;
        }
        keyX_asc[0] = 'H';
        memcpy(key_data_send + (i - n) * 6 + valueF_asc_len1 + 1, keyX_asc, 5);//H+4字节+‘:’;共6字节
        key_data_send[(i - n) * 6 + valueF_asc_len1 + 1 + 5] = ':';
        memcpy(key_data_send + (i - n) * 6 + valueF_asc_len1 + 6 + 1, value_asc, valueF_asc_len);//5字节+‘，’;共6字节

        key_data_send[(i - n) * 6 + 7 + valueF_asc_len1 + valueF_asc_len] = ',';
        valueF_asc_len1 = valueF_asc_len1 + valueF_asc_len + 1;
    }
    key_data_send[0] = '{';
    key_data_send[4 * 6 + valueF_asc_len1] = '}';
    return	 4 * 6 + valueF_asc_len1 + 1;//最后一个字节 ',' 被  '}'  替换了，所以只加1，每个参数需要10个字节
}

/*该函数将要发送平台的2字节起始地址(keyX_addr)、2字节的数据int型数组(valueN)转换成4字节数据数组(valueF)及参数个数n转换为：{"key1":valueF1}，{"key2":valueF2}，...
   {"keyn":valueFn" 的ASC码,并封装成{{"key1":valueF1}，{"key2":valueF2}，...{"keyn":valueFn}}送到key_data_send数组中;valueFN的值为XXXX.XX*/
static u8 made_keyX_valueF4(u16 keyX_addr, u8 *value, float *valueF, u8 n, u8 *key_data_send)
{
    u8 i, keyX_asc[4], value_asc[10];//u8 8位无符号数
    u8 valueF_asc_len = 0, valueF_asc_len1 = 0;
    s8 j;                                 //s8 8位有符号数
    float unit_valueF[8];
    u16 temp_addr, temp_value;

    for (i = n; i < n + 4; i++)
    {
        temp_addr = keyX_addr + i * 4;

        temp_value = (value[2 * i + 1] << 8) | value[2 * i];
        if (valueF[2 * i] <= 0.1&&valueF[2 * i] >= -0.1)
        {
            valueF[2 * i] = 655.35;
            valueF[2 * i + 1] = 0;
        }
        if (valueF[2 * i + 1] > 99999.99 || valueF[2 * i + 1] < -99999.99)
        {
            valueF[2 * i] = 655.35;
            valueF[2 * i + 1] = 0;
        }
        unit_valueF[i] = temp_value / valueF[2 * i] + valueF[2 * i + 1];//float_to_string(float data, u8 *str)
        valueF_asc_len = float_to_string(unit_valueF[i], value_asc);

        for (j = 3; j >= 0; j--)
        {
            keyX_asc[j] = temp_addr % 16;//把2字节16位的地址转换成ASC码；即key值
            if (keyX_asc[j] <= 9)
            {
                keyX_asc[j] = keyX_asc[j] + '0';
            }
            else
            {
                keyX_asc[j] = keyX_asc[j] + 0x37;
            }
            temp_addr = temp_addr / 16;
        }
        memcpy(key_data_send + (i - n) * 8 + valueF_asc_len1 + 1 + 2, keyX_asc, 4);//{"4字节":共8字节;说明： {"2100":
        key_data_send[(i - n) * 8 + valueF_asc_len1 + 1] = '{';
        key_data_send[(i - n) * 8 + valueF_asc_len1 + 1 + 1] = '"';
        key_data_send[(i - n) * 8 + valueF_asc_len1 + 1 + 6] = '"';
        key_data_send[(i - n) * 8 + valueF_asc_len1 + 1 + 7] = ':';
        memcpy(key_data_send + (i - n) * 8 + valueF_asc_len1 + 1 + 8, value_asc, valueF_asc_len);//4字节+‘，’;共5字节
        key_data_send[(i - n) * 8 + 9 + valueF_asc_len1 + valueF_asc_len] = '}';
        key_data_send[(i - n) * 8 + 9 + valueF_asc_len1 + valueF_asc_len + 1] = ',';
        valueF_asc_len1 = valueF_asc_len1 + valueF_asc_len + 2;
    }
    key_data_send[0] = '[';
    key_data_send[4 * 8 + valueF_asc_len1] = ']';
    return	 4 * 8 + valueF_asc_len1 + 1;//最后一个字节 ',' 被  '}'  替换了，所以只加1，每个参数需要10个字节
}

//其它函数
static void bytelen_to_asc(u8 *zfc, u8 len)   //把1个字节十六进制数转换为十进制数值的asc码
{
    s8 i;
    for (i = 2; i >= 0; i--)
    {
        zfc[i] = len % 10 + '0';
        len = len / 10;
    }
}

static void byte_to_asc(u8 byte_hex, u8 *byte_asc)   //把1个字节十六进制数转换为对应asc码
{
    u8 temp_hex = 0;
    temp_hex = (byte_hex & 0xF0) >> 4;
    if (temp_hex <= 9)
    {
        *byte_asc = temp_hex + '0';
    }
    else
    {
        *byte_asc = temp_hex + 0x57;
    }
    temp_hex = (byte_hex & 0x0F);
    if (temp_hex <= 9)
    {
        *(byte_asc + 1) = temp_hex + '0';
    }
    else
    {
        *(byte_asc + 1) = temp_hex + 0x57;
    }
}

static u16 dword_asc_hex(u8 *dword_asc)   //把4个字节的ASC码("210C")转换成一个16位十六进制数0x210C
{
    u8 i;
    u16 len = 0;
    for (i = 0; i <= 3; i++)
    {
        if (dword_asc[i] <= 0x39)
        {
            len = len * 16 + (dword_asc[i] - 0x30);
        }
        else if (dword_asc[i] >= 0x41 && dword_asc[i] <= 0x46)
        {
            len = len * 16 + (dword_asc[i] - 0x37);
        }
        else
        {
            len = len * 16 + (dword_asc[i] - 0x57);
        }
    }
    return len;
}

static void cycle_cmd(void)   //测试各运行程序事件是否正常
{
    //开始测试网络初始化事件是否运行正常
    if (halt_module < 0x10 && cmd_flg != 0xFF)   //检测4G通信模块工作是否正常，不正常则重新REST4G通信模块
    {
        halt_module++;//GSM模块初始化完成才进行检测;		if(cmd_flg==0xFF)取消
        Start_timerEx(CYCLE_CMD_EVT, 10000);//检测各事件程序运行是否工作正常
    }
    else if (halt_module >= 0x10 && cmd_flg != 0xFF)
    {
        while (1);
        //    halt_module=0x00;
        //	  cmd_flg   = 0x01;//CDMA重新REST
        //    send_flg  = 0x01;
        //	  Start_timerEx(NET_INIT_EVT,500);
        //		Start_timerEx(CYCLE_CMD_EVT,65000);
    }
    else
    {
        Start_timerEx(CYCLE_CMD_EVT, 10000);//检测各事件程序运行是否工作正常
        return;
    }
    //结束测试网络初始化事件是否运行正常
}
static void set_slave_param(void)   //网关有线设定子站参数
{
    u8 i;
    if (slave_set_flg == 1)   //控制器参数设定
    {
        while ((ctrlslave_param_flg[set_slave_Index] <= 0) && (set_slave_Index < 4))   //CSH_Wired_finish==0|
        {
            set_slave_Index++;
        }
        if (set_slave_Index < 4)
        {
            YX_Slave_ID = set_slave_Index + first_xiabiao_I + 33;
            bytelen3 = WriteMultipleRegister(YX_Slave_ID, 4, 12, ctrlslave_param_set[set_slave_Index], ReportData3);
            memcpy(USART3SendTCB, ReportData3, bytelen3);
            WriteDataToDMA_BufferTX3(bytelen3);
            if (ctrlslave_param_flg[set_slave_Index] > 0) ctrlslave_param_flg[set_slave_Index]--;//子站发送记录，每发送一次减1，收到回复清0，否则重复发送3次
            set_slave_Index++;
        }
        if (set_slave_Index >= 4)
        {
            set_slave_Index = 0;
        }
        slave_set_finish = 0;
        for (i = 0; i < 4; i++)
        {
            slave_set_finish = slave_set_finish | ctrlslave_param_flg[i];
        }
        if (slave_set_finish != 0)
        {
            Start_timerEx(SET_SLAVEPARAM_EVT, 280);
        }
        else
        {
            set_slave_Index = 0;
            slave_set_flg = 0;
            set_finish_flg = 0;
            Stop_timerEx(SET_SLAVEPARAM_EVT);
            Start_timerEx(WG_SENDZZ_EVT, 200);
        }
    }
    if (slave_set_flg == 2)   //采集器参数设定
    {
        while ((cjqslave_param_flg[set_slave_Index] <= 0) && (set_slave_Index < 8))   //CSH_Wired_finish==0|
        {
            set_slave_Index++;
        }
        if (set_slave_Index < 8)
        {
            YX_Slave_ID = set_slave_Index + first_xiabiao_I + 1;
            bytelen3 = WriteMultipleRegister(YX_Slave_ID, 16, 18, cjqslave_param_set[set_slave_Index], ReportData3);
            memcpy(USART3SendTCB, ReportData3, bytelen3);
            WriteDataToDMA_BufferTX3(bytelen3);
            if (cjqslave_param_flg[set_slave_Index] > 0) cjqslave_param_flg[set_slave_Index]--;//子站发送记录，每发送一次减1，收到回复清0，否则重复发送3次
            set_slave_Index++;
        }
        if (set_slave_Index >= 8)
        {
            set_slave_Index = 0;
        }
        slave_set_finish = 0;
        for (i = 0; i < 8; i++)
        {
            slave_set_finish = slave_set_finish || cjqslave_param_flg[i];
        }
        if (slave_set_finish != 0)
        {
            Start_timerEx(SET_SLAVEPARAM_EVT, 280);
        }
        else
        {
            set_slave_Index = 0;
            slave_set_flg = 0;
            set_finish_flg = 0;
            Stop_timerEx(SET_SLAVEPARAM_EVT);
            Start_timerEx(WG_SENDZZ_EVT, 200);
        }
    }
    if (slave_set_flg == 3)   //设定子站地址及信道
    {
        bytelen3 = WriteMultipleRegister(0xF0, 34, 8, set_slaveID_channel, ReportData3);
        memcpy(USART3SendTCB, ReportData3, bytelen3);
        WriteDataToDMA_BufferTX3(bytelen3);
        if (slaveID_channel_flg > 0)
        {
            slaveID_channel_flg--;   //子站发送记录，每发送一次减1，收到回复清0，否则重复发送5次
        }
        if (slaveID_channel_flg != 0)
        {
            Start_timerEx(SET_SLAVEPARAM_EVT, 280);
        }
        else
        {
            slave_set_flg = 0;
            set_finish_flg = 0;
            if (set_slaveID_channel[0] == 65 && set_slaveID_channel[2] == 255)
            {
                SI4463_Channel = set_slaveID_channel[3];
                Flash_Write(0x0807D000, set_slaveID_channel + 3, 1);
                if (close_433MHZ != 0)
                {
                    SI4463_Init();
                    Stop_timerEx(SET_SLAVEPARAM_EVT);
                    Start_timerEx(WX_SENDZZ_EVT, 3000);
                }//触摸屏出厂设置应加入变量
            }
            Stop_timerEx(SET_SLAVEPARAM_EVT);
            Start_timerEx(WG_SENDZZ_EVT, 200);
        }
    }
}

//433MHZ无线函数

// static void	Get_WX_Channel(void)
//{
//	if(!GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2))
//		SI4463_Channel += 1;
//	if(!GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3))
//		SI4463_Channel += 2;
//	if(!GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4))
//		SI4463_Channel += 4;
//	if(!GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_5))
//		SI4463_Channel += 8;
//	if(!GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6))
//		SI4463_Channel += 16;
//}

static void Clear_Buf(u8 buf[], u8 length, u8 data)
{
    u8 i = 0;
    for (i = 0; i < length; i++)
        buf[i] = data;
}

static void SI4463Receive(u8 len, u8 *pData)   //与子站通信，接收子站回复
{
    if (GetCRC16(pData, len) == 0)
    {
        online_slaveID_WX[pData[0] - 1] = pData[0];//表示该子站在线，记录下来
        ZZ_Wireles_flag[pData[0] - 1] = 0x01;//收到子站无线通信，设置标志，0x01表示存在，0表示该子站不存在
        init_cmd_numWX[pData[0] - 1] = 0;
        //		 Start_timerEx(WX_SENDZZ_EVT,50);
        if (pData[1] == 0x03)
        {
            memcpy(Collectors[pData[0] - 1], pData + 3, pData[2]);
            Start_timerEx(WX_SENDZZ_EVT, 250);
            return;
        }
        if (pData[1] == 0x06)
        {
            if ((pData[2] == 0x00) && (pData[3] <= 0x03) && (pData[0] >= 33) && (pData[0] <= 64))
                //slave_set_flg在Send_slave_cmd和RxReport5函数中设定，slave_set_flg==0正常接收子站回复
            {
                crtl_cmd_numWX[pData[0] - 33][pData[3]] = 0;//表示子站收到控制指令，无需再发送写命令
                Start_timerEx(WX_SENDZZ_EVT, 250);
                return;
            }
        }

        if (pData[1] == 0x10)
        {
            if ((pData[0] >= 33) && (slave_set_flgWX == 1) && (pData[3] >= 4) && (pData[3] <= 15))   //slave_set_flgWX==1接收设定参数控制子站无线回复
            {
                ctrlslave_param_flgWX[pData[0] - firstwx_xiabiao_i - 32] = 0;
                Start_timerEx(WX_SET_SLAVEPARAM_EVT, 500);
                return;
            }
            if ((pData[3] >= 16) && (pData[3] <= 33) && (slave_set_flgWX == 2))   //slave_set_flgWX==2接收设定参数采集器和控制器子站无线回复
            {
                cjqslave_param_flgWX[pData[0] - firstwx_xiabiao_i - 1] = 0;
                Start_timerEx(WX_SET_SLAVEPARAM_EVT, 500);
                return;
            }
            if ((pData[3] >= 34) && (pData[3] <= 35) && (slave_set_flgWX == 3))   //slave_set_flgWX==2接收设定参数采集器和控制器子站无线回复
            {
                slaveID_channel_flgWX = 0;
                Start_timerEx(WX_SET_SLAVEPARAM_EVT, 500);
                return;
            }
        }
    }
    else if (slave_set_flgWX != 0)
    {
        Start_timerEx(WX_SET_SLAVEPARAM_EVT, 500);
    }
    else
    {
        Start_timerEx(WX_SENDZZ_EVT, 500);
    }
}

static void SI4463_SENDZZ(void)
{
    switch (WX_Query_Flag)   //WX_Query_Flag
    {
    case CONTROLLERS_CMD:
        //初始化未完成 或 发送子站命令成功收到回复无需发送，同时控制器子站ID小于等于32个。下面的发送子站命令跳过;
        while ((crtl_cmd_numWX[Query_Index_ControllerWX][ctrl_wx_j] <= 0) && (Query_Index_ControllerWX < 32))
        {
            ctrl_wx_j++;
            if (ctrl_wx_j >= 4)
            {
                Query_Index_ControllerWX++;
                ctrl_wx_j = 0;
            }
        }
        if (Query_Index_ControllerWX < 32 && ctrl_wx_j <= 3)
        {
            WX_len = WriteSingleRegister(Query_Index_ControllerWX + 33, ctrl_wx_j, Controllers[Query_Index_ControllerWX] + ctrl_wx_j * 2, SI4463_TxBUFF);
            SI4463_SEND_PACKET(SI4463_TxBUFF, WX_len, SI4463_Channel, 0);
            if (crtl_cmd_numWX[Query_Index_ControllerWX][ctrl_wx_j] > 0)crtl_cmd_numWX[Query_Index_ControllerWX][ctrl_wx_j]--;//子站发送记录，每发送一次减1，收到回复清0，否则重复发送3次
            ctrl_wx_j++;
            if (ctrl_wx_j >= 4)
            {
                Query_Index_ControllerWX++;
                ctrl_wx_j = 0;
            }
        }
        if (Query_Index_ControllerWX >= 32)
        {
            Query_Index_ControllerWX = 0;
            WX_Query_Flag = COLLECTORS_CMD;
        }
        Start_timerEx(WX_SENDZZ_EVT,350);
        break;

    case COLLECTORS_CMD:
        while ((online_slaveID_WX[Query_Index_CollectorWX] == 0) && (Query_Index_CollectorWX < 64))
        {
            Query_Index_CollectorWX++;
        }
        if (Query_Index_CollectorWX < 64)
        {
            WX_len = ReadData(Query_Index_CollectorWX + 1, READ_HOLDING_REGISTER, 0x0000, 0x0008, SI4463_TxBUFF);
            SI4463_SEND_PACKET(SI4463_TxBUFF, WX_len, SI4463_Channel, 0);
            Query_Index_CollectorWX++;
        }
        if (Query_Index_CollectorWX > 63)   //采集器查询完毕，开始查询控制器
        {
            Query_Index_CollectorWX = 0;
            WX_Query_Flag = ZZ_QUERY_COLLECTOR;
            Start_timerEx(WX_SENDZZ_EVT, 350);
            break;
        }
        else
        {
            WX_Query_Flag = CONTROLLERS_CMD;
            Start_timerEx(WX_SENDZZ_EVT, 350);
            break;
        }
    case ZZ_QUERY_COLLECTOR://采集器查询
        while ((online_slaveID_WX[Query_IndexZZ_C_WX] != 0) && (Query_IndexZZ_C_WX < 32))
        {
            Query_IndexZZ_C_WX++;
        }
        if (Query_IndexZZ_C_WX < 32)
        {
            WX_len = ReadData(Query_IndexZZ_C_WX + 1, READ_HOLDING_REGISTER, 0x0000, 0x0008, SI4463_TxBUFF);
            SI4463_SEND_PACKET(SI4463_TxBUFF, WX_len, SI4463_Channel, 0);
            if (init_cmd_numWX[Query_IndexZZ_C_WX] > 0)
            {
                init_cmd_numWX[Query_IndexZZ_C_WX]--;
            }
            else
            {
                Query_IndexZZ_C_WX++;
            }
        }
        if (Query_IndexZZ_C_WX > 31)
        {
            Query_IndexZZ_C_WX = 0;   //采集器查询完毕，开始查询控制器
        }
        WX_Query_Flag = ZZ_QUERY_CONTROLLER;
        Start_timerEx(WX_SENDZZ_EVT, 350);
        break;

    case ZZ_QUERY_CONTROLLER://控制器查询
        while ((online_slaveID_WX[Query_IndexZZ_K_WX + 32] != 0) && (Query_IndexZZ_K_WX < 32))
        {
            Query_IndexZZ_K_WX++;
        }
        if (Query_IndexZZ_K_WX < 32)
        {
            WX_len = ReadData(Query_IndexZZ_K_WX + 33, READ_HOLDING_REGISTER, 0x0000, 0x0008, SI4463_TxBUFF);
            SI4463_SEND_PACKET(SI4463_TxBUFF, WX_len, SI4463_Channel, 0);
            if (init_cmd_numWX[Query_IndexZZ_K_WX + 32] > 0)
            {
                init_cmd_numWX[Query_IndexZZ_K_WX + 32]--;
            }
            else
            {
                Query_IndexZZ_K_WX++;
            }
        }
        if (Query_IndexZZ_K_WX > 31)   //采集器查询完毕，开始查询控制器
        {
            Query_IndexZZ_K_WX = 0;

            if (ZZ_Wireles_flag[64] == 0)
            {
                u8 i;
                CSH_countWX = 0;
                for (i = 0; i < 64; i++)
                {
                    CSH_countWX = CSH_countWX | init_cmd_numWX[i];
                }
                if ((CSH_countYX == 0) && (CSH_countWX == 0))
                {
                    ZZ_Wireles_flag[64] = 1;
                }
            }

            if (Query_Wired_WirelesWX > 0)
            {
                ZZ_Wireles_flag[Query_Wired_WirelesWX - 1] = ZZ_temp_stateWX;
            }
            else
            {
                ZZ_Wireles_flag[63] = ZZ_temp_stateWX;
            }
            ZZ_temp_stateWX = ZZ_Wireles_flag[Query_Wired_WirelesWX];
            ZZ_Wireles_flag[Query_Wired_WirelesWX] = 0x02;//子站无线通信标志=2,触摸屏显示黄灯,正在检测
//							online_slaveID_WX[Query_Wired_WirelesWX]=0x00;//正在检测
            if (ZZ_temp_stateWX == 0x02)
            {
                ZZ_temp_stateWX = 0;
            }
            Query_Wired_WirelesWX++;
            if (Query_Wired_WirelesWX > 63)
            {
                Query_Wired_WirelesWX = 0;
            }
        }
        WX_Query_Flag = CONTROLLERS_CMD;
        Start_timerEx(WX_SENDZZ_EVT, 350);
        break;
    default:
        Start_timerEx(WX_SENDZZ_EVT, 500);
        break;
    }
}

static void set_slave_paramWX(void)   //网关无线设定子站参数
{
    u8 i;
    if (slave_set_flgWX == 1)
    {
        while ((ctrlslave_param_flgWX[set_slave_IndexWX] <= 0) && (set_slave_IndexWX < 4))   //CSH_Wireles_finish==0|
        {
            set_slave_IndexWX++;
        }
        if (set_slave_IndexWX < 4)
        {
            WX_Slave_ID = set_slave_IndexWX + firstwx_xiabiao_i + 33;
            WX_len = WriteMultipleRegister(WX_Slave_ID, 4, 12, ctrlslave_param_set[set_slave_IndexWX], SI4463_TxBUFF);
            SI4463_SEND_PACKET(SI4463_TxBUFF, WX_len, SI4463_Channel, 0);
            if (ctrlslave_param_flgWX[set_slave_IndexWX] > 0)
            {
                ctrlslave_param_flgWX[set_slave_IndexWX]--;   //子站发送记录，每发送一次减1，收到回复清0，否则重复发送3次
            }
            set_slave_IndexWX++;
        }
        if (set_slave_IndexWX >= 4)
        {
            set_slave_IndexWX = 0;
        }
        slave_set_finishWX = 0;
        for (i = 0; i < 4; i++)
        {
            slave_set_finishWX = slave_set_finishWX | ctrlslave_param_flgWX[i];
        }
        if (slave_set_finishWX != 0)
        {
            Start_timerEx(WX_SET_SLAVEPARAM_EVT, 280);
        }
        else
        {
            set_slave_IndexWX = 0;
            slave_set_flgWX = 0;
            set_finish_flg = 0;
            Stop_timerEx(WX_SET_SLAVEPARAM_EVT);
            Start_timerEx(WX_SENDZZ_EVT, 350);
        }
    }
    if (slave_set_flgWX == 2)
    {
        while ((cjqslave_param_flgWX[set_slave_IndexWX] <= 0) && (set_slave_IndexWX < 8))   //CSH_Wireles_finish==0|
        {
            set_slave_IndexWX++;
        }
        if (set_slave_IndexWX < 8)
        {
            WX_Slave_ID = set_slave_IndexWX + firstwx_xiabiao_i + 1;
            WX_len = WriteMultipleRegister(WX_Slave_ID, 16, 18, cjqslave_param_set[set_slave_IndexWX], SI4463_TxBUFF);
            SI4463_SEND_PACKET(SI4463_TxBUFF, WX_len, SI4463_Channel, 0);
            if (cjqslave_param_flgWX[set_slave_IndexWX] > 0)
            {
                cjqslave_param_flgWX[set_slave_IndexWX]--;   //子站发送记录，每发送一次减1，收到回复清0，否则重复发送3次
            }
            set_slave_IndexWX++;
        }
        if (set_slave_IndexWX >= 8)
        {
            set_slave_IndexWX = 0;
        }
        slave_set_finishWX = 0;
        for (i = 0; i < 8; i++)
        {
            slave_set_finishWX = slave_set_finishWX | cjqslave_param_flgWX[i];
        }
        if (slave_set_finishWX != 0)
        {
            Start_timerEx(WX_SET_SLAVEPARAM_EVT, 500);
        }
        else
        {
            set_slave_IndexWX = 0;
            slave_set_flgWX = 0;
            set_finish_flg = 0;
            Stop_timerEx(WX_SET_SLAVEPARAM_EVT);
            Start_timerEx(WX_SENDZZ_EVT, 350);
        }
    }
    if (slave_set_flgWX == 3)
    {
        WX_len = WriteMultipleRegister(0xF0, 34, 8, set_slaveID_channel, SI4463_TxBUFF);//子站ID,子站相应的保存地址，变量数，发送内容
        SI4463_SEND_PACKET(SI4463_TxBUFF, WX_len, SI4463_Channel, 0);
        if (slaveID_channel_flgWX > 0)
        {
            slaveID_channel_flgWX--;   //子站发送记录，每发送一次减1，收到回复清0，否则重复发送5次
        }
        if (slaveID_channel_flgWX != 0)
        {
            Start_timerEx(WX_SET_SLAVEPARAM_EVT, 500);
        }
        else
        {
            slave_set_flgWX = 0;
            set_finish_flg = 0;
            if (set_slaveID_channel[0] == 65 && set_slaveID_channel[2] == 255)
            {
                SI4463_Channel = set_slaveID_channel[3];
                Flash_Write(0x0807D000, set_slaveID_channel + 3, 1);
                if (close_433MHZ != 0)
                {
                    SI4463_Init();
                    Stop_timerEx(WX_SET_SLAVEPARAM_EVT);
                    Start_timerEx(WX_SENDZZ_EVT, 3000);
                    return;
                }//触摸屏出厂设置应加入变量
            }
            Stop_timerEx(WX_SET_SLAVEPARAM_EVT);
            Start_timerEx(WX_SENDZZ_EVT, 350);
        }
    }
}
//以下为六参数采集程序
static void wgcollector_data(void)   //传感器采集数据，水肥采用0,1,4三路通道，频率输入转换成流量
{
    //	u16 temp;
    switch (ReadDataCNT)
    {
    case 0:
        TD_param_num = 0;//建立每个通道上报参数的顺序，如果该通道不检测参数，后面通道检测的参数自动往前移
        if (factory_gateway_set[12] == 1 && factory_gateway_set[15] != 18)   //光照度;factory_gateway_set[15]=18 为超声波液位脉冲宽度检测，不允许使用SCK
        {
            collector_temp = Get_Illuminance();//光照度；PC0
            if (collector_temp != 0xFFFF)
            {
                wgcollector_data_buff[TD_param_num++] = collector_temp & 0x00ff;           		//光照度低位
                wgcollector_data_buff[TD_param_num++] = (collector_temp & 0xff00) >> 8;      		//光照度高位
            }
            else
            {
                TD_param_num++;           	                      	//光照度低位
                TD_param_num++;      		                           //光照度高位
            }
            break;
        }

        else if (factory_gateway_set[12] == 5 && factory_gateway_set[15] != 18)   //大气压力;factory_gateway_set[15]=18 为超声波液位脉冲宽度检测，不允许使用SCK
        {
            collector_temp = GET_PRESSUE0();
            if (collector_temp != 0xffff)
            {
                wgcollector_data_buff[TD_param_num++] = collector_temp & 0x00ff;           		//大气压力低位
                wgcollector_data_buff[TD_param_num++] = (collector_temp & 0xff00) >> 8;      		//大气压力高位
            }
            else
            {
                TD_param_num++;
                TD_param_num++;
            }
            break;
        }

        else if (factory_gateway_set[12] == 6)   //开关量输入
        {
            collector_temp = PCin(0);//光照度输入脚PC0
            wgcollector_data_buff[TD_param_num++] = collector_temp & 0x00ff;           		//输入开关量=1，则collector_data_buff[0]=0x01
            wgcollector_data_buff[TD_param_num++] = (collector_temp & 0xff00) >> 8;      		//输入开关量=1，则collector_data_buff[1]=0x00
            break;
        }
        else if (factory_gateway_set[12] == 7)   //频率输入;
        {
            u8 i;
            collector_temp = 0;
            for (i = freq_I + 1; i <= 60; i++)
            {
                collector_temp = collector_temp + TIM2_FrequencyPC0[i];
            }
            for (i = 0; i < freq_I; i++)
            {
                collector_temp = collector_temp + TIM2_FrequencyPC0[i];
            }
            wgcollector_data_buff[TD_param_num++] = collector_temp & 0x00ff;           		//输入频率字节低
            wgcollector_data_buff[TD_param_num++] = (collector_temp & 0xff00) >> 8;      		//输入频率字节高；每分钟脉冲数，除以60为频率，水肥除以12
            break;
        }
        else  if (factory_gateway_set[12] == 8)   //0~1VDC输入；
        {
            TD_param_num++;
            TD_param_num++;
            break;
        }
        else if (factory_gateway_set[12] == 10 && factory_gateway_set[15] != 18)   ////液位压力cps20;factory_gateway_set[15]=18 为超声波液位脉冲宽度检测，不允许使用SCK
        {
            collector_temp = GET_level0();
            if (collector_temp != 0xffff)
            {
                wgcollector_data_buff[TD_param_num++] = collector_temp & 0x00ff;           		//液位压力低位,触摸屏k=193.16 b=30 单位：kpa
                wgcollector_data_buff[TD_param_num++] = (collector_temp & 0xff00) >> 8;      		//液位压力高位
                absolute_pressure_lower = collector_temp;
            }
            else
            {
                TD_param_num++;
                TD_param_num++;
            }
            break;
        }
        else if (factory_gateway_set[12] == 28)   //表压液位LWP5050GD, LWP5050测量大气压力（通道0 光照）
        {
            collector_temp = PRESSUE_level2();//空气湿度;PA1是数据口，PC3为CLK   输出pa=(65000*collector_temp）/65536-10000
            collector_temp = (65000 * collector_temp) / 65535;
            if (collector_temp > 9000 && collector_temp < 11000)  	//如果collector_temp>=61440,表示没有接传感器，最大差压：-10000~50937pa
            {
                wgcollector_data_buff[TD_param_num++] = collector_temp & 0x00ff;//表压液位  大气压力= collector_temp/K=100+b=0
                wgcollector_data_buff[TD_param_num++] = (collector_temp & 0xff00) >> 8;
            }
            else
            {
                TD_param_num++;           	                      	//光照度低位
                TD_param_num++;   		                           //光照度高位
            }
            break;
        }

        break;
    case 1:
        if (factory_gateway_set[15] == 2)   //空气温湿度输入
        {
            collector_temp = SI7021_HumiMeasurement();//空气湿度;PA1是数据口，PC3为CLK
            if (collector_temp >= 0xD916)
            {
                collector_temp = 0xD916;
            }
            wgcollector_data_buff[TD_param_num++] = collector_temp & 0x00ff;
            wgcollector_data_buff[TD_param_num++] = (collector_temp & 0xff00) >> 8;

            collector_temp = SI7021_TempMeasurement();//空气温度
            wgcollector_data_buff[TD_param_num++] = collector_temp & 0x00ff;
            wgcollector_data_buff[TD_param_num++] = (collector_temp & 0xff00) >> 8;
            break;
        }
        else if (factory_gateway_set[15] == 10)   //液位压力cps120
        {
            collector_temp = GET_level1();//空气湿度;PA1是数据口，PC3为CLK
            if (collector_temp != 0xffff)
            {
                wgcollector_data_buff[TD_param_num++] = collector_temp & 0x00ff;//大气压力  触摸屏k=182.04  b=30；单位：kpa
                wgcollector_data_buff[TD_param_num++] = (collector_temp & 0xff00) >> 8;
                wgcollector_data_buff[TD_param_num++] = level1_temperature & 0x00ff; //空气温度；触摸屏k=99.30  b=-40；单位：℃）
                wgcollector_data_buff[TD_param_num++] = (level1_temperature & 0xff00) >> 8;
                absolute_pressure_upper = collector_temp;
            }
            else
            {
                TD_param_num++;
                TD_param_num++;
                TD_param_num++;
                TD_param_num++;
            }
            break;
        }

        else if (factory_gateway_set[15] == 12)   //表压液位LWP5050GD,公式 输出pa=(65000*collector_temp）/65536-10000
        {
            collector_temp = PRESSUE_level1();//空气湿度;PA1是数据口，PC3为CLK
            if (collector_temp <= 0xF000)  	//如果collector_temp>=61440,表示没有接传感器，最大差压：-10000~50937pa
            {
                collector_temp = collector_temp - (factory_gateway_set[16] * 100);
                collector_temp = collector_temp + (factory_gateway_set[17] * 100);
                wgcollector_data_buff[TD_param_num++] = collector_temp & 0x00ff;//表压液位 ；
                wgcollector_data_buff[TD_param_num++] = (collector_temp & 0xff00) >> 8;
                wgcollector_data_buff[TD_param_num++] = level1_temperature & 0x00ff; //空气温度；）
                wgcollector_data_buff[TD_param_num++] = (level1_temperature & 0xff00) >> 8;
                absolute_pressure_upper = collector_temp;
                if (absolute_pressure_upper >= (10082 - 1000) && absolute_pressure_upper <= (10082 + 1000))   //（2500）-2.28kpa~+2.28kpa
                {
                    pressure_correct++;
                    absolute_pressure_zero[pressure_correct] = absolute_pressure_upper;
                    if (pressure_correct > 36)   //5*36=180s,pressure_correct最大不超过50，否则修改absolute_pressure_zero数组
                    {
                        u32 sum_pressure_lower;
                        u8 i;
                        sum_pressure_lower = 0;
                        for (i = 0; i < pressure_correct; i++)
                        {
                            sum_pressure_lower = sum_pressure_lower + absolute_pressure_zero[i];
                        }
                        absolute_pressure_lower = sum_pressure_lower / pressure_correct;
                        pressure_correct = 0;
                    }
                }
                else     //if(absolute_pressure_upper>(10082+1500))
                {
                    pressure_correct = 0;
                }
            }
            else
            {
                TD_param_num++;
                TD_param_num++;
                TD_param_num++;
                TD_param_num++;
            }
            break;
        }
        else if (factory_gateway_set[15] == 5)   //大气压力
        {
            collector_temp = GET_PRESSUE1();
            if (collector_temp != 0xffff)
            {
                wgcollector_data_buff[TD_param_num++] = collector_temp & 0x00ff;           		//大气压力低位
                wgcollector_data_buff[TD_param_num++] = (collector_temp & 0xff00) >> 8;      		//大气压力高位
            }
            else
            {
                TD_param_num++;
                TD_param_num++;
            }
            break;
        }
        else if (factory_gateway_set[15] == 6)   //开关量输入
        {
            collector_temp = PAin(1);//空气温湿度数据脚PA0
            wgcollector_data_buff[TD_param_num++] = collector_temp & 0x00ff;        //输入开关量=0，则collector_data_buff[0]=0x00
            wgcollector_data_buff[TD_param_num++] = (collector_temp & 0xff00) >> 8;   //输入开关量=0，则collector_data_buff[1]=0x00
            break;
        }
        else if (factory_gateway_set[15] == 7)   //频率输入
        {
            u8 i;
            collector_temp = 0;
            for (i = freq_I + 1; i <= 60; i++)
            {
                collector_temp = collector_temp + TIM2_FrequencyPA1[i];
            }
            for (i = 0; i < freq_I; i++)
            {
                collector_temp = collector_temp + TIM2_FrequencyPA1[i];
            }

            wgcollector_data_buff[TD_param_num++] = collector_temp & 0x00ff;        //输入频率字节低
            wgcollector_data_buff[TD_param_num++] = (collector_temp & 0xff00) >> 8;   //输入频率字节高；每分钟脉冲数，除以60为频率，水肥除以12
            break;
        }
        else if (factory_gateway_set[15] == 18)   //脉冲输入；超声波液位检测
        {
            u8 i, j;
            u16 t;
            PA1_pulse_time = 0;
            TIM5_pulsePA1_NUM = factory_gateway_set[12 + 3 * 5 + 2];//参数8滤波次数
            if (TIM5_pulsePA1_NUM < 12)
            {
                TIM5_pulsePA1_NUM = 12;
            }
            if (TIM5_pulsePA1_NUM > 50)
            {
                TIM5_pulsePA1_NUM = 50;
            }
            memcpy(TIM5_sort_pulsePA1, TIM5_pulsePA1, 4 * TIM5_pulsePA1_NUM);//TIM2_pulsePA1_NUM=32*2字节
            for (i = 0; i < TIM5_pulsePA1_NUM; i++)   //32个数排序
            {
                for (j = i + 1; j < TIM5_pulsePA1_NUM; j++)
                {
                    if (TIM5_sort_pulsePA1[i] > TIM5_sort_pulsePA1[j])   //TIM2_sort_pulsePA1排序数组
                    {
                        t = TIM5_sort_pulsePA1[i];
                        TIM5_sort_pulsePA1[i] = TIM5_sort_pulsePA1[j];
                        TIM5_sort_pulsePA1[j] = t;
                    }
                }
            }
            for (i = 5; i < TIM5_pulsePA1_NUM - 5; i++)
            {
                PA1_pulse_time = PA1_pulse_time + TIM5_sort_pulsePA1[i];
            }
            collector_temp = (PA1_pulse_time * 340) / (200 * (TIM5_pulsePA1_NUM - 10)); //单位：0.1mm
            temp_level = collector_temp;//给通道5使用；参数8，计算车载实际液位
            wgcollector_data_buff[TD_param_num++] = collector_temp & 0x00ff;        //输入频率字节低
            wgcollector_data_buff[TD_param_num++] = (collector_temp & 0xff00) >> 8;   //输入频率字节高；每分钟脉冲数，除以60为频率
            break;
        }
        else if (factory_gateway_set[15] == 8)   //0~1VDC输入,硬件取消50欧姆电阻
        {
            TD_param_num++;
            TD_param_num++;
            break;
        }
        else if (factory_gateway_set[15] == 23)   //感应雨雪传感器输入,硬件取消50欧姆电阻
        {
            u8 i;
            collector_temp = 0;
            for (i = 0; i < 180; i++)   //180S雨滴数
            {
                collector_temp = collector_temp + report_last_rain[i];
            }
            wgcollector_data_buff[TD_param_num++] = collector_temp & 0x00ff;        //输入频率字节低
            wgcollector_data_buff[TD_param_num++] = (collector_temp & 0xff00) >> 8;   //输入频率字节高；每分钟脉冲数，除以60为频率
            break;
        }


        break;

    case 2:
        if (factory_gateway_set[18] == 3 || factory_gateway_set[18] == 8)
        {
            TD_number = 0;
            if (factory_gateway_set[18] == 8)
            {
                temp_adc = Get_Adclvbo(TD_number, 0, 1305);
            }
            else
            {
                temp_adc = Get_Adclvbo(TD_number, 262, 1305);
            }
            if (First_adc_average[TD_number] == 0)
            {
                temp_adc = First_Getaverage(TD_number, Maxlvbo_number, temp_adc);
                First_adc_average[TD_number] = 1;
            }
            TDlvbo_number[TD_number] = factory_gateway_set[12 + 2 * 3 + 2];//取出触摸屏设定的滤波次数低字节
            temp_adc = TD_Getaverage(TD_number, TDlvbo_number[TD_number], temp_adc, tdcycle_i[TD_number]);
            First_adc_average[TD_number] = 1;
            tdcycle_i[TD_number]++;
            if (tdcycle_i[TD_number] >= TDlvbo_number[TD_number])
            {
                tdcycle_i[TD_number] = 0;
            }
            collector_temp = temp_adc;//土壤水分
            dp_temp_level = temp_adc;//高静压差压，给通道5使用；参数8，计算车载实际液位
            dr_temp_level = temp_adc;//电容射频，给通道5使用；参数8，计算车载实际液位
            wgcollector_data_buff[TD_param_num++] = collector_temp & 0x00ff;         		 //土壤水分低位
            wgcollector_data_buff[TD_param_num++] = (collector_temp & 0xff00) >> 8;
        }
        break;

    case 3:
        if (factory_gateway_set[21] == 3 || factory_gateway_set[21] == 8)
        {
            TD_number = 1;
            if (factory_gateway_set[21] == 8)
            {
                temp_adc = Get_Adclvbo(TD_number, 0, 1305);
            }
            else
            {
                temp_adc = Get_Adclvbo(TD_number, 262, 1305);
            }
            if (First_adc_average[TD_number] == 0)
            {
                temp_adc = First_Getaverage(TD_number, Maxlvbo_number, temp_adc);
                First_adc_average[TD_number] = 1;
            }
            TDlvbo_number[TD_number] = factory_gateway_set[12 + 3 * 3 + 2];//取出触摸屏设定的滤波次数低字节，滤波次数最大255次
            temp_adc = TD_Getaverage(TD_number, TDlvbo_number[TD_number], temp_adc, tdcycle_i[TD_number]);
            First_adc_average[TD_number] = 1;
            tdcycle_i[TD_number]++;
            if (tdcycle_i[TD_number] >= TDlvbo_number[TD_number])
            {
                tdcycle_i[TD_number] = 0;
            }
            collector_temp = temp_adc; //土壤温度
            wgcollector_data_buff[TD_param_num++] = collector_temp & 0x00ff;          	//土壤温度低位
            wgcollector_data_buff[TD_param_num++] = (collector_temp & 0xff00) >> 8;    	//土壤温度高位
        }
        break;

    case 4:
        if (factory_gateway_set[24] == 4 && factory_gateway_set[15] != 18)   //二氧化碳输入
        {
            collector_temp = Get_Carbon();//二氧化碳
            if (collector_temp < 60000)
            {
                wgcollector_data_buff[TD_param_num++] = collector_temp & 0x00ff;          	//二氧化碳低位
                wgcollector_data_buff[TD_param_num++] = (collector_temp & 0xff00) >> 8;    	//二氧化碳高位
            }
            else
            {
                TD_param_num++;                             	//二氧化碳低位
                TD_param_num++;                             	//二氧化碳高位
            }
            break;
        }
        else if (factory_gateway_set[24] == 5 && factory_gateway_set[15] != 18)   //大气压力
        {
            collector_temp = GET_PRESSUE4();
            if (collector_temp != 0xffff)
            {
                wgcollector_data_buff[TD_param_num++] = collector_temp & 0x00ff;           		//大气压力低位
                wgcollector_data_buff[TD_param_num++] = (collector_temp & 0xff00) >> 8;      		//大气压力高位
            }
            else
            {
                TD_param_num++;
                TD_param_num++;
            }
            break;
        }
        else  if (factory_gateway_set[24] == 6)   //开关量输入
        {
            collector_temp = PAin(0);//二氧化碳数据脚PA0
            wgcollector_data_buff[TD_param_num++] = collector_temp & 0x00ff;           		//输入开关量=1，则collector_data_buff[10]=0x01
            wgcollector_data_buff[TD_param_num++] = (collector_temp & 0xff00) >> 8;
            break;
        }
        else if (factory_gateway_set[24] == 7)   //频率输入
        {
            u8 i;
            collector_temp = 0;
            for (i = freq_I + 1; i <= 60; i++)
            {
                collector_temp = collector_temp + TIM2_FrequencyPA0[i];
            }
            for (i = 0; i < freq_I; i++)
            {
                collector_temp = collector_temp + TIM2_FrequencyPA0[i];
            }
            wgcollector_data_buff[TD_param_num++] = collector_temp & 0x00ff;           //输入频率字节低
            wgcollector_data_buff[TD_param_num++] = (collector_temp & 0xff00) >> 8;     //输入频率字节高；每分钟脉冲数，除以60为频率，水肥除以12
            break;
        }
        else if (factory_gateway_set[24] == 8)   //0~1VDC输入,硬件取消50欧姆电阻
        {
            TD_param_num++;
            TD_param_num++;
            break;
        }
        else if (factory_gateway_set[24] == 10 && factory_gateway_set[15] != 18)   //液位压力cps120
        {
            collector_temp = GET_level4();
            if (collector_temp != 0xffff)
            {
                wgcollector_data_buff[TD_param_num++] = collector_temp & 0x00ff;           		//液位压力低位，触摸屏k=193.16 b=30；单位：kpa
                wgcollector_data_buff[TD_param_num++] = (collector_temp & 0xff00) >> 8;      		//液位压力高位
            }
            else
            {
                TD_param_num++;
                TD_param_num++;
            }
            break;

        }

        break;

    case 5:
        if (factory_gateway_set[27] == 11 || factory_gateway_set[27] == 16)   //差压计算
        {
            if (absolute_pressure_upper >= absolute_pressure_lower)
            {
                temp_flow = absolute_pressure_upper - absolute_pressure_lower;
                TD_number = 2;
                if (temp_flow < 300)
                {
                    First_adc_average[TD_number] = 0;//是否第一次进入，是=0
                    temp_flow = 0;
                }
                if (First_adc_average[TD_number] == 0)
                {
                    temp_flow = First_Getaverage(TD_number, Maxlvbo_number, temp_flow);
                    First_adc_average[TD_number] = 1;
                }
                TDlvbo_number[TD_number] = factory_gateway_set[12 + 5 * 3 + 2];//取出触摸屏设定的差压滤波次数低字节
                temp_flow = TD_Getaverage(TD_number, TDlvbo_number[TD_number], temp_flow, tdcycle_i[TD_number]);
                //			      First_adc_average[TD_number]=1;
                tdcycle_i[TD_number]++;
                if (tdcycle_i[TD_number] >= TDlvbo_number[TD_number])
                {
                    tdcycle_i[TD_number] = 0;
                }
                if (factory_gateway_set[13] != 0 && factory_gateway_set[14] != 0)
                {
                    temp_flow = (factory_gateway_set[14] * temp_flow) / factory_gateway_set[13];
                }
                collector_temp = temp_flow;//
                wgcollector_data_buff[14] = collector_temp & 0x00ff;           		//差压低位，触摸屏k=1893.72  b=0；单位：M
                wgcollector_data_buff[15] = (collector_temp & 0xff00) >> 8;      		//差压高位
            }
            break;
        }
        else if (factory_gateway_set[27] == 19 || factory_gateway_set[27] == 17)   //脉冲超声波车载液位高度计算
        {
            temp_level_up = factory_gateway_set[28] * 100;//加压车超声波检测液位，参数8的变量个数为净高度，单位：cm；这里转换为0.1mm
            if (temp_level > temp_level_up)
            {
                temp_level = temp_level_up;
            }
            if (temp_level < 3000)
            {
                temp_level = 0;   //死区设定
            }
            temp_levelS = temp_level_up - temp_level;
            wgcollector_data_buff[14] = temp_levelS & 0x00ff;
            wgcollector_data_buff[15] = (temp_levelS & 0xff00) >> 8;
            break;
        }

        else if (factory_gateway_set[27] == 20)   //高静压差压车载液位高度计算
        {
            temp_level_up = factory_gateway_set[28] * 100;//加压车超声波检测液位，参数8的变量个数为净高度，单位：cm；这里转换为0.1mm
            temp_levelS = (dp_temp_level - 262)*(20000 / (1305 - 262));
            if (temp_levelS < 0)
            {
                temp_levelS = 0;   //死区设定
            }
            if (temp_levelS > temp_level_up)
            {
                temp_levelS = temp_level_up;
            }
            temp_levelS = temp_levelS + 800;
            if (temp_levelS < 2000 && temp_levelS >= 0)temp_levelS = 0;//液位小于80mm，则认为0，液位自动校零
            wgcollector_data_buff[14] = temp_levelS & 0x00ff;
            wgcollector_data_buff[15] = (temp_levelS & 0xff00) >> 8;
            break;
        }

        if (factory_gateway_set[27] == 21)   //电容射频车载液位高度计算
        {
            u32 temp_js;
            temp_level_up = factory_gateway_set[28] * 100;//加压车超声波检测液位，参数8的变量个数为净高度，单位：cm；这里转换为0.1mm
            temp_js = (dr_temp_level - 262)*factory_gateway_set[19] * 100;//利用土壤水分的参数个数调整检测值
            temp_levelS = temp_js / (1305 - 262);
            if (temp_levelS < 0)
            {
                temp_levelS = 0;   //零点下漂移，自动清零
            }
            temp_levelS = temp_levelS + 800;
            if (temp_levelS > temp_level_up)
            {
                temp_levelS = temp_level_up;   //不能超过超过净高值
            }

            //液位小于factory_gateway_set[29]设定的下，则认为0，液位自动校零;lag_value防止零点反复震荡
            if (temp_levelS < ((factory_gateway_set[29] * 100) + lag_value))
            {
                temp_levelS = 0;
                lag_value = 1000;
            }
            if (temp_levelS > (temp_level_up - 3000))
            {
                lag_value = 0;   //重新装车接近满，调整迟滞区lag_value的值
            }
            wgcollector_data_buff[14] = temp_levelS & 0x00ff;
            wgcollector_data_buff[15] = (temp_levelS & 0xff00) >> 8;
            break;
        }

        if (factory_gateway_set[27] == 22 && TD_param_num <= 6)   //风速、风向；PM值
        {
            wgcollector_data_buff[TD_param_num++] = data_buf_FXFS[1];//风向低字节
            wgcollector_data_buff[TD_param_num++] = data_buf_FXFS[0];//风向高字节
            wgcollector_data_buff[TD_param_num++] = data_buf_FXFS[3];//风速低字节
            wgcollector_data_buff[TD_param_num++] = data_buf_FXFS[2];//风速高字节

            wgcollector_data_buff[TD_param_num++] = data_buf_PM[1];//PM1.0低字节
            wgcollector_data_buff[TD_param_num++] = data_buf_PM[0];//PM1.0高字节
            wgcollector_data_buff[TD_param_num++] = data_buf_PM[3];//PM2.5低字节
            wgcollector_data_buff[TD_param_num++] = data_buf_PM[2];//PM2.5高字节
            wgcollector_data_buff[TD_param_num++] = data_buf_PM[5];//PM2.5低字节
            wgcollector_data_buff[TD_param_num++] = data_buf_PM[4];//PM2.5高字节
            break;
        }
        if (factory_gateway_set[27] == 29)   //土壤RS485传感器
        {
            u8 i;
            u16 temp_value;
            for (i = 0; i < factory_gateway_set[28]; i++)
            {
                if (factory_gateway_set[29] > 0 && TD_param_num < 15)
                {
                    wgcollector_data_buff[TD_param_num++] = data_RS485[i][1];//土壤水分低
                    wgcollector_data_buff[TD_param_num++] = data_RS485[i][0];//土壤水分高
                }
                if (factory_gateway_set[29] > 1 && factory_gateway_set[29] <= 3 && TD_param_num < 15)
                {
                    temp_value = 400 + (data_RS485[i][2] << 8 | data_RS485[i][3]);//温度量程：-40~90℃；转换成无符号整数0~130；温度=（temp_value-400)/10=temp_value/10-40
                    wgcollector_data_buff[TD_param_num++] = temp_value & 0x00ff;
                    wgcollector_data_buff[TD_param_num++] = (temp_value & 0xff00) >> 8;
                }
                if (factory_gateway_set[29] > 2 && factory_gateway_set[29] <= 3 && TD_param_num < 15)
                {
                    wgcollector_data_buff[TD_param_num++] = data_RS485[i][5];//土壤EC低
                    wgcollector_data_buff[TD_param_num++] = data_RS485[i][4];//土壤EC高
                }
            }
            break;
        }
        if (factory_gateway_set[27] == 30)   //RS485气体传感器
        {
            u8 i;
            for (i = 0; i < factory_gateway_set[28]; i++)
            {
                if (factory_gateway_set[29] > 0 && TD_param_num < 15)
                {
                    wgcollector_data_buff[TD_param_num++] = data_RS485[i][1];//气体浓度低
                    wgcollector_data_buff[TD_param_num++] = data_RS485[i][0];//气体浓度高
                }
            }
            break;
        }
        if (factory_gateway_set[27] == 31)   //研华触摸屏+RS485水质传感器；
        {
            u8 i;
            for (i = 0; i < factory_gateway_set[28]; i++)
            {
                if (factory_gateway_set[29] > 0 && TD_param_num < 15)
                {
                    wgcollector_data_buff[TD_param_num++] = water_param_buf[2 * i];//参数低字节
                    wgcollector_data_buff[TD_param_num++] = water_param_buf[2 * i + 1];//参数高字节
                }
            }
            break;
        }
    default:
        break;
    }
    memcpy(Collectors[64], wgcollector_data_buff, 16);

    if (ReadDataCNT >= 5)
    {
        ReadDataCNT = 0;
    }
    else
    {
        ReadDataCNT++;
    }
    /*以下为液位---流量导出*/
    if (factory_gateway_set[27] == 16 || factory_gateway_set[27] == 17)
    {
        u8  flow_temp[12];
        bytelen_to_asc((unsigned char *)flow_temp, wgcollector_data_buff[15]);//液位高字节 pa
        bytelen_to_asc((unsigned char *)flow_temp + 3, wgcollector_data_buff[14]);//液位低字节 pa
        bytelen_to_asc((unsigned char *)flow_temp + 6, wgcollector_data_buff[13]);//累积量高字节 升
        bytelen_to_asc((unsigned char *)flow_temp + 9, wgcollector_data_buff[12]);//累积量低字节 升

        memcpy(USART1SendTCB, flow_temp, 12);
        WriteDataToDMA_BufferTX1(12);
    }
}

static void startadc(void)
{
    DMA_Cmd(DMA1_Channel1, ENABLE);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);  //连续转换开始，ADC通过DMA方式不断的更新RAM区。
}

u16 Get_Adclvbo(u8 TD_Xnumber, u16 TD_xiaxian, u16 TD_shangxian)
{
    u8 i, j;
    u16 AdcLvbo[100], Temp_adc, t;
    u32 Temp_adcto = 0;
    for (i = 0; i < 100; i++)
    {
        Temp_adc = ADC_ConvertedValue[TD_Xnumber];
        if (Temp_adc < TD_xiaxian)   //过滤
            Temp_adc = TD_xiaxian;
        if (Temp_adc > TD_shangxian)	 //过滤
            Temp_adc = TD_shangxian;
        AdcLvbo[i] = Temp_adc;
    }

    for (i = 0; i < 100; i++)   //100个数排序
    {
        for (j = i + 1; j < 100; j++)
        {
            if (AdcLvbo[i] >= AdcLvbo[j])
            {
                t = AdcLvbo[i];
                AdcLvbo[i] = AdcLvbo[j];
                AdcLvbo[j] = t;
            }
        }
    }
    for (i = 40; i < 60; i++)
    {
        Temp_adcto = Temp_adcto + AdcLvbo[i];
    }
    Temp_adcto = Temp_adcto / 20;

    return Temp_adcto;
}

u16 First_Getaverage(u8 td_xnumber, u8 maxlvbo_xnumber, u16 temp_adc)
{
    u8 i;
    for (i = 0; i < maxlvbo_xnumber; i++)
    {
        Adc_average[td_xnumber][i] = temp_adc;
    }
    return temp_adc;
}

u16 TD_Getaverage(u8 td_xnumber, u8 tdlvbo_xnumber, u16 temp_xadc, u8 tdcycle_xi)
{
    u8 i;
    u32 average_adcto = 0;
    Adc_average[td_xnumber][tdcycle_xi] = temp_xadc;
    if (tdlvbo_xnumber == 0)
    {
        tdlvbo_xnumber = 1;
    }
    for (i = 0; i < tdlvbo_xnumber; i++)
    {
        average_adcto = average_adcto + Adc_average[td_xnumber][i];  //求和
    }
    temp_xadc = average_adcto / tdlvbo_xnumber;  //求平均值
    return temp_xadc;
}
//六参数采集程序定义结束

// 浮点数转换成字符串
// 参数：data
// 返回：str
static u8 float_to_string(float data, u8 *str)
{
    int i, j, k;
    float data_temp;
    long temp, tempoten;
    u8 intpart[20], dotpart[20];  // 数的整数部分和小数部分

    //1.确定符号位
    if (data < 0)
    {
        str[0] = '-';
        data = -data;
    }
    else str[0] = ' ';
    // 小数第3位≥5，先进位
    data_temp = data * 100.0f - (long)(data*100.0f);
    if (data_temp >= 0.5)
    {
        data = data + 0.01f;
    }
    //2.确定整数部分
    temp = (long)data;

    i = 0;
    tempoten = temp / 10;
    while (tempoten != 0)
    {
        intpart[i] = temp - 10 * tempoten + 48; //to ascii code
        temp = tempoten;
        tempoten = temp / 10;
        i++;
    }
    intpart[i] = temp + 48;

    //3.确定小数部分,取了2位小数
    data = data - (long)data;
    for (j = 0; j < 2; j++)
    {
        dotpart[j] = (int)(data * 10) + 48;
        data = data * 10.0;
        data = data - (long)data;
    }
    //4.数据组装
    for (k = 1; k <= i + 1; k++) str[k] = intpart[i + 1 - k];
    str[i + 2] = '.';
    for (k = i + 3; k < i + j + 3; k++) str[k] = dotpart[k - i - 3];
    return i + j + 3;
    //  str[i+j+3]=0x0D;
}

static void IOxh_send_cmd(void)
{
    if (factory_gateway_set[15] == 18)   //有PA1-DATA4的脉冲宽度检测，打开TIM2及相应的中断
    {
        level_num_err++;
        if (level_num_err > 5)
        {
            while (1);   //如果20次发送检测请求命令，没有收到TIME5D1中断,则系统重启。在TIME5中断程序中level_num_err清零
        }
        TIM5->CCER &= ~(1 << 5);//上升沿捕获
        PCout(3) = 1;
        Delayus(2000);
        PCout(3) = 0;
    }
    if (factory_gateway_set[27] == 15)   //串口1超声波液位函数
    {
        memcpy(USART1SendTCB, uart1_cmd_csb, 1);//ReportData1
        WriteDataToDMA_BufferTX1(1);
    }
}
//串口5函数
static void uart5_send_cmd(void)
{
    if (factory_gateway_set[27] == 13)
    {
        WriteDataToBuffer(5, (u8 *)uart5_cmd_csb, 0, 8);    //超声波流量计
        return;
    }
    if (factory_gateway_set[27] == 14 || factory_gateway_set[27] == 16 || factory_gateway_set[27] == 17)
    {
        WriteDataToBuffer(5, (u8 *)uart5_cmd_dc, 0, 8);    //电磁流量计
        return;
    }
    if (factory_gateway_set[27] == 34)           //上海帆扬 水肥机总管流量查询指令，流量信息在rxreport5里接收，需下发
    {
        WriteDataToBuffer(5, (u8 *)uart5_cmd_fydc, 0, 8);
        return;
    }
    if (factory_gateway_set[27] == 33)           //泰州 凯达水肥机总管流量查询指令，流量信息在rxreport5里接收，需下发
    {
        WriteDataToBuffer(5, (u8 *)uart5_cmd_tzkd, 0, 8);
        return;
    }
    if (factory_gateway_set[27] == 22)
    {
        if (switch_cmd_addr == 0)
        {
            WriteDataToBuffer(5, (u8 *)uart5_cmd_FSFX, 0, 8);   //风向+风速
        }
        if (switch_cmd_addr == 1)
        {
            WriteDataToBuffer(5, (u8 *)uart5_cmd_PM, 0, 8);   //PM1.0+PM2.5+PM10
        }
        if (switch_cmd_addr >= 1)
        {
            switch_cmd_addr = 0;
        }
        else
        {
            switch_cmd_addr++;
        }
        return;
    }
    if (factory_gateway_set[27] == 25)
    {
        WriteDataToBuffer(5, (u8 *)uart5_cmd_LEL, 0, 8);    //可燃气体报警器
        return;
    }
    if (factory_gateway_set[27] == 35)   //中航LED屏 数据显示
    {
        //LEDShow();  //串口5每两秒发送一次
        return;
    }


    if (factory_gateway_set[27] == 29)   //RS485土壤水分+温度+EC
    {
        u8 bytelen5;
        bytelen5 = ReadData(switch_cmd_TR485_addr, READ_HOLDING_REGISTER, 0x0000, factory_gateway_set[29], ReportData5);
        WriteDataToBuffer(5, (u8 *)ReportData5, 0, bytelen5);
        switch_cmd_RS485_CNT++;
        if (switch_cmd_RS485_CNT < factory_gateway_set[28])   //RS485土壤水分+温度+EC(土壤传感器站地址从0xFE开始，递减地址)
        {
            switch_cmd_TR485_addr--;
        }
        else
        {
            switch_cmd_TR485_addr = 0xFE;
            switch_cmd_RS485_CNT = 0;
        }
        return;
    }
    if (factory_gateway_set[27] == 30)   //RS485气体传感器（威海精讯畅通）；
    {
        u8 bytelen5;
        bytelen5 = ReadData(switch_cmd_RS485_addr, READ_HOLDING_REGISTER, 0x0006, 0x0001, ReportData5);
        WriteDataToBuffer(5, (u8 *)ReportData5, 0, bytelen5);
        switch_cmd_RS485_CNT++;
        if (switch_cmd_RS485_CNT < factory_gateway_set[28])   //RS485气体传感器个数，
        {
            switch_cmd_RS485_addr++;
        }
        else
        {
            switch_cmd_RS485_addr = 0x01;
            switch_cmd_RS485_CNT = 0;
        }
        return;
    }
    if (factory_gateway_set[27] == 31)   //研华触摸屏+RS485水质传感器；
    {
        u8 bytelen5;
        switch (switch_cmd_RS485_addr)
        {
        case 1:
            if (switch_cmd_RS485_CNT == 0)
            {
                bytelen5 = ReadData(switch_cmd_RS485_addr, READ_HOLDING_REGISTER, 0x0000, 0x0002, ReportData5);//读取液位高度；单位：米		K=100;B=0
                switch_cmd_RS485_CNT = 1;
            }
            else if (switch_cmd_RS485_CNT == 1)
            {
                bytelen5 = ReadData(switch_cmd_RS485_addr, READ_HOLDING_REGISTER, 0x0018, 0x0002, ReportData5);//读取每秒顺时流量，单位：L/S	K=10;B=0
                switch_cmd_RS485_CNT = 0;
                switch_cmd_RS485_addr++;
            }
            WriteDataToBuffer(5, (u8 *)ReportData5, 0, bytelen5);
            break;
        case 2:
            bytelen5 = ReadData(switch_cmd_RS485_addr, READ_HOLDING_REGISTER, 0x0000, 0x0001, ReportData5);//读取pH，K=100,B=0
            WriteDataToBuffer(5, (u8 *)ReportData5, 0, bytelen5);
            switch_cmd_RS485_addr = switch_cmd_RS485_addr + 2;
            break;
        case 4:
            bytelen5 = ReadData(switch_cmd_RS485_addr, READ_HOLDING_REGISTER, 0x0000, 0x0002, ReportData5);//读取电导率，K=10,B=0	（0~2000uS,一位小数）
            WriteDataToBuffer(5, (u8 *)ReportData5, 0, bytelen5);
            switch_cmd_RS485_addr = switch_cmd_RS485_addr + 4;
            break;
        case 8:
            bytelen5 = ReadData(switch_cmd_RS485_addr, READ_HOLDING_REGISTER, 0x0000, 0x0001, ReportData5);//读取浊度，K=1,B=0	(0~1000NTU，无小数小数)
            WriteDataToBuffer(5, (u8 *)ReportData5, 0, bytelen5);
            switch_cmd_RS485_addr = switch_cmd_RS485_addr + 2;
            break;
        case 10:
            if (switch_cmd_RS485_CNT == 0)
            {
                bytelen5 = ReadData(switch_cmd_RS485_addr, READ_HOLDING_REGISTER, 0x0000, 0x0002, ReportData5);//读取溶解氧；单位：mg/L	 K=100,B=0
                switch_cmd_RS485_CNT = 1;
            }
            else if (switch_cmd_RS485_CNT == 1)
            {
                bytelen5 = ReadData(switch_cmd_RS485_addr, READ_HOLDING_REGISTER, 0x0004, 0x0002, ReportData5);//读取温度，单位：℃	K=10;B=0
                switch_cmd_RS485_CNT = 0;
                switch_cmd_RS485_addr = 0x01;
            }
            WriteDataToBuffer(5, (u8 *)ReportData5, 0, bytelen5);
            break;
        default:
            break;
        }
        return;
    }
}

static void RxReport5(u8 len, u8 *pData)
{
    if (GetCRC16(pData, len) == 0)
    {
        if (factory_gateway_set[27] == 13)   //超声波流量计
        {
            if (pData[0] == 0x01 && pData[1] == 0x03)  					//收到传感器返回的数据
            {
                //流量计回复瞬时流量，地址：0x01
                u32 temp;
                memcpy(uart5_data_buf, pData + 3, 4);	// 低字节Q_water_data[0]~Q_water_data[3];单位：升/小时
                temp = (u32)uart5_data_buf[3] << 24 | (u32)uart5_data_buf[2] << 16 | (u32)uart5_data_buf[1] << 8 | (u32)uart5_data_buf[0];
                temp = temp >> 4;//先除以16；平台公式再除62.5,转换成单位为：立方米/小时
                uart5_data_buf[0] = (u8)(temp & 0x000000FF);
                uart5_data_buf[1] = (u8)((temp >> 8) & 0x000000FF);
                wgcollector_data_buff[12] = uart5_data_buf[0];
                wgcollector_data_buff[13] = uart5_data_buf[1];
                //			memcpy(Collectors[pData[0]-1],pData+3,12);	//  01 03 0C |-- --| -- --| -- --| -- --| -- --| CRC CRC
                //上面指令相当于从memcpy[子站号][0]开始存放12个数据;pData[0]=子站号+1；pData[1]=0x03功能号；pData[2]=参数个数(每个参数2字节)
                //			Start_timerEx( WGSEND_SLAVECMD_EVT, 80 );
            }
            return;
        }

        if (factory_gateway_set[27] == 14 || factory_gateway_set[27] == 16 || factory_gateway_set[27] == 17)   //电磁流量计
        {
            if (pData[0] == 0x01 && pData[1] == 0x03)  					//收到传感器返回的数据
            {
                u32 temp;
                memcpy(uart5_data_buf, pData + 3, 4);	// 低字节Q_water_data[0]~Q_water_data[3];累计流量 单位：升
                temp = (u32)uart5_data_buf[0] << 24 | (u32)uart5_data_buf[1] << 16 | (u32)uart5_data_buf[2] << 8 | (u32)uart5_data_buf[3];
                uart5_data_buf[0] = (u8)(temp & 0x000000FF);
                uart5_data_buf[1] = (u8)((temp >> 8) & 0x000000FF);
                wgcollector_data_buff[12] = uart5_data_buf[0];
                wgcollector_data_buff[13] = uart5_data_buf[1];
            }
            return;
        }

        if (factory_gateway_set[27] == 34)   //上海 帆扬水肥电磁流量计，测总管瞬时流量，浮点数，单位为T/H ，放大100倍即保留两位小数 注意支管流量为L/H
        {
            if (pData[0] == 0x01 && pData[1] == 0x03)  					//收到传感器返回的数据
            {

                u32 SF_Flow_Total_temp = 0;
                SF_Flow_Total_temp =  (pData[3]<<24) | (pData[4]<<16) | (pData[5]<<8) |(pData[6]);
                SF_Flow_Total[SF_Flow_Total_I] = (u16)(*((float *)(&SF_Flow_Total_temp)) * 100) ; //将浮点数扩大100倍，即保留两位小数

                if(SF_Flow_Total_flg == 0)   //前6个瞬时流量直接下发
                {
                    wgcollector_data_buff[12] =(u8) (SF_Flow_Total[SF_Flow_Total_I] & 0x00FF);
                    wgcollector_data_buff[13] = (u8) (SF_Flow_Total[SF_Flow_Total_I] >>8);
                    memcpy(SF_wgcollector_data_buff+6, &SF_Flow_Total[SF_Flow_Total_I],2);	//2字节流量整型数，放在[6][7]
                    SF_Flow_Total_I++;
                    if(SF_Flow_Total_I>5)
                    {
                        SF_Flow_Total_flg = 1;
                        SF_Flow_Total_I = 0;
                    }
                    return;
                }
                else if(SF_Flow_Total_flg == 1)     //后面流量取平均值下发
                {
                    u8 i;
                    u16 temp = 0;
                    for(i=0; i<6; i++)
                    {
                        temp += SF_Flow_Total[i];
                    }
                    temp /= 6;
                    wgcollector_data_buff[12] =(u8) (temp & 0x00FF);
                    wgcollector_data_buff[13] = (u8) (temp >>8);
                    memcpy(SF_wgcollector_data_buff+6, &temp, 2);
                    SF_Flow_Total_I++;
                    if(SF_Flow_Total_I>5)
                    {
                        SF_Flow_Total_I = 0;
                    }
                    return;

                }
            }
            return;
        }
        if (factory_gateway_set[27] == 33)   //泰州 凯达水肥电磁流量计，测总管瞬时流量，16进制整型数，单位为L/H，要转换成T/H，除以1000，由于k647=100，再除以10即可
        {
            if (pData[0] == 0x01 && pData[1] == 0x03)  					//收到传感器返回的数据
            {

                u32 SF_Flow_Total_temp = 0;
                SF_Flow_Total_temp =  (pData[3]<<24) | (pData[4]<<16) | (pData[5]<<8) |(pData[6]);
                SF_Flow_Total[SF_Flow_Total_I] = SF_Flow_Total_temp/10;  //2字节流量整型数,由于k647=100，再除以10即可，单位是T/H

                if(SF_Flow_Total_flg == 0)   //前6个瞬时流量直接下发
                {
                    wgcollector_data_buff[12] =(u8) (SF_Flow_Total[SF_Flow_Total_I] & 0x00FF);
                    wgcollector_data_buff[13] = (u8) (SF_Flow_Total[SF_Flow_Total_I] >>8);
                    memcpy(SF_wgcollector_data_buff+6, &SF_Flow_Total[SF_Flow_Total_I],2);	//2字节流量整型数，放在[6][7]
                    SF_Flow_Total_I++;
                    if(SF_Flow_Total_I>5)
                    {
                        SF_Flow_Total_flg = 1;
                        SF_Flow_Total_I = 0;
                    }
                    return;
                }
                else if(SF_Flow_Total_flg == 1)     //后面流量取平均值下发
                {
                    u8 i;
                    u16 temp = 0;
                    for(i=0; i<6; i++)
                    {
                        temp += SF_Flow_Total[i];
                    }
                    temp /= 6;
                    wgcollector_data_buff[12] =(u8) (temp & 0x00FF);
                    wgcollector_data_buff[13] = (u8) (temp >>8);
                    memcpy(SF_wgcollector_data_buff+6, &temp, 2);
                    SF_Flow_Total_I++;
                    if(SF_Flow_Total_I>5)
                    {
                        SF_Flow_Total_I = 0;
                    }
                    return;

                }
            }
            return;
        }
				
				/*
				u16 SF_Flow_Total[6];
				u8 SF_Flow_Total_I;
				u8 SF_Flow_Total_flg ;
				u8 SF_wgcollector_data_buff[16] ;
				*/

        if (factory_gateway_set[27] == 22)   //风向(k=1,b=0)+风速(k=100,b=0) PM1.0+PM2.5+PM10(k=1,b=0)
        {
            if (pData[0] == 0x01 && pData[1] == 0x03)  					//收到风速风向传感器返回的数据
            {
                u16 temp_FS;
                u8 	temp_FS_int[4];
                memcpy(uart5_data_buf, pData + 3, 6);	// 风向高字节 低字节+风速4字节浮点
                data_buf_FXFS[0] = uart5_data_buf[0];
                data_buf_FXFS[1] = uart5_data_buf[1];
                temp_FS_int[3] = uart5_data_buf[4];
                temp_FS_int[2] = uart5_data_buf[5];
                temp_FS_int[1] = uart5_data_buf[2];
                temp_FS_int[0] = uart5_data_buf[3];
                memcpy(data_buf_FS.wind_speed_int, temp_FS_int, 4);
                temp_FS = data_buf_FS.wind_speed_float * 100;
                data_buf_FXFS[2] = (temp_FS & 0xff00) >> 8;
                data_buf_FXFS[3] = temp_FS & 0x00ff;
            }
            if (pData[0] == 0x02 && pData[1] == 0x03)  					//收到PM1.0+PM2.5+PM10传感器返回的数据
            {
                memcpy(data_buf_PM, pData + 3, 6);	//PM1.0高字节 低字节+PM2.5高字节 低字节+PM10高字节 低字节
            }
            return;
        }
        if (factory_gateway_set[27] == 25)   //可燃气体报警器
        {
            if (pData[0] == 0x01 && pData[1] == 0x03)  					//收到传感器返回的数据
            {
                wgcollector_data_buff[12] = pData[4];//k=100,b=0,单位%
                wgcollector_data_buff[13] = pData[3];
            }
            return;
        }

        if (factory_gateway_set[27] == 29)   //RS485土壤水分+温度+EC
        {
            u8 temp_I;
            temp_I = 0xFE - pData[0];//站地址转换为数组下标;站地址从0xFE;0xFD;...;最多8个
            if (temp_I <= 7)
            {
                memcpy(data_RS485[temp_I], pData + 3, pData[2]);
            }
            return;
        }
        if (factory_gateway_set[27] == 30)   //RS485气体传感器
        {
            u8 temp_I;
            temp_I = pData[0] - 0x01;//站地址转换为数组下标;站地址从0x01;0x02;...;最多8个
            if (temp_I <= 7)
            {
                memcpy(data_RS485[temp_I], pData + 3, pData[2]);
            }
            return;
        }
        if (factory_gateway_set[27] == 31)   //研华触摸屏+RS485水质传感器；
        {
            u16 temp_data;
            u32 temp_data32;
            switch (pData[0])
            {
            case 1:
                water_param_data.water_param_int[3] = pData[3];
                water_param_data.water_param_int[2] = pData[4];
                water_param_data.water_param_int[1] = pData[5];
                water_param_data.water_param_int[0] = pData[6];
                if (switch_cmd_RS485_CNT == 1)
                {
                    temp_data = water_param_data.water_param_float * 100;//水位，k=100，b=0，单位：米
                    water_param_buf[0] = temp_data & 0x00ff;
                    water_param_buf[1] = (temp_data & 0xff00) >> 8;
                }
                else if (switch_cmd_RS485_CNT == 0)
                {
                    temp_data = water_param_data.water_param_float * 10; //瞬时流量，k=10，b=0，单位：M3/H
                    water_param_buf[2] = temp_data & 0x00ff;
                    water_param_buf[3] = (temp_data & 0xff00) >> 8;
                }
                break;
            case 2://pH，k=100，b=0
                water_param_buf[4] = pData[4];
                water_param_buf[5] = pData[3];
                break;
            case 4://电导率，k=10，b=0
                temp_data32 = ((pData[3] << 24) | (pData[4] << 16) | (pData[5] << 8) | pData[6]) / 10;
                water_param_buf[6] = temp_data32 & 0x000000ff;
                water_param_buf[7] = (temp_data32 & 0x0000ff00) >> 8;
                break;
            case 8://浊度，k=1，b=0
                water_param_buf[8] = pData[4];
                water_param_buf[9] = pData[3];
                break;
            case 10:
                water_param_data.water_param_int[3] = pData[5];
                water_param_data.water_param_int[2] = pData[6];
                water_param_data.water_param_int[1] = pData[3];
                water_param_data.water_param_int[0] = pData[4];
                if (switch_cmd_RS485_CNT == 1)   //DO，k=100，b=0
                {
                    temp_data = water_param_data.water_param_float * 100;//DO，k=100，b=0，单位：mg/L
                    water_param_buf[10] = temp_data & 0x00ff;
                    water_param_buf[11] = (temp_data & 0xff00) >> 8;
                }
                else if (switch_cmd_RS485_CNT == 0)
                {
                    temp_data = water_param_data.water_param_float * 10; //水位，k=10，b=0，单位：℃
                    water_param_buf[12] = temp_data & 0x00ff;
                    water_param_buf[13] = (temp_data & 0xff00) >> 8;
                }
                break;
            default:
                break;
            }
            return;
        }
    }
}

static void RxReport1_csb_yw(u8 len, u8 *pData)   //超声波串口1液位检测
{
    if (len != 2)return;
    temp_level = (u16)pData[0] << 8 | (u16)pData[1];
    TD_number = 2;
    if (First_adc_average[TD_number] == 0)
    {
        temp_level = First_Getaverage(TD_number, Maxlvbo_number, temp_level);
        First_adc_average[TD_number] = 1;
    }
    TDlvbo_number[TD_number] = factory_gateway_set[12 + 5 * 3 + 2];//取出触摸屏设定的差压滤波次数低字节
    temp_level = TD_Getaverage(TD_number, TDlvbo_number[TD_number], temp_level, tdcycle_i[TD_number]);
    tdcycle_i[TD_number]++;
    if (tdcycle_i[TD_number] >= TDlvbo_number[TD_number])
    {
        tdcycle_i[TD_number] = 0;
    }
    temp_level_up = factory_gateway_set[28] * 10;//加压车超声波检测液位，参数8的变量个数为净高度，单位：cm；这里转换为mm
    if (temp_level > temp_level_up)
    {
        temp_level = temp_level_up;
    }
    temp_levelS = temp_level_up - temp_level;
    wgcollector_data_buff[14] = temp_levelS & 0x00ff;
    wgcollector_data_buff[15] = (temp_levelS & 0xff00) >> 8;
}

static void RxReport1_YANHUA_touch_screen(u8 len, u8 *pData)   //研华触摸屏返回网关采集的8个检测参数
{
    if (GetCRC16(pData, len) == 0)
    {
        if (pData[0] == 0x01 && pData[1] == 0x03)  					//收到传感器返回的数据
        {
            u8 temp_data[16], bytelen1;
            u16 CRCReport1;
            bytelen1 = ((pData[4] << 8) | pData[5]) * 2;
            if (bytelen1 <= 16)
            {
                temp_data[0] = Collectors[64][1];
                temp_data[1] = Collectors[64][0];
                temp_data[2] = Collectors[64][3];
                temp_data[3] = Collectors[64][2];
                temp_data[4] = Collectors[64][5];
                temp_data[5] = Collectors[64][4];
                temp_data[6] = Collectors[64][7];
                temp_data[7] = Collectors[64][6];
                temp_data[8] = Collectors[64][9];
                temp_data[9] = Collectors[64][8];
                temp_data[10] = Collectors[64][11];
                temp_data[11] = Collectors[64][10];
                temp_data[12] = Collectors[64][13];
                temp_data[13] = Collectors[64][12];
                temp_data[14] = Collectors[64][15];
                temp_data[15] = Collectors[64][14];

                USART1SendTCB[0] = 0x01;	                //站号
                USART1SendTCB[1] = 0x03;	                        //功能码
                USART1SendTCB[2] = bytelen1;	                    //字节数
                memcpy(USART1SendTCB + 3, temp_data, bytelen1);
                CRCReport1 = GetCRC16(USART1SendTCB, 3 + bytelen1);
                USART1SendTCB[3 + bytelen1] = CRCReport1 & 0x00FF;      //CRC低位
                USART1SendTCB[4 + bytelen1] = (CRCReport1 & 0xFF00) >> 8; //CRC高位
                WriteDataToDMA_BufferTX1(bytelen1 + 5);
            }
        }
    }
}

void SF_Para_Trans()   //将收到的0x51水肥PID参数下发到子站地址为SF_SlaveID_0控制器上
{


    switch(SF_Trans_flg)
    {
    case 0:
        while (fertigation51.set_float[SF_Qeury_index] == 0)
        {

            SF_Qeury_index++;
            if (SF_Qeury_index >= 28)
            {
                SF_Qeury_index = 0;
                return;
            }

        }

        if (SF_cmd_num[SF_Qeury_index][0] != 0)
        {
            bytelen3 = WriteMultipleRegister(SF_SlaveID_0, SF_Qeury_index + 40, 2, fertigation51.set_int[SF_Qeury_index], ReportData3);
            memcpy(USART3SendTCB, ReportData3, bytelen3);
            WriteDataToDMA_BufferTX3(bytelen3);
            SF_cmd_num[SF_Qeury_index][0]--;
            SF_Trans_flg=0;
            Start_timerEx(SF_Para_Trans_EVT,200);
            return;
        }
        SF_Trans_flg=1;
        Start_timerEx(SF_Para_Trans_EVT,400);
        break;

    case 1:
        if (SF_cmd_num[SF_Qeury_index][1] != 0)
        {
            bytelen3 = WriteMultipleRegister(SF_SlaveID_1, SF_Qeury_index + 40, 2, fertigation51.set_int[SF_Qeury_index], ReportData3);
            memcpy(USART3SendTCB, ReportData3, bytelen3);
            WriteDataToDMA_BufferTX3(bytelen3);
            SF_cmd_num[SF_Qeury_index][1]--;
            SF_Trans_flg=1;
            Start_timerEx(SF_Para_Trans_EVT,200);
            return;
        }

        if(SF_cmd_num[SF_Qeury_index][1] == 0 && SF_cmd_num[SF_Qeury_index][0] == 0)
        {
            SF_Qeury_index++;
            if (SF_Qeury_index >= 28)
            {
                SF_Trans_flg =0;
                SF_Qeury_index = 0;
                return;
            }
        }
        SF_Trans_flg =0;
        Start_timerEx(SF_Para_Trans_EVT,400);
        break;
    default:
        break;
    }
}

void SF_Flow_Measu()   //三路配肥流量监测
{
    
    if(factory_gateway_set[12]==33 ||factory_gateway_set[15]==33 ||factory_gateway_set[24]==33)
    {
        SF_TD_param_num = 0;
        if(factory_gateway_set[12]==33)   //水肥涡轮流量检测计算  //第一路配肥流量下发子站 pc0 H0900
        {
            u8 i;
            SF_collector_temp=0;
            for(i=freq_I+1; i<=SF_Lvbo641; i++)
            {
                SF_collector_temp=SF_collector_temp+TIM2_FrequencyPC0[i];
            }
            for(i=0; i<freq_I; i++)
            {
                SF_collector_temp=SF_collector_temp+TIM2_FrequencyPC0[i];   //collector_temp为6s的脉冲数；1L脉冲数为450个
            }
            SF_collector_temp=80*SF_collector_temp/SF_Lvbo641;//保留1位小数；所以需要乘以10；（collector_temp*3600*10/450/lvbo641）；采样滤波12s
            SF_wgcollector_data_buff[SF_TD_param_num]   =  SF_collector_temp & 0x00ff;           //施肥流量字节低
            wgcollector_data_buff[0] = SF_wgcollector_data_buff[SF_TD_param_num];
            SF_TD_param_num++;
            SF_wgcollector_data_buff[SF_TD_param_num]   =  (SF_collector_temp & 0xff00)>>8;     //施肥流量字节高；
            wgcollector_data_buff[1] = SF_wgcollector_data_buff[SF_TD_param_num];
            SF_TD_param_num++;
        }
        else
        {
            SF_TD_param_num++;
            SF_TD_param_num++;
        }

        if(factory_gateway_set[15]==33)   //水肥涡轮流量检测计算 第二路配肥流量下发子站 pa1 H0904
        {
            u8 i;
            SF_collector_temp=0;
            for(i=freq_I+1; i<=SF_Lvbo641; i++)
            {
                SF_collector_temp=SF_collector_temp+TIM2_FrequencyPA1[i];
            }
            for(i=0; i<freq_I; i++)
            {
                SF_collector_temp=SF_collector_temp+TIM2_FrequencyPA1[i];   //collector_temp为6s的脉冲数；1L脉冲数为450个
            }
            SF_collector_temp=80*SF_collector_temp/SF_Lvbo641;//保留1位小数；所以需要乘以10；（collector_temp*3600*10/450/12）；采样滤波12s
            SF_wgcollector_data_buff[SF_TD_param_num]   =  SF_collector_temp & 0x00ff;           //施肥流量字节低
            wgcollector_data_buff[2] = SF_wgcollector_data_buff[SF_TD_param_num];
            SF_TD_param_num++;
            SF_wgcollector_data_buff[SF_TD_param_num]   =  (SF_collector_temp & 0xff00)>>8;     //施肥流量字节高；
            wgcollector_data_buff[3] = SF_wgcollector_data_buff[SF_TD_param_num];
            SF_TD_param_num++;

        }
        else
        {
            SF_TD_param_num++;
            SF_TD_param_num++;
        }

        if (factory_gateway_set[21] == 3 || factory_gateway_set[21] == 8)
        {
            SF_collector_temp=0;
            if (factory_gateway_set[21] == 8)
            {
                temp_adc = Get_Adclvbo(1, 0, 1305);
            }
            else
            {
                temp_adc = Get_Adclvbo(1, 262, 1305);
            }
            if (First_adc_average[1] == 0)
            {
                temp_adc = First_Getaverage(1, Maxlvbo_number, temp_adc);
                First_adc_average[1] = 1;
            }
            TDlvbo_number[1] = factory_gateway_set[12 + 3 * 3 + 2];//取出触摸屏设定的滤波次数低字节，滤波次数最大255次
            temp_adc = TD_Getaverage(1, TDlvbo_number[1], temp_adc, tdcycle_i[1]);
            First_adc_average[1] = 1;
            tdcycle_i[1]++;
            if (tdcycle_i[1] >= TDlvbo_number[1])
            {
                tdcycle_i[1] = 0;
            }
            SF_collector_temp = temp_adc; //土壤温度
            wgcollector_data_buff[14] = SF_collector_temp & 0x00ff;          	//土壤温度低位
            wgcollector_data_buff[15] = (SF_collector_temp & 0xff00) >> 8;    	//土壤温度高位
        }

        if(factory_gateway_set[24]==33)   //水肥涡轮流量检测计算 第二路配肥流量下发子站 pa0 H0914
        {
            u8 i;
            SF_collector_temp=0;
            for(i=freq_I+1; i<=SF_Lvbo641; i++)
            {
                SF_collector_temp=SF_collector_temp+TIM2_FrequencyPA0[i];
            }
            for(i=0; i<freq_I; i++)
            {
                SF_collector_temp=SF_collector_temp+TIM2_FrequencyPA0[i];   //collector_temp为6s的脉冲数；1L脉冲数为450个
            }
            SF_collector_temp=80*SF_collector_temp/SF_Lvbo641;//保留1位小数；所以需要乘以10；（collector_temp*3600*10/450/12）；采样滤波12s
            SF_wgcollector_data_buff[SF_TD_param_num]   =  SF_collector_temp & 0x00ff;           //施肥流量字节低
            wgcollector_data_buff[10] = SF_wgcollector_data_buff[SF_TD_param_num];
            SF_TD_param_num++;
            SF_wgcollector_data_buff[SF_TD_param_num]   =  (SF_collector_temp & 0xff00)>>8;     //施肥流量字节高；
            wgcollector_data_buff[11] = SF_wgcollector_data_buff[SF_TD_param_num];
            SF_TD_param_num++;

        }
        else
        {
            SF_TD_param_num++;
            SF_TD_param_num++;
        }
        /*wgcollector_data_buff[16]
        				[0][1] 采集参数整型641   第一路流量
        				[2][3] 采集参数整型642   第二路流量
        				[4][5] 采集参数整型643
        				[6][7] 采集参数整型644
        				[8][9] 采集参数整型645
        				[10][11] 采集参数整型646   第三路流量
        				[12][13] 采集参数整型647		总管流量   串口5函数
        				[14][15] 采集参数整型648   总管压力 4-20mA信号 data1插头
        			*/
        memcpy(Collectors[64], wgcollector_data_buff, 16);
        SF_wgcollector_data_buff[8]=(u16)((fertigation52.prarm_float[3] * 100)) & 0x00FF;    //第一路施肥浓度放大100倍，低字节
        SF_wgcollector_data_buff[9]=(u16)((fertigation52.prarm_float[3] * 100))>>8;         //第一路施肥浓度放大100倍，高字节
        SF_wgcollector_data_buff[10]=(u16)((fertigation52.prarm_float[4] * 100)) & 0x00FF;  //第二路施肥浓度放大100倍，低字节
        SF_wgcollector_data_buff[11]=(u16)((fertigation52.prarm_float[4] * 100))>>8;         //第二路施肥浓度放大100倍，高字节
        SF_wgcollector_data_buff[12]=(u16)((fertigation52.prarm_float[5] * 100)) & 0x00FF;  //第三路施肥浓度放大100倍，低字节
        SF_wgcollector_data_buff[13]=(u16)((fertigation52.prarm_float[5] * 100))>>8;        //第三路施肥浓度放大100倍，高字节
        SF_wgcollector_data_buff[14]=param_fertigation52[36] | (param_fertigation52[36]<<8);        //正在施肥标志 =1则表明开始施肥 =0则表明 没有施肥
        /*SF_wgcollector_data_buff[16]
        				[0][1]   第一路流量
        				[2][3]   第二路流量
        				[4][5]   第三路流量
        				[6][7]   总管流量
        				[8][9]   第一路设定浓度
        				[10][11] 第二路设定浓度
        				[12][13] 第三路设定浓度
        				[14]      正在施肥标志
        				[15]
        			*/

        Start_timerEx(SF_Flow_Trans_EVT,50);
    }
}
void SF_Flow_Trans()   //配肥三路直管瞬时流量下发，总管流量电磁流量计485,寄存器地址固定0x64
{
    switch(SF_Flow_Trans_Flg)
    {
    case 0:
        Start_timerEx(SF_Flow_Trans_EVT,200);
        bytelen3 = WriteMultipleRegister(SF_SlaveID_0, 100, 8, SF_wgcollector_data_buff, ReportData3); //寄存器100用于存储三路流量数据
        memcpy(SF_USART3SendTCB, ReportData3, bytelen3);
        SF_WriteDataToDMA_BufferTX3(bytelen3);
        SF_Flow_Trans_Flg++;

        break;
    case 1:
        bytelen3 = WriteMultipleRegister(SF_SlaveID_1, 100, 8, SF_wgcollector_data_buff, ReportData3);
        memcpy(SF_USART3SendTCB, ReportData3, bytelen3);
        SF_WriteDataToDMA_BufferTX3(bytelen3);
        SF_Flow_Trans_Flg = 0;
        break;
    default:
        break;
    }

}










/*中航字符卡函数  闪烁屏*/


#if 0

void LEDShowAirTemp(vs16 temperature)
{
    u8 message_ID1[36] = {0x78,0x34, 0x01, 0x00,0x29,0x12, 0xF2, 0x00,0x00,0x00, 0x00,0x00,0x00,\
                          0x10,0x00,\
                          0x01,0x00,0x01, 0x02, 0x00,0x01,0x08,0x00,\
                          0xbf,0xd5,0xc6,0xf8,0xce,0xc2,0xb6,0xc8,0x49,0x2b,0xa5
                         }; //固定在ID1区域显示“空气温度” ,长度固定34
    WriteDataToBuffer(5, (u8 *)message_ID1, 0, 34);
    LEDShowFlg = 0;
    LEDDataTemp = temperature;
    Start_timerEx(LED_SHOW_EVT,LEDDelayTime);
//	u8 message_ID2[44] ={0x78 ,0x34, 0x01, 0x00,0x29 ,0x12, 0xF2, 0x00 ,0x00 ,0x00, 0x00 ,0x00 ,0x00 ,\
//		0x14,0x00,\
//		0x02 ,0x00 ,0x01, 0x02, 0x00 ,0x01 ,0x0c ,0x00 ,\
//		0x00,0x20,0x00,0x32,0x00,0x33,0x00,0x2E,0x00,0x35,0xa1,0xe6,\
//		0x00,0x00,0xa5};//校验码 停止位 2区发送实际数据 长度固定38
//		//显示实际温度字符串 temperature是放大10倍的整型数
//		message_ID2[26] = temperature/100+48;     //十位
//		message_ID2[28] = (temperature/10)%10+48; //个位
//		message_ID2[32] = temperature % 10+48; //小数点十分位
//		u16 crc = getLEDCRC(message_ID2,35);
//		message_ID2[35] = crc & 0x00FF;
//		message_ID2[36] = crc >> 8 ;
//		WriteDataToBuffer(5, (u8 *)message_ID2, 0, 38);
}
void LEDShowAirHumi(vs16 humidity)
{
    u8 message_ID1[36] = {0x78,0x34, 0x01, 0x00,0x29,0x12, 0xF2, 0x00,0x00,0x00, 0x00,0x00,0x00,\
                          0x10,0x00,\
                          0x01,0x00,0x01, 0x02, 0x00,0x01,0x08,0x00,\
                          0xbf, 0xd5, 0xc6, 0xf8, 0xca, 0xaa, 0xb6, 0xc8,0xc9,0xc7,0xa5
                         }; //固定在ID1区域显示“空气湿度” ,长度固定34
    WriteDataToBuffer(5, (u8 *)message_ID1, 0, 34);
    LEDShowFlg = 1;
    LEDDataTemp = humidity;
    Start_timerEx(LED_SHOW_EVT,LEDDelayTime);
//	u8 message_ID2[36] ={0x78 ,0x34, 0x01, 0x00,0x29 ,0x12, 0xF2, 0x00 ,0x00 ,0x00, 0x00 ,0x00 ,0x00 ,\
//		0x12,0x00,\
//		0x02 ,0x00 ,0x01, 0x02, 0x00 ,0x01 ,0x0a ,0x00 ,\
//		0x00,0x20,0x00,0x32,0x00,0x33,0x00,0x35,0xa3,0xa5,\
//		0x00,0x00,0xa5};//校验码 停止位 2区发送实际数据 长度固定36
//		//显示实际湿度 0-100
//		message_ID2[26] = 32;     //百位为空格
//		message_ID2[28] = humidity/10+48; //十位
//		message_ID2[30] = humidity % 10+48; //个位
//		u16 crc = getLEDCRC(message_ID2,33);
//		message_ID2[33] = crc & 0x00FF;
//		message_ID2[34] = crc >> 8 ;
//		WriteDataToBuffer(5, (u8 *)message_ID2, 0, 36);
}
void LEDShowSoilTemp(vs16 temperature)
{
    u8 message_ID1[36] = {0x78,0x34, 0x01, 0x00,0x29,0x12, 0xF2, 0x00,0x00,0x00, 0x00,0x00,0x00,\
                          0x10,0x00,\
                          0x01,0x00,0x01, 0x02, 0x00,0x01,0x08,0x00,\
                          0xcd, 0xc1, 0xc8, 0xc0, 0xce, 0xc2, 0xb6, 0xc8, 0x3b,0x3c,0xa5
                         }; //固定在ID1区域显示“土壤温度” ,长度固定34
    WriteDataToBuffer(5, (u8 *)message_ID1, 0, 34);
    LEDShowFlg = 2;
    LEDDataTemp = temperature;
    Start_timerEx(LED_SHOW_EVT,LEDDelayTime);
//	u8 message_ID2[44] ={0x78 ,0x34, 0x01, 0x00,0x29 ,0x12, 0xF2, 0x00 ,0x00 ,0x00, 0x00 ,0x00 ,0x00 ,\
//		0x14,0x00,\
//		0x02 ,0x00 ,0x01, 0x02, 0x00 ,0x01 ,0x0c ,0x00 ,\
//		0x00,0x20,0x00,0x32,0x00,0x33,0x00,0x2E,0x00,0x35,0xa1,0xe6,\
//		0x00,0x00,0xa5};//校验码 停止位 2区发送实际数据 长度固定38
//		//显示实际温度字符串 temperature是放大10倍的整型数
//		message_ID2[26] = temperature/100+48;     //十位
//		message_ID2[28] = (temperature/10)%10+48; //个位
//		message_ID2[32] = temperature % 10+48; //小数点十分位
//		u16 crc = getLEDCRC(message_ID2,35);
//		message_ID2[35] = crc & 0x00FF;
//		message_ID2[36] = crc >> 8 ;
//  	WriteDataToBuffer(5, (u8 *)message_ID2, 0, 38);
}
void LEDShowSoilHumi(vs16 humidity)
{
    u8 message_ID1[36] = {0x78,0x34, 0x01, 0x00,0x29,0x12, 0xF2, 0x00,0x00,0x00, 0x00,0x00,0x00,\
                          0x10,0x00,\
                          0x01,0x00,0x01, 0x02, 0x00,0x01,0x08,0x00,\
                          0xcd, 0xc1, 0xc8, 0xc0, 0xca, 0xaa, 0xb6, 0xc8,0xbb,0xd0,0xa5
                         }; //固定在ID1区域显示“土壤湿度” ,长度固定34

    WriteDataToBuffer(5, (u8 *)message_ID1, 0, 34);
    LEDShowFlg = 3;
    LEDDataTemp = humidity;
    Start_timerEx(LED_SHOW_EVT,LEDDelayTime);
//	u8 message_ID2[36] ={0x78 ,0x34, 0x01, 0x00,0x29 ,0x12, 0xF2, 0x00 ,0x00 ,0x00, 0x00 ,0x00 ,0x00 ,\
//		0x12,0x00,\
//		0x02 ,0x00 ,0x01, 0x02, 0x00 ,0x01 ,0x0a ,0x00 ,\
//		0x00,0x20,0x00,0x32,0x00,0x33,0x00,0x35,0xa3,0xa5,\
//		0x00,0x00,0xa5};//校验码 停止位 2区发送实际数据 长度固定36
//		//显示实际温度字符串 temperature是放大10倍的整型数
//		message_ID2[26] = 32;     //百位为空格
//		message_ID2[28] = humidity/10+48; //湿度十位
//		message_ID2[30] = humidity % 10+48; //湿度个位
//		u16 crc = getLEDCRC(message_ID2,33);
//		message_ID2[33] = crc & 0x00FF;
//		message_ID2[34] = crc >> 8 ;
//		WriteDataToBuffer(5, (u8 *)message_ID2, 0, 36);
}
void LEDShowIlluminance(vs16 illuminance)
{
    u8 message_ID1[36] = {0x78,0x34, 0x01, 0x00,0x29,0x12, 0xF2, 0x00,0x00,0x00, 0x00,0x00,0x00,\
                          0x10,0x00,\
                          0x01,0x00,0x01, 0x02, 0x00,0x01,0x08,0x00,\
                          0x00, 0x20,0xb9, 0xe2, 0xd5, 0xd5, 0xb6, 0xc8,0x43,0xfd,0xa5
                         }; //固定在ID1区域显示“光照度” ,长度固定34

    WriteDataToBuffer(5, (u8 *)message_ID1, 0, 34);
    LEDShowFlg = 4;
    LEDDataTemp = illuminance;
    Start_timerEx(LED_SHOW_EVT,LEDDelayTime);
}
void LEDShow()
{


    switch(LEDSwitchFlg)
    {
    case 0:
        LEDShowAirTemp(8);
        break;           //空气温度 三位有效数字 有符号
    case 1:
        LEDShowAirHumi(85);
        break;            //空气湿度 三位有效数字 无符号
    case 2:
        LEDShowSoilTemp(-35);
        break;        //土壤温度 三位有效数字 有符号 -35会显示-3.5℃
    case 3:
        LEDShowSoilHumi(25);
        break;          //土壤湿度 三位有效数字 无符号 25显示 25%
    case 4:
        LEDShowIlluminance(25123);
        break;       //光照度 五位有效数字 无符号  20012 显示200.12 KL
    default:
        break;
    }
    LEDSwitchFlg++;
    if(LEDSwitchFlg>4)
    {
        LEDSwitchFlg = 0;
    }

}
void LEDDelay(u8 flg,vs16 data)
{
    if(flg == 0)   //实际空气温度 有符号 三位有效数字
    {
        u8 message_ID2[40] = {0x78,0x34, 0x01, 0x00,0x29,0x12, 0xF2, 0x00,0x00,0x00, 0x00,0x00,0x00,0x14,0x00,0x02,0x00,0x01, 0x02, 0x00,0x01,0x0c,0x00,\
                              0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x2E,0x00,0x20,0xa1,0xe6,\
                              0x00,0x00,0xa5
                             };//校验码 停止位 2区发送实际数据 数据域12字节 长度固定38
        //显示实际空气温度字符串 temperature是放大10倍的整型数
        if(data < 0)   //"-" gb2312 a3ad
        {
            message_ID2[23] = 0xa3;
            message_ID2[24] = 0xad;   //负数增加 - 号
        }
        data = abs(data);
        message_ID2[26] = data/100+48;     //十位
        if(data/100 == 0)
            message_ID2[26] =0x20;
        message_ID2[28] = (data/10)%10+48; //个位
        message_ID2[32] = data % 10+48; //小数点十分位
        u16 crc = getLEDCRC(message_ID2,35);
        message_ID2[35] = crc & 0x00FF;
        message_ID2[36] = crc >> 8 ;
        WriteDataToBuffer(5, (u8 *)message_ID2, 0, 38);
    }
    else if(flg == 1)     //空气湿度 最大100% 三位有效数字
    {
        u8 message_ID2[40] = {0x78,0x34, 0x01, 0x00,0x29,0x12, 0xF2, 0x00,0x00,0x00, 0x00,0x00,0x00,\
                              0x14,0x00,\
                              0x02,0x00,0x01, 0x02, 0x00,0x01,0x0c,0x00,\
                              0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0xa3,0xa5,\
                              0x00,0x00,0xa5
                             };//校验码 停止位 2区发送实际数据 长度固定38
        //显示实际空气湿度 0-100
        message_ID2[28] = data/100+48;     //湿度百位
        if(data/100 == 0)
            message_ID2[28] =0x20;                 //为0 则空格
        message_ID2[30] = (data/10)%10+48; //十位
        message_ID2[32] = data % 10+48; //个位
        u16 crc = getLEDCRC(message_ID2,35);
        message_ID2[35] = crc & 0x00FF;
        message_ID2[36] = crc >> 8 ;
        WriteDataToBuffer(5, (u8 *)message_ID2, 0, 38);

    }
    else if(flg == 2)     //土壤温度 三位有效数字 有符号
    {
        u8 message_ID2[40] = {0x78,0x34, 0x01, 0x00,0x29,0x12, 0xF2, 0x00,0x00,0x00, 0x00,0x00,0x00,\
                              0x14,0x00,\
                              0x02,0x00,0x01, 0x02, 0x00,0x01,0x0c,0x00,\
                              0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x2E,0x00,0x20,0xa1,0xe6,\
                              0x00,0x00,0xa5
                             };//校验码 停止位 2区发送实际数据 长度固定38
        //显示实际土壤温度字符串 temperature是放大10倍的有符号整型数
        if(data < 0)   //"-" gb2312 a3ad
        {
            message_ID2[23] = 0xa3;
            message_ID2[24] = 0xad;   //负数增加 - 号
        }
        data = abs(data);
        message_ID2[26] = data/100+48;     //十位
        if(data/100 == 0)
            message_ID2[26] =0x20;
        message_ID2[28] = (data/10)%10+48; //个位
        message_ID2[32] = data % 10+48; //小数点十分位
        u16 crc = getLEDCRC(message_ID2,35);
        message_ID2[35] = crc & 0x00FF;
        message_ID2[36] = crc >> 8 ;
        WriteDataToBuffer(5, (u8 *)message_ID2, 0, 38);
    }
    else if(flg == 3)     //土壤湿度 三位有效数字
    {
        u8 message_ID2[40] = {0x78,0x34, 0x01, 0x00,0x29,0x12, 0xF2, 0x00,0x00,0x00, 0x00,0x00,0x00,\
                              0x14,0x00,\
                              0x02,0x00,0x01, 0x02, 0x00,0x01,0x0c,0x00,\
                              0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0xa3,0xa5,\
                              0x00,0x00,0xa5
                             };//校验码 停止位 2区发送实际数据 长度固定38
        //显示实际土壤湿度字符串 temperature是放大10倍的整型数
        message_ID2[28] = data/100+48;
        if(data/100 == 0)
            message_ID2[28] =0x20;
        message_ID2[30] = (data/10)%10+48; //湿度十位
        message_ID2[32] = data % 10+48; //湿度个位
        u16 crc = getLEDCRC(message_ID2,35);
        message_ID2[35] = crc & 0x00FF;
        message_ID2[36] = crc >> 8 ;
        WriteDataToBuffer(5, (u8 *)message_ID2, 0, 38);
    }
    else if(flg == 4)     //光照度 5位有效数字 单位klx
    {
        u8 message_ID2[44] = {0x78,0x34, 0x01, 0x00,0x29,0x12, 0xF2, 0x00,0x00,0x00, 0x00,0x00,0x00,\
                              0x18,0x00,\
                              0x02,0x00,0x01, 0x02, 0x00,0x01,0x10,0x00,\
                              0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x2e,0x00,0x20,0x00,0x20,0x00,0x4b,0x00,0x4c,\
                              0x00,0x00,0xa5
                             };//校验码 停止位 2区发送实际数据 长度固定42
        //显示实际光照度字符串 放大100倍
        message_ID2[24] = data/10000+48;  //百位  为0则不显示
        if(data/10000 == 0)
            message_ID2[24] =0x20;
        message_ID2[26] = (data/1000)%10+48; //十位 为0则不显示
        if(message_ID2[26] == 0x30)
        {
            message_ID2[26] = 0x20;
        }
        message_ID2[28] = (data/100) % 10+48; //个位 为0显示0

        message_ID2[32] = (data/10) % 10+48; //十分位
        message_ID2[34] = data % 10+48;        //百分位
        u16 crc = getLEDCRC(message_ID2,39);
        message_ID2[39] = crc & 0x00FF;
        message_ID2[40] = crc >> 8 ;
        WriteDataToBuffer(5, (u8 *)message_ID2, 0, 42);
    }


}
u32 getLEDCRC(u8 *message,u16 length)
{
    u32 CRCFull = 0xFFFF;
    u8 CRCLSB;
    int i = 0;
    int j = 0;
    u8 *mess = message;
    for (i = 0; i < length; i++)
    {
        CRCFull = (u16)(CRCFull ^ mess[i]);
        for (j = 0; j < 8; j++)
        {
            CRCLSB = (u8)(CRCFull & 0x0001);
            CRCFull = (u16)((CRCFull>>1)&0x7FFF);
            if(CRCLSB == 1)
                CRCFull = (u16)(CRCFull ^ 0xA001);
        }
    }
    return CRCFull;
}
#endif


/**************************************中航字符卡 滚动屏*******************************************/
u32 getLEDCRC(u8 *message,u16 length)
{
    u32 CRCFull = 0xFFFF;
    u8 CRCLSB;
    int i = 0;
    int j = 0;
    u8 *mess = message;
    for (i = 0; i < length; i++)
    {
        CRCFull = (u16)(CRCFull ^ mess[i]);
        for (j = 0; j < 8; j++)
        {
            CRCLSB = (u8)(CRCFull & 0x0001);
            CRCFull = (u16)((CRCFull>>1)&0x7FFF);
            if(CRCLSB == 1)
                CRCFull = (u16)(CRCFull ^ 0xA001);
        }
    }
    return CRCFull;
}

vs8 ZHLEDShowAirTemp(vs16 temperature,u8 *message)   //温度为有符号数 -20℃,传进来是温度放大10倍
{
    vs8 len = -1;
    if(temperature < 0)   //"-" gb2312 a3ad
    {
        message[++len] = 0xa3;
        message[++len] = 0xad;   //负数增加 - 号
        temperature =0 - temperature;
    }
    //printf("len = %d\n",len );


    //printf("temp = %d\n",temperature );

    if(temperature/100 != 0)
    {
        message[len+=2] = temperature/100+48;     //十位

    }

    message[len+=2] = (temperature/10)%10+48; //个位

    message[++len] = 0x00;
    message[++len] = 0x2E;          //小数点

    message[len+=2] = temperature % 10+48; //小数点十分位

    message[++len] = 0xa1;
    message[++len]=0xe6;               // %
    message[++len]=0x00;
    message[++len]=0x20;             //加两个空格
    message[++len]=0x00;
    message[++len]=0x20;             //加两个空格
    //printf("len = %d\n",len+1 );
    return len+1;
}

vs8 ZHLEDShowSoilTemp(vs16 temperature,u8 *message)   //温度为有符号数 -20℃,传进来是温度放大10倍
{

    vs8 len = -1;
    if(temperature < 0)   //"-" gb2312 a3ad
    {
        message[++len] = 0xa3;
        message[++len] = 0xad;   //负数增加 - 号
        temperature =0 - temperature;
    }
    //printf("len = %d\n",len );


    //printf("temp = %d\n",temperature );

    if(temperature/100 != 0)
    {
        message[len+=2] = temperature/100+48;     //十位

    }

    message[len+=2] = (temperature/10)%10+48; //个位

    message[++len] = 0x00;
    message[++len] = 0x2E;          //小数点

    message[len+=2] = temperature % 10+48; //小数点十分位

    message[++len] = 0xa1;
    message[++len]=0xe6;               // %
    message[++len]=0x00;
    message[++len]=0x20;             //加两个空格
    message[++len]=0x00;
    message[++len]=0x20;             //加两个空格
    //printf("len = %d\n",len+1 );
    return len+1;
}

vs8 ZHLEDShowAirHumi(u16 Humidity,u8 *message)   //温度为有符号数 -20℃,传进来是温度放大10倍
{
    vs8 len = -1;

    if(Humidity/100 != 0)
        message[len += 2] = Humidity/100+48;     //湿度百位
    if(!(Humidity/10%10 == 0 && Humidity/100 == 0))
        message[len += 2] = Humidity/10%10+48;     //湿度十位

    message[len += 2] = Humidity % 10+48; //个位
    message[++len] =0xa3;
    message[++len] =0xa5;            //百分号
    message[++len] =0x00;
    message[++len] =0x20;
    message[++len] =0x00;
    message[++len] =0x20;             //空两格
    return len+1;
}

vs8 ZHLEDShowSoilHumi(u16 Humidity,u8 *message)   //
{
    vs8 len = -1;

    if(Humidity/100 != 0)
        message[len += 2] = Humidity/100+48;     //湿度百位
    if(!(Humidity/10%10 == 0 && Humidity/100 == 0))
        message[len += 2] = Humidity/10%10+48;     //湿度十位

    message[len += 2] = Humidity % 10+48; //个位
    message[++len] =0xa3;
    message[++len] =0xa5;            //百分号
    message[++len] =0x00;
    message[++len] =0x20;
    message[++len] =0x00;
    message[++len] =0x20;             //空两格
    return len+1;

}

vs8 ZHLEDShowCO2Density(u16 CO2Density,u8 *message)   //格式0-10000PPM
{
    vs16 len = -1;


    if(CO2Density/10000 != 0)  //万位
        message[len+=2] = CO2Density/10000 + 0x30;
    if(!(CO2Density/10000 == 0 && CO2Density/1000 %10 == 0 ))
        message[len+=2] = CO2Density/1000 %10 + 0x30;
    if( !(CO2Density/10000 == 0 && CO2Density/1000 %10 == 0 && CO2Density/100 % 10 == 0 ))
        message[len+=2] = CO2Density/100%10 + 0x30;
    if(!(CO2Density/10000 == 0 && CO2Density/1000 %10 == 0 && CO2Density/100 % 10 == 0 && CO2Density/10%10 == 0))
        message[len+=2] = CO2Density/10%10 + 0x30;
    message[len+=2] = CO2Density%10 + 0x30; //个位
    message[len+=2] = 0x50;
    message[len+=2] = 0x50;
    message[len+=2] = 0x4d;  //ppm

    message[len+=2] = 0x20;
    message[len+=2] = 0x20; //空两格
    return len+1;

}

vs8 ZHLEDShowIllumi(u16 Illuminance,u8 *message)   //温度为有符号数 -20℃,传进来是
{
    vs16 len = -1;

    if(Illuminance/10000 != 0)
        message[len+=2] =Illuminance/10000+48;
    if(!(Illuminance/10000 == 0 && Illuminance/1000%10 ==0))
        message[len+=2] = (Illuminance/1000)%10+48; //十位 为0则不显示


    message[len+=2] = (Illuminance/100) % 10+48; //个位 为0显示0
    message[++len] = 0x00;
    message[++len] = 0x2e;                       //小数点
    message[len+=2] = (Illuminance/10) % 10+48; //十分位
    message[len+=2] = Illuminance % 10+48;        //百分位
    message[++len] = 0x00;
    message[++len] = 0x4b;
    message[++len] = 0x00;
    message[++len] = 0x4c;              //kl

    message[++len] = 0x00;
    message[++len] = 0x20;
    message[++len] = 0x00;
    message[++len] = 0x20;
    return len+1;

}
void LedDisplay1(u8 region,vs16 airtemp,u16 airhumi,u16 illumi)   //字符分区1显示  空气温湿度+光照度
{

    u8 messageLen = 23;
    u8 messageID1[150] = {0x78,0x34, 0x01, 0x00,0x29,0x12, 0xF2, 0x00,0x00,0x00, 0x00,0x00,0x00,\
                          /*长度字段 2字节[13][14]*/	 													 0x10,0x00,\
                          /*ID 2字节*[15][16]*/	   													 0x01,0x00,\
                          /*编码 01gb2312，00 unicode*/			    							 0x01,\
                          /*显示方式，2为立即显示，0为保存数据模式*/		         0x02,\
                          /*字符串索引*/	                                     0x00,\
                          /*颜色 红*/                                     	 0x01,\
                          /*实际数据长度[21][22]*/                         0x08,0x00
                         };


    if( factory_gateway_set[28] != 1)                   //
    {
        messageID1[messageLen++]=0x00;
        messageID1[messageLen++]=region+48;					//区域编号
        messageID1[messageLen++]=0x00;
        messageID1[messageLen++]=0x23;         // #
    }

    memcpy(&messageID1[messageLen],AirTemp,8);     //写入空气温度
    messageLen += 8;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x3A;


    messageLen += ZHLEDShowAirTemp(airtemp,&messageID1[messageLen]);


    if( factory_gateway_set[28] != 1)
    {
        messageID1[messageLen++]=0x00;
        messageID1[messageLen++]=region+48;					//区域编号
        messageID1[messageLen++]=0x00;
        messageID1[messageLen++]=0x23;         // #
    }

    memcpy(&messageID1[messageLen],AirHumi,8);     //写入空气湿度
    messageLen += 8;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x3A;
    // messageID1[messageLen++]=0x00;
    // messageID1[messageLen++]=0x20;							// :+空格

    messageLen += ZHLEDShowAirHumi(airhumi,&messageID1[messageLen]);

    if( factory_gateway_set[28] != 1)
    {
        messageID1[messageLen++]=0x00;
        messageID1[messageLen++]=region+48;					//区域编号
        messageID1[messageLen++]=0x00;
        messageID1[messageLen++]=0x23;         // #
    }


    memcpy(&messageID1[messageLen],Illumi,6);     //写入光照度
    messageLen += 6;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x3A;


    messageLen += ZHLEDShowIllumi(illumi,&messageID1[messageLen]);

    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;


    messageID1[21] = (messageLen-23)&0xFF;
    messageID1[22] = (messageLen-23)>>8;  //实际数据长度
    messageID1[13] = (messageLen-15)&0xFF;
    messageID1[14] = (messageLen-15)>>8;  //数据长度

    u16 crc = getLEDCRC(messageID1,messageLen);
    // printf("crc = %x\n",crc);
    messageID1[messageLen++] = crc & 0x00FF;
    messageID1[messageLen++] = crc >> 8 ;
    messageID1[messageLen] = 0xa5;
    WriteDataToBuffer(5, (u8 *)messageID1, 0, messageLen+1);
//		for (int i = 0; i < messageLen+1; ++i)
//		{
//			printf("messageID1[%d]=%x\n",i,messageID1[i] );
//		}

}
void LedDisplay2(u8 region,vs16 soiltemp,u16 soilhumi,u16 CO2density)   //字符分区2显示  土壤温湿度+CO2浓度
{

    u8 messageLen = 23;
    u8 messageID2[150] = {0x78,0x34, 0x01, 0x00,0x29,0x12, 0xF2, 0x00,0x00,0x00, 0x00,0x00,0x00,\
                          /*长度字段 2字节[13][14]*/	 													 0x10,0x00,\
                          /*ID 2字节*[15][16]*/	   													 0x02,0x00,\
                          /*编码 01gb2312，00 unicode*/			    							 0x01,\
                          /*显示方式，2为立即显示，0为保存数据模式*/		         0x02,\
                          /*字符串索引*/	                                     0x00,\
                          /*颜色 红*/                                     	 0x01,\
                          /*实际数据长度[21][22]*/                         0x08,0x00
                         };


    if( factory_gateway_set[28] != 1)
    {
        messageID2[messageLen++]=0x00;
        messageID2[messageLen++]=region+48;					//区域编号
        messageID2[messageLen++]=0x00;
        messageID2[messageLen++]=0x23;         // #
    }

    memcpy(&messageID2[messageLen],SoilTemp,8);     //写入土壤温度
    messageLen += 8;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x3A;


    messageLen += ZHLEDShowSoilTemp(soiltemp,&messageID2[messageLen]);

    if( factory_gateway_set[28] != 1)
    {
        messageID2[messageLen++]=0x00;
        messageID2[messageLen++]=region+48;					//区域编号
        messageID2[messageLen++]=0x00;
        messageID2[messageLen++]=0x23;         // #
    }

    memcpy(&messageID2[messageLen],SoilHumi,8);     //写入土壤湿度
    messageLen += 8;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x3A;


    messageLen += ZHLEDShowSoilHumi(soilhumi,&messageID2[messageLen]);
    if(factory_gateway_set[28] >= 3)
    {
        messageID2[messageLen++]=0x00;
        messageID2[messageLen++]=region+48;					//区域编号
        messageID2[messageLen++]=0x00;
        messageID2[messageLen++]=0x23;         // #
        memcpy(&messageID2[messageLen],CO2Density,10);     //写入CO2浓度
        messageLen += 10;
        messageID2[messageLen++]=0x00;
        messageID2[messageLen++]=0x3A;

        messageLen += ZHLEDShowCO2Density(CO2density,&messageID2[messageLen]);
    }


    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;

    messageID2[21] = (messageLen-23)&0xFF;
    messageID2[22] = (messageLen-23)>>8;  //实际数据长度
    messageID2[13] = (messageLen-15)&0xFF;
    messageID2[14] = (messageLen-15)>>8;  //数据长度

    u16 crc = getLEDCRC(messageID2,messageLen);

    messageID2[messageLen++] = crc & 0x00FF;
    messageID2[messageLen++] = crc >> 8 ;
    messageID2[messageLen] = 0xa5;
    WriteDataToBuffer(5, (u8 *)messageID2, 0, messageLen+1);


}

void LedDisplayERR2(u8 region)   //字符分区2显示  土壤温湿度+CO2浓度
{

    u8 messageLen = 23;
    u8 messageID2[150] = {0x78,0x34, 0x01, 0x00,0x29,0x12, 0xF2, 0x00,0x00,0x00, 0x00,0x00,0x00,\
                          /*长度字段 2字节[13][14]*/	 													 0x10,0x00,\
                          /*ID 2字节*[15][16]*/	   													 0x02,0x00,\
                          /*编码 01gb2312，00 unicode*/			    							 0x01,\
                          /*显示方式，2为立即显示，0为保存数据模式*/		         0x02,\
                          /*字符串索引*/	                                     0x00,\
                          /*颜色 红*/                                     	 0x01,\
                          /*实际数据长度[21][22]*/                         0x08,0x00
                         };


    if( factory_gateway_set[28] != 1)
    {
        messageID2[messageLen++]=0x00;
        messageID2[messageLen++]=region+48;					//区域编号
        messageID2[messageLen++]=0x00;
        messageID2[messageLen++]=0x23;         // #
    }

    memcpy(&messageID2[messageLen],"参数设定错误",12);     //写入土壤温度
    messageLen += 12;

    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;
    messageID2[messageLen++]=0x00;
    messageID2[messageLen++]=0x20;

    messageID2[21] = (messageLen-23)&0xFF;
    messageID2[22] = (messageLen-23)>>8;  //实际数据长度
    messageID2[13] = (messageLen-15)&0xFF;
    messageID2[14] = (messageLen-15)>>8;  //数据长度

    u16 crc = getLEDCRC(messageID2,messageLen);

    messageID2[messageLen++] = crc & 0x00FF;
    messageID2[messageLen++] = crc >> 8 ;
    messageID2[messageLen] = 0xa5;
    WriteDataToBuffer(5, (u8 *)messageID2, 0, messageLen+1);


}
void LedDisplayERR1(u8 region)   //字符分区1显示  空气温湿度+光照度
{

    u8 messageLen = 23;
    u8 messageID1[150] = {0x78,0x34, 0x01, 0x00,0x29,0x12, 0xF2, 0x00,0x00,0x00, 0x00,0x00,0x00,\
                          /*长度字段 2字节[13][14]*/	 													 0x10,0x00,\
                          /*ID 2字节*[15][16]*/	   													 0x01,0x00,\
                          /*编码 01gb2312，00 unicode*/			    							 0x01,\
                          /*显示方式，2为立即显示，0为保存数据模式*/		         0x02,\
                          /*字符串索引*/	                                     0x00,\
                          /*颜色 红*/                                     	 0x01,\
                          /*实际数据长度[21][22]*/                         0x08,0x00
                         };
    if( factory_gateway_set[28] != 1)
    {
        messageID1[messageLen++]=0x00;
        messageID1[messageLen++]=region+48;					//区域编号
        messageID1[messageLen++]=0x00;
        messageID1[messageLen++]=0x23;         // #
    }

    memcpy(&messageID1[messageLen],"参数设定错误",12);
    messageLen += 12;




    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;
    messageID1[messageLen++]=0x20;
    messageID1[messageLen++]=0x00;



    messageID1[21] = (messageLen-23)&0xFF;
    messageID1[22] = (messageLen-23)>>8;  //实际数据长度
    messageID1[13] = (messageLen-15)&0xFF;
    messageID1[14] = (messageLen-15)>>8;  //数据长度

    u16 crc = getLEDCRC(messageID1,messageLen);

    messageID1[messageLen++] = crc & 0x00FF;
    messageID1[messageLen++] = crc >> 8 ;
    messageID1[messageLen] = 0xa5;
    WriteDataToBuffer(5, (u8 *)messageID1, 0, messageLen+1);


}

void LEDFunc(void)
{
    if(LEDCollectorNum>factory_gateway_set[28])      //factory_gateway_set[28]
    {
        LEDCollectorNum=1;
    }
    switch(LEDCollectorNum)
    {
    case 1:
        if(LEDSwitchFlg == 0)                          //1#采集参数显示
        {
            if(zero_rang.k_b_float[LEDCollecID1][0]!=0 && zero_rang.k_b_float[LEDCollecID1][2]!= 0 && zero_rang.k_b_float[LEDCollecID1][4]!=0)    //分母都不为0
            {
                vs16 airtemp= (vs16)(10*((u16)(Collectors[LEDCollecID1][4] |( Collectors[LEDCollecID1][5]<<8))/ zero_rang.k_b_float[LEDCollecID1][4]+zero_rang.k_b_float[LEDCollecID1][5]));  //003/k003+b003 空气温度
                u16 airhumi = (u16)((u16)(Collectors[LEDCollecID1][2] | (Collectors[LEDCollecID1][3]<<8))/ zero_rang.k_b_float[LEDCollecID1][2]+zero_rang.k_b_float[LEDCollecID1][3]);            //002/k002+b002  空气湿度
                u16 illumi = (u16)(100*((u16)(Collectors[LEDCollecID1][0] | (Collectors[LEDCollecID1][1]<<8))/ zero_rang.k_b_float[LEDCollecID1][0]+zero_rang.k_b_float[LEDCollecID1][1])); //001/k001+b001 光照度 kl 放大100倍传入
                LedDisplay1(1,airtemp,airhumi,illumi);
                LEDSwitchFlg = 1;
                Start_timerEx(LED_SHOW_EVT, LEDEDELAYTIME);
            }
            else
            {
                LedDisplayERR1(1);
                LEDSwitchFlg = 1;
                Start_timerEx(LED_SHOW_EVT, LEDEDELAYTIME);
            }

        }
        else if(LEDSwitchFlg == 1)
        {
            if(factory_gateway_set[28] >= 3 )
            {
                if(zero_rang.k_b_float[LEDCollecID1][6]!=0 && zero_rang.k_b_float[LEDCollecID1][8]!= 0 && zero_rang.k_b_float[LEDCollecID1][10]!=0)    //分母都不为0
                {
                    vs16 soiltemp= (vs16)(10*((u16)(Collectors[LEDCollecID1][8] |( Collectors[LEDCollecID1][9]<<8))/ zero_rang.k_b_float[LEDCollecID1][8]+zero_rang.k_b_float[LEDCollecID1][9]));  //005/k005+b005 土壤温度
                    u16 soilhumi = (u16)((u16)(Collectors[LEDCollecID1][6] | (Collectors[LEDCollecID1][7]<<8))/ zero_rang.k_b_float[LEDCollecID1][6]+zero_rang.k_b_float[LEDCollecID1][7]);            //004/k004+b004   土壤湿度
                    u16 CO2Density = (u16)((u16)(Collectors[LEDCollecID1][10] | (Collectors[LEDCollecID1][11]<<8))/ zero_rang.k_b_float[LEDCollecID1][10]+zero_rang.k_b_float[LEDCollecID1][11]); //001/k006+b006   CO2浓度
                    LedDisplay2(1,soiltemp,soilhumi,CO2Density);
                    LEDSwitchFlg = 0;
                    LEDCollectorNum++;
                    Start_timerEx(LED_SHOW_EVT,(factory_gateway_set[29]*1000));  //时间长度0-255s
                }
                else
                {
                    LedDisplayERR2(1);
                    LEDSwitchFlg = 0;
                    LEDCollectorNum++;
                    Start_timerEx(LED_SHOW_EVT,(factory_gateway_set[29]*1000));  //时间长度0-255s
                }
            }
            else
            {
                if(zero_rang.k_b_float[LEDCollecID1][6]!=0 && zero_rang.k_b_float[LEDCollecID1][8]!= 0)    //分母都不为0,显示两个参数
                {
                    vs16 soiltemp= (vs16)(10*((u16)(Collectors[LEDCollecID1][8] |( Collectors[LEDCollecID1][9]<<8))/ zero_rang.k_b_float[LEDCollecID1][8]+zero_rang.k_b_float[LEDCollecID1][9]));  //005/k005+b005 土壤温度
                    u16 soilhumi = (u16)((u16)(Collectors[LEDCollecID1][6] | (Collectors[LEDCollecID1][7]<<8))/ zero_rang.k_b_float[LEDCollecID1][6]+zero_rang.k_b_float[LEDCollecID1][7]);            //004/k004+b004   土壤湿度

                    LedDisplay2(1,soiltemp,soilhumi,0);
                    LEDSwitchFlg = 0;
                    LEDCollectorNum++;
                    Start_timerEx(LED_SHOW_EVT,(factory_gateway_set[29]*1000));  //时间长度0-255s
                }
                else
                {
                    LedDisplayERR2(1);
                    LEDSwitchFlg = 0;
                    LEDCollectorNum++;
                    Start_timerEx(LED_SHOW_EVT,(factory_gateway_set[29]*1000));  //时间长度0-255s
                }
            }



        }
        break;
    case 2:
        if(LEDSwitchFlg == 0)                         //2#采集参数显示
        {
            if(zero_rang.k_b_float[LEDCollecID2][0]!=0 && zero_rang.k_b_float[LEDCollecID2][2]!= 0 && zero_rang.k_b_float[LEDCollecID2][4]!=0)    //分母都不为0
            {
                vs16 airtemp= (vs16)(10*((u16)(Collectors[LEDCollecID2][4] |( Collectors[LEDCollecID2][5]<<8))/ zero_rang.k_b_float[LEDCollecID2][4]+zero_rang.k_b_float[LEDCollecID2][5]));  //013/k013+b013 空气温度
                u16 airhumi = (u16)((u16)(Collectors[LEDCollecID2][2] | (Collectors[LEDCollecID2][3]<<8))/ zero_rang.k_b_float[LEDCollecID2][2]+zero_rang.k_b_float[LEDCollecID2][3]);            //012/k012+b012  空气湿度
                u16 illumi = (u16)(100*((u16)(Collectors[LEDCollecID2][0] | (Collectors[LEDCollecID2][1]<<8))/ zero_rang.k_b_float[LEDCollecID2][0]+zero_rang.k_b_float[LEDCollecID2][1])); //011/k011+b011 光照度 kl 放大100倍传入
                LedDisplay1(2,airtemp,airhumi,illumi);
                LEDSwitchFlg = 1;
                Start_timerEx(LED_SHOW_EVT, LEDEDELAYTIME);
            }
            else
            {
                LedDisplayERR1(2);
                LEDSwitchFlg = 1;
                Start_timerEx(LED_SHOW_EVT, LEDEDELAYTIME);
            }

        }
        else if(LEDSwitchFlg == 1)
        {
            if(factory_gateway_set[28] >= 3)
            {
                if(zero_rang.k_b_float[LEDCollecID2][6]!=0 && zero_rang.k_b_float[LEDCollecID2][8]!= 0 && zero_rang.k_b_float[LEDCollecID2][10]!=0)    //分母都不为0
                {
                    vs16 soiltemp= (vs16)(10*((u16)(Collectors[LEDCollecID2][8] |( Collectors[LEDCollecID2][9]<<8))/ zero_rang.k_b_float[LEDCollecID2][8]+zero_rang.k_b_float[LEDCollecID2][9]));  //015/k015+b015 土壤温度
                    u16 soilhumi =(u16)((u16)(Collectors[LEDCollecID1][6] | (Collectors[LEDCollecID1][7]<<8))/ zero_rang.k_b_float[LEDCollecID1][6]+zero_rang.k_b_float[LEDCollecID1][7]);            //014/k004+b014   土壤湿度
                    u16 CO2Density = (u16)((u16)(Collectors[LEDCollecID2][10] | (Collectors[LEDCollecID2][11]<<8))/ zero_rang.k_b_float[LEDCollecID2][10]+zero_rang.k_b_float[LEDCollecID2][11]); //011/k016+b016   CO2浓度
                    LedDisplay2(2,soiltemp,soilhumi,CO2Density);
                    LEDSwitchFlg = 0;
                    LEDCollectorNum++;
                    Start_timerEx(LED_SHOW_EVT,(factory_gateway_set[29]*1000));  //时间长度0-65
                }
                else
                {
                    LedDisplayERR2(2);
                    LEDSwitchFlg = 0;
                    LEDCollectorNum++;
                    Start_timerEx(LED_SHOW_EVT,(factory_gateway_set[29]*1000));  //时间长度0-65
                }
            }
            else
            {
                if(zero_rang.k_b_float[LEDCollecID2][6]!=0 && zero_rang.k_b_float[LEDCollecID2][8]!= 0 )    //分母都不为0
                {
                    vs16 soiltemp= (vs16)(10*((u16)(Collectors[LEDCollecID2][8] |( Collectors[LEDCollecID2][9]<<8))/ zero_rang.k_b_float[LEDCollecID2][8]+zero_rang.k_b_float[LEDCollecID2][9]));  //015/k015+b015 土壤温度
                    u16 soilhumi =(u16)((u16)(Collectors[LEDCollecID1][6] | (Collectors[LEDCollecID1][7]<<8))/ zero_rang.k_b_float[LEDCollecID1][6]+zero_rang.k_b_float[LEDCollecID1][7]);            //014/k004+b014   土壤湿度

                    LedDisplay2(2,soiltemp,soilhumi,0);
                    LEDSwitchFlg = 0;
                    LEDCollectorNum++;
                    Start_timerEx(LED_SHOW_EVT,(factory_gateway_set[29]*1000));  //时间长度0-65
                }
                else
                {
                    LedDisplayERR2(2);
                    LEDSwitchFlg = 0;
                    LEDCollectorNum++;
                    Start_timerEx(LED_SHOW_EVT,(factory_gateway_set[29]*1000));  //时间长度0-65
                }
            }


        }
        break;
    case 3:
        if(LEDSwitchFlg == 0)                         //3#采集参数显示
        {
            if(zero_rang.k_b_float[LEDCollecID3][0]!=0 && zero_rang.k_b_float[LEDCollecID3][2]!= 0 && zero_rang.k_b_float[LEDCollecID3][4]!=0)    //分母都不为0
            {
                vs16 airtemp= (vs16)(10*((u16)(Collectors[LEDCollecID3][4] |( Collectors[LEDCollecID3][5]<<8))/ zero_rang.k_b_float[LEDCollecID3][4]+zero_rang.k_b_float[LEDCollecID3][5]));  //003/k003+b003 空气温度
                u16 airhumi = (u16)((u16)(Collectors[LEDCollecID3][2] | (Collectors[LEDCollecID3][3]<<8))/ zero_rang.k_b_float[LEDCollecID3][2]+zero_rang.k_b_float[LEDCollecID3][3]);            //002/k002+b002  空气湿度
                u16 illumi = (u16)(100*((u16)(Collectors[LEDCollecID3][0] | (Collectors[LEDCollecID3][1]<<8))/ zero_rang.k_b_float[LEDCollecID3][0]+zero_rang.k_b_float[LEDCollecID3][1])); //001/k001+b001 光照度 kl 放大100倍传入
                LedDisplay1(3,airtemp,airhumi,illumi);
                LEDSwitchFlg = 1;
                Start_timerEx(LED_SHOW_EVT, LEDEDELAYTIME);
            }
            else
            {
                LedDisplayERR1(3);
                LEDSwitchFlg = 1;
                Start_timerEx(LED_SHOW_EVT, LEDEDELAYTIME);
            }

        }
        else if(LEDSwitchFlg == 1)
        {
            if(factory_gateway_set[28] >= 3)
            {
                if(zero_rang.k_b_float[LEDCollecID3][6]!=0 && zero_rang.k_b_float[LEDCollecID3][8]!= 0 && zero_rang.k_b_float[LEDCollecID3][10]!=0)    //分母都不为0
                {
                    vs16 soiltemp= (vs16)(10*((u16)(Collectors[LEDCollecID3][8] |( Collectors[LEDCollecID3][9]<<8))/ zero_rang.k_b_float[LEDCollecID3][8]+zero_rang.k_b_float[LEDCollecID3][9]));  //015/k015+b015 土壤温度
                    u16 soilhumi =(u16)((u16)(Collectors[LEDCollecID3][6] | (Collectors[LEDCollecID3][7]<<8))/ zero_rang.k_b_float[LEDCollecID3][6]+zero_rang.k_b_float[LEDCollecID3][7]);            //014/k004+b014   土壤湿度
                    u16 CO2Density = (u16)((u16)(Collectors[LEDCollecID3][10] | (Collectors[LEDCollecID3][11]<<8))/ zero_rang.k_b_float[LEDCollecID3][10]+zero_rang.k_b_float[LEDCollecID3][11]); //011/k016+b016   CO2浓度
                    LedDisplay2(3,soiltemp,soilhumi,CO2Density);
                    LEDSwitchFlg = 0;
                    LEDCollectorNum++;
                    Start_timerEx(LED_SHOW_EVT,(factory_gateway_set[29]*1000));  //时间长度0-255s
                }
                else
                {
                    LedDisplayERR2(3);
                    LEDSwitchFlg = 0;
                    LEDCollectorNum++;
                    Start_timerEx(LED_SHOW_EVT,(factory_gateway_set[29]*1000));  //时间长度0-255s
                }
            }
            else
            {
                if(zero_rang.k_b_float[LEDCollecID3][6]!=0 && zero_rang.k_b_float[LEDCollecID3][8]!= 0 )    //分母都不为0
                {
                    vs16 soiltemp= (vs16)(10*((u16)(Collectors[LEDCollecID3][8] |( Collectors[LEDCollecID3][9]<<8))/ zero_rang.k_b_float[LEDCollecID3][8]+zero_rang.k_b_float[LEDCollecID3][9]));  //015/k015+b015 土壤温度
                    u16 soilhumi =(u16)((u16)(Collectors[LEDCollecID3][6] | (Collectors[LEDCollecID3][7]<<8))/ zero_rang.k_b_float[LEDCollecID3][6]+zero_rang.k_b_float[LEDCollecID3][7]);            //014/k004+b014   土壤湿度

                    LedDisplay2(3,soiltemp,soilhumi,0);
                    LEDSwitchFlg = 0;
                    LEDCollectorNum++;
                    Start_timerEx(LED_SHOW_EVT,(factory_gateway_set[29]*1000));  //时间长度0-255s
                }
                else
                {
                    LedDisplayERR2(3);
                    LEDSwitchFlg = 0;
                    LEDCollectorNum++;
                    Start_timerEx(LED_SHOW_EVT,(factory_gateway_set[29]*1000));  //时间长度0-255s
                }
            }



        }
        break;
    default:
        break;
    }
}