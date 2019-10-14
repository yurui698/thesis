#include "fertigation.h"

//水肥机定义
/*static union { //子站地址0x51
    float set_float[28];//水肥机管道布置及PID设定
    u8 set_int[28][4];
} fertigation51;
static u8 set_fertigation51_finish[28];
static u8 read_fertigation_finish51;
static void chge_fertisnd(u8 flg);
static union { //子站地址0x52
    float prarm_float[6];//水肥机管道布置及PID设定
    u8 param_int[6][4];
} fertigation52;

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
u8 reportfertigation52[3][16]={0};
u8 fertigation52databuf[12] = { 0 }; //用于拷52下发的浮点数，转成2字节
u8 SF_cmd_num[28][2] = { 0 }; //用于限制消息发送次数，当接收回复时将该下标处清0,二维数组表示两路控制器
u8 SF_Qeury_index=0;      //查询下发后收到的回复确认
u8 SF_Trans_flg=0;					//参数下发标志位，向两路控制器下发设定参数
u16 SF_collector_temp = 0;
u8 SF_wgcollector_data_buff[16] = {0};
u8 SF_TD_param_num = 0;
#define SF_Para_Tran_time 500

#define TXENABLE3		         GPIO_SetBits(GPIOD, GPIO_Pin_10)

u8 SF_USART3SendTCB[128]={0};
u8 SF_Flow_Trans_Flg = 0;      //流量下发标志位，向两路控制器下发配肥流量以及总管流量
u16 SF_Flow_Total[6] = {0};  //取6个瞬时总管流量的平均值下发
u8 SF_Flow_Total_I = 0;
u8 SF_Flow_Total_flg = 0;
u8 SF_Lvbo641=12;
u8 SF_Lvbo642=12;
u8 SF_Lvbo646=12;

extern union //子站地址0x51
{
      float set_float[28];//水肥机管道布置及PID设定
      u8 set_int[28][4];
}fertigation51;
 extern u8 set_fertigation51_finish[28];
 extern u8 read_fertigation_finish51;

 extern union //子站地址0x52
{
      float prarm_float[6];//水肥机管道布置及PID设定
      u8 param_int[6][4];
}fertigation52;
 extern u8 param_fertigation52[38];
 extern u8 fertigation_flg;

extern  u8 USART3SendTCB[128];
extern u8 factory_gateway_set[255];
extern u8 bytelen3;
extern  u8 ReportData3[128];
extern u8 freq_I;
extern u8  wgcollector_data_buff[16];
extern u16  TIM2_FrequencyPC0[61];
extern u16  TIM2_FrequencyPA1[61];
extern u16 TIM2_FrequencyPA0[61];
extern u16 temp_adc;
extern u8  WriteMultipleRegister(u8 Slave_ID,u16 addr,u16 num,u8 *pData,u8 *temp);
extern void WriteDataToDMA_BufferTX3(uint16_t size);
extern u16 Get_Adclvbo(u8 TD_Xnumber,u16 TD_xiaxian,u16 TD_shangxian);
extern u8 First_adc_average[3];
extern u16 First_Getaverage(u8 td_xnumber,u8 maxlvbo_xnumber,u16 temp_adc);
static const u8 Maxlvbo_number= 50;
static const u8 MaxTD_number= 3;	
extern u8 TDlvbo_number[MaxTD_number];
extern u8 tdcycle_i[MaxTD_number];
extern u16 TD_Getaverage(u8 td_xnumber,u8 tdlvbo_xnumber,u16 temp_xadc,u8 tdcycle_xi);
extern u8 Collectors[65][16];
extern u8 TxFlag3 ;
void SF_Para_Trans() { //将收到的0x51水肥PID参数下发到子站地址为SF_SlaveID_0控制器上
	
	
    switch(SF_Trans_flg) {
    case 0:
        while (fertigation51.set_float[SF_Qeury_index] == 0) {

            SF_Qeury_index++;
            if (SF_Qeury_index >= 28) {
                SF_Qeury_index = 0;
                return;
            }

        }

        if (SF_cmd_num[SF_Qeury_index][0] != 0) {
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
        if (SF_cmd_num[SF_Qeury_index][1] != 0) {
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
						if (SF_Qeury_index >= 28) {
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

  void SF_Flow_Measu() { //三路配肥流量监测
    
    if(factory_gateway_set[12]==33 ||factory_gateway_set[15]==33 ||factory_gateway_set[24]==33) {
        SF_TD_param_num = 0;
        if(factory_gateway_set[12]==33) { //水肥涡轮流量检测计算  //第一路配肥流量下发子站 pc0 H0900
            u8 i;
            SF_collector_temp=0;
            for(i=freq_I+1; i<=SF_Lvbo641; i++) {
                SF_collector_temp=SF_collector_temp+TIM2_FrequencyPC0[i];
            }
            for(i=0; i<freq_I; i++) {
                SF_collector_temp=SF_collector_temp+TIM2_FrequencyPC0[i];   //collector_temp为6s的脉冲数；1L脉冲数为450个
            }
            SF_collector_temp=80*SF_collector_temp/SF_Lvbo641;//保留1位小数；所以需要乘以10；（collector_temp*3600*10/450/lvbo641）；采样滤波12s
            SF_wgcollector_data_buff[SF_TD_param_num]   =  SF_collector_temp & 0x00ff;           //施肥流量字节低
            wgcollector_data_buff[0] = SF_wgcollector_data_buff[SF_TD_param_num];
            SF_TD_param_num++;
            SF_wgcollector_data_buff[SF_TD_param_num]   =  (SF_collector_temp & 0xff00)>>8;     //施肥流量字节高；
            wgcollector_data_buff[1] = SF_wgcollector_data_buff[SF_TD_param_num];
            SF_TD_param_num++;
        } else {
            SF_TD_param_num++;
            SF_TD_param_num++;
        }

        if(factory_gateway_set[15]==33) { //水肥涡轮流量检测计算 第二路配肥流量下发子站 pa1 H0904
            u8 i;
            SF_collector_temp=0;
            for(i=freq_I+1; i<=SF_Lvbo641; i++) {
                SF_collector_temp=SF_collector_temp+TIM2_FrequencyPA1[i];
            }
            for(i=0; i<freq_I; i++) {
                SF_collector_temp=SF_collector_temp+TIM2_FrequencyPA1[i];   //collector_temp为6s的脉冲数；1L脉冲数为450个
            }
            SF_collector_temp=80*SF_collector_temp/SF_Lvbo641;//保留1位小数；所以需要乘以10；（collector_temp*3600*10/450/12）；采样滤波12s
            SF_wgcollector_data_buff[SF_TD_param_num]   =  SF_collector_temp & 0x00ff;           //施肥流量字节低
            wgcollector_data_buff[2] = SF_wgcollector_data_buff[SF_TD_param_num];
            SF_TD_param_num++;
            SF_wgcollector_data_buff[SF_TD_param_num]   =  (SF_collector_temp & 0xff00)>>8;     //施肥流量字节高；
            wgcollector_data_buff[3] = SF_wgcollector_data_buff[SF_TD_param_num];
            SF_TD_param_num++;

        } else {
            SF_TD_param_num++;
            SF_TD_param_num++;
        }
				
			if (factory_gateway_set[21] == 3 || factory_gateway_set[21] == 8) {
            SF_collector_temp=0;
            if (factory_gateway_set[21] == 8) {
                temp_adc = Get_Adclvbo(1, 0, 1305);
            } else {
                temp_adc = Get_Adclvbo(1, 262, 1305);
            }
            if (First_adc_average[1] == 0) {
                temp_adc = First_Getaverage(1, Maxlvbo_number, temp_adc);
                First_adc_average[1] = 1;
            }
            TDlvbo_number[1] = factory_gateway_set[12 + 3 * 3 + 2];//取出触摸屏设定的滤波次数低字节，滤波次数最大255次
            temp_adc = TD_Getaverage(1, TDlvbo_number[1], temp_adc, tdcycle_i[1]);
            First_adc_average[1] = 1;
            tdcycle_i[1]++;
            if (tdcycle_i[1] >= TDlvbo_number[1]) {
                tdcycle_i[1] = 0;
            }
            SF_collector_temp = temp_adc; //土壤温度
            wgcollector_data_buff[14] = SF_collector_temp & 0x00ff;          	//土壤温度低位
            wgcollector_data_buff[15] = (SF_collector_temp & 0xff00) >> 8;    	//土壤温度高位
        }

        if(factory_gateway_set[24]==33) { //水肥涡轮流量检测计算 第二路配肥流量下发子站 pa0 H0914
            u8 i;
            SF_collector_temp=0;
            for(i=freq_I+1; i<=SF_Lvbo641; i++) {
                SF_collector_temp=SF_collector_temp+TIM2_FrequencyPA0[i];
            }
            for(i=0; i<freq_I; i++) {
                SF_collector_temp=SF_collector_temp+TIM2_FrequencyPA0[i];   //collector_temp为6s的脉冲数；1L脉冲数为450个
            }
            SF_collector_temp=80*SF_collector_temp/SF_Lvbo641;//保留1位小数；所以需要乘以10；（collector_temp*3600*10/450/12）；采样滤波12s
            SF_wgcollector_data_buff[SF_TD_param_num]   =  SF_collector_temp & 0x00ff;           //施肥流量字节低
            wgcollector_data_buff[10] = SF_wgcollector_data_buff[SF_TD_param_num];
            SF_TD_param_num++;
            SF_wgcollector_data_buff[SF_TD_param_num]   =  (SF_collector_temp & 0xff00)>>8;     //施肥流量字节高；
            wgcollector_data_buff[11] = SF_wgcollector_data_buff[SF_TD_param_num];
            SF_TD_param_num++;

        } else {
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
void SF_WriteDataToDMA_BufferTX3(uint16_t size) {
    TXENABLE3;
    TxFlag3 = 1;
    DMA1_Channel2->CMAR = (uint32_t)SF_USART3SendTCB;
    DMA1_Channel2->CNDTR = (uint16_t)size; // 设置要发送的字节数目
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);  // 开启串口DMA发送
    DMA_Cmd(DMA1_Channel2, ENABLE);        //开始DMA发送
}
void SF_Flow_Trans() { //配肥三路直管瞬时流量下发，总管流量电磁流量计485,寄存器地址固定0x64
    switch(SF_Flow_Trans_Flg) {
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


