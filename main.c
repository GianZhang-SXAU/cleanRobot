/**			                                                    
		   ____                    _____ _____  _____        XTARK@塔克创新
		  / __ \                  / ____|  __ \|  __ \  
		 | |  | |_ __   ___ _ __ | |    | |__) | |__) |
		 | |  | | '_ \ / _ \ '_ \| |    |  _  /|  ___/ 
		 | |__| | |_) |  __/ | | | |____| | \ \| |     
		  \____/| .__/ \___|_| |_|\_____|_|  \_\_|     
		    		| |                                    
		    		|_|  OpenCRP 树莓派 专用ROS机器人控制器                                   
									 
  ****************************************************************************** 
  *           
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 官网网站： www.xtark.cn
  * 淘宝店铺： https://shop246676508.taobao.com  
  * 塔克媒体： www.cnblogs.com/xtark（博客）
	* 塔克微信： 微信公众号：塔克创新（获取最新资讯）
  *      
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @版  本  V1.0
  * @日  期  2020-7-26
  * @内  容  OpenCRP控制器功能展示示例代码
  * 
	******************************************************************************
  * @说  明
  *
  * 1.示例程序引导用户使用OpenCRP控制器进行机器人编程。
	* 2.每个例程都经过测试，取消注释，编译下载即可查看效果。
  * 3.每个例程具有功能说明。
	*  
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>

#include "ax_sys.h"    //系统设置
#include "ax_delay.h"  //软件延时
#include "ax_led.h"    //LED灯控制
#include "ax_vin.h"    //输入电压检测
#include "ax_key.h"    //按键检测 
#include "ax_uart_db.h"  //调试串口
#include "ax_uart_pi.h"  //树莓派串口
#include "ax_motor.h"    //直流电机调速控制
#include "ax_encoder.h"  //编码器控制
#include "ax_servo.h"    //舵机控制
#include "ax_tim.h"      //定时器
#include "ax_mpu6050.h"  //IMU加速度陀螺仪测量
#include "ax_mpu6050_dmp.h"  //DMP功能函数

/******************************************************************************
      基础例程  清单
			
* LED闪烁，蜂鸣器，调试串口Printf输出例程
* VIN输入电压检测例程，简易电量计
* KEY按键检测检测例程，软件消抖
* MPU6050数据采集例程
* 舵机控制例程
* 直流电机PWM速度控制例程
* 电机AB正交编码器例程
* 直流电机PID调速例程
* STM32与树莓派通信

*******************************************************************************/

/******************************************************************************
例程名称：LED闪烁，调试串口Printf输出
例程说明：本例程演示LED控制和调试串口Printf功能
*******************************************************************************/
//int main(void)
//{
//	uint8_t i=0;
//	
//	//设置中断优先级分组
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    

//	//JTAG口设置
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
//	AX_JTAG_Set(SWD_ENABLE);  //打开SWD接口 可以利用主板的SWD接口调试 
//	
//	//软件延时初始化
//	AX_DELAY_Init(); 	
//	AX_LED_Init();  //LED初始化

//	//调试串口初始化
//	AX_UART_DB_Init(115200); //调试串口
//	printf("  \r\n"); //输出空格，CPUBUG
//	
//	//LED点亮0.5S
//	AX_LED_Green_On();
//	AX_Delayms(500);
//	AX_LED_Green_Off();
//	AX_Delayms(500);
//	
//	//LED点亮0.5S
//	AX_LED_Red_On();
//	AX_Delayms(500);
//	AX_LED_Red_Off();
//	AX_Delayms(500);
//	
//	AX_LED_Red_On();
//	
//	while (1)
//	{	
//    //调试串口输出信息		
//		printf("Printf输出测试：%d \r\n",i);
//		i++;
//		AX_Delayms(100);
//		
//		//LED反转
//	  AX_LED_Green_Toggle();
//	  AX_LED_Red_Toggle();
//	}
//}


///******************************************************************************
//例程名称：VIN输入电压检测
//例程说明：串口循环输出采集到的电压值
//*******************************************************************************/
//uint16_t vol;

//int main(void)
//{
////	uint16_t vol;
//	//设置中断优先级分组
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    

//	//JTAG口设置
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
//	AX_JTAG_Set(SWD_ENABLE);  //打开SWD接口 可以利用主板的SWD接口调试 
//	
//	//软件延时初始化
//	AX_DELAY_Init(); 	
//	
//	//调试串口初始化
//	AX_UART_DB_Init(115200); //调试串口
//	printf("  \r\n"); //输出空格，CPUBUG
//	
//	//VIN检测初始化
//	AX_VIN_Init();
//	
//	//提示信息
//	printf("*循环采集VIN输入口电压值\r\n");
//	
//	while (1) 
//	{	
//		//每100MS输出一次电池电压值
//		vol = AX_VIN_GetVol_X100();
//		printf("*VIN电压：%d(0.01V)\r\n",vol );	
//		AX_Delayms(100);
//	}
//}


///******************************************************************************
//例程名称：KEY按键检测检测
//例程说明：按键按下后，LED灯闪烁一次
//*******************************************************************************/
//int main(void)
//{
//	uint8_t temp;
//	
//	//设置中断优先级分组
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    

//	//JTAG口设置
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
//	AX_JTAG_Set(SWD_ENABLE);  //打开SWD接口 可以利用主板的SWD接口调试 
//	
//	//软件延时初始化
//	AX_DELAY_Init(); 	
//	AX_LED_Init();  	
//	
//	//调试串口初始化
//	AX_UART_DB_Init(115200); //调试串口
//	printf("  \r\n"); //输出空格，CPUBUG
//	
//	//初始化
//	AX_KEY_Init();
//	
//	while (1) 
//	{	
//		temp = AX_KEY_Scan();
//		
//		if(temp == 1)
//		{
//			AX_LED_Green_On();  
//			AX_Delayms(100);
//			AX_LED_Green_Off();
//		}
//		AX_Delayms(10);
//	}
//}	


///******************************************************************************
//例程名称：读取MPU6050数据并输出
//例程说明：读取MPU6050数据，通过串口Printf输出
//操作说明：串口输出三轴加速度，三轴陀螺仪数据，转动控制器，观察数据变化
//         使用我们的X-PrintfScope软件，可以观察到波形数据
//*******************************************************************************/
//int main(void)
//{
//	int16_t ax_acc[3],ax_gyro[3]; //IMU数据
//	
//	//设置中断优先级分组
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    

//	//JTAG口设置
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
//	AX_JTAG_Set(SWD_ENABLE);  //打开SWD接口 可以利用主板的SWD接口调试 
//	
//	//软件延时初始化
//	AX_DELAY_Init(); 	
//	AX_LED_Init();  //LED初始化

//	//调试串口初始化
//	AX_UART_DB_Init(115200); //调试串口
//	printf("  \r\n"); //输出空格，CPUBUG
//	
//	//MPU6050初始化  
//	AX_MPU6050_Init();    
//	AX_MPU6050_SetAccRange(AX_ACC_RANGE_2G);    //设置加速度量程
//	AX_MPU6050_SetGyroRange(AX_GYRO_RANGE_2000); //设置陀螺仪量程
//	AX_MPU6050_SetGyroSmplRate(200);            //设置陀螺仪采样率
//	AX_MPU6050_SetDLPF(AX_DLPF_ACC94_GYRO98);   //设置低通滤波器带宽
//	
//	while (1)
//	{	
//		//更新姿态、陀螺仪、加速度数据
//		AX_MPU6050_GetAccData(ax_acc);  //读取三轴加速度数据
//		AX_MPU6050_GetGyroData(ax_gyro);  //读取三轴陀螺仪数据		
//		
//		//调试串口输出信息
//		printf("@%d %d %d %d %d %d \r\n",
//		ax_acc[0],ax_acc[1],ax_acc[2],ax_gyro[0],ax_gyro[1],ax_gyro[2]);		
//		
//		AX_Delayms(50);
//	}
//}


///******************************************************************************
//例程名称：舵机控制例程
//例程说明：控制8路舵机60度、90度、120度间隔运动
//操作说明：可在8路舵机接口中任意一路插入舵机，舵机即可循环60°、90°、120°运动，
//         如果插入多路舵机，或舵机负载扭矩大，请注意电源供电能力
//				 注意OpenCRP4默认是不供电的，需要连接跳线帽，具体参考原理图
//*******************************************************************************/
//int main(void)
//{
//	//设置中断优先级分组
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
//	
//	//调试串口初始化
//	AX_UART_DB_Init(115200); //调试串口
//	printf("  \r\n"); //输出空格，CPUBUG
//	
//	//JTAG口设置
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
//	AX_JTAG_Set(SWD_ENABLE);  //打开SWD接口 可以利用主板的SWD接口调试 		
//	
//	//软件延时初始化
//	AX_DELAY_Init();
//  AX_LED_Init();
//	
//	//初始化
//	AX_SERVO_AB_Init();
//  AX_SERVO_CD_Init();
//	AX_SERVO_EF_Init();
//	AX_SERVO_GH_Init();
//	
//	//提示信息
//	printf("八路舵机控制测试\r\n");

//	while (1) 
//	{		
//		printf("*60度...... \r\n");		
//		AX_SERVO_A_SetAngle(600);
//		AX_SERVO_B_SetAngle(600);
//		AX_SERVO_C_SetAngle(600);
//		AX_SERVO_D_SetAngle(600);
//		AX_SERVO_E_SetAngle(600);
//		AX_SERVO_F_SetAngle(600);
//		AX_SERVO_G_SetAngle(600);
//		AX_SERVO_H_SetAngle(600);
//		AX_Delayms(1000);
//		
//		printf("*90度...... \r\n");
//		AX_SERVO_A_SetAngle(900);
//		AX_SERVO_B_SetAngle(900);
//		AX_SERVO_C_SetAngle(900);
//		AX_SERVO_D_SetAngle(900);
//		AX_SERVO_E_SetAngle(900);
//		AX_SERVO_F_SetAngle(900);
//		AX_SERVO_G_SetAngle(900);
//		AX_SERVO_H_SetAngle(900);
//		AX_Delayms(1000);
//		
//		printf("*120度...... \r\n");
//		AX_SERVO_A_SetAngle(1200);
//		AX_SERVO_B_SetAngle(1200);
//		AX_SERVO_C_SetAngle(1200);
//		AX_SERVO_D_SetAngle(1200);
//		AX_SERVO_E_SetAngle(1200);
//		AX_SERVO_F_SetAngle(1200);
//		AX_SERVO_G_SetAngle(1200);
//		AX_SERVO_H_SetAngle(1200);
//		AX_Delayms(1000);
//	}
//}		



///******************************************************************************
//例程名称：直流电机PWM调速控制
//例程说明：控制4路电机变速正传和变速反转交替运行
//操作说明：电机可连接至四路控制接口中的任意一路，可以看到电机做间隔的增速正转和反转
//*******************************************************************************/
//int main(void)
//{
//	uint16_t temp;
//	
//	//设置中断优先级分组
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
//	
//	//JTAG口设置
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
//	AX_JTAG_Set(SWD_ENABLE);  //打开SWD接口 可以利用主板的SWD接口调试 		
//	
//	//软件延时初始化
//	AX_DELAY_Init(); 	
//  AX_LED_Init();  
//	
//	//初始化
//	AX_MOTOR_Init(10);  //设置电机控制PWM频率为10K

//	while (1) 
//	{	
//		//控制电机转动
//		for(temp=0; temp<=2000; temp++)
//		{
//			AX_MOTOR_A_SetSpeed(temp); 
//			AX_MOTOR_B_SetSpeed(temp); 
//			AX_MOTOR_C_SetSpeed(temp); 
//			AX_MOTOR_D_SetSpeed(temp); 
//			AX_Delayms(5);
//		}
//		AX_MOTOR_A_SetSpeed(0); 
//		AX_MOTOR_B_SetSpeed(0); 
//		AX_MOTOR_C_SetSpeed(0); 
//		AX_MOTOR_D_SetSpeed(0); 	
//		AX_Delayms(1000);
//		
//		//控制电机反向转动
//		for(temp=0; temp<=2000; temp++)
//		{
//			AX_MOTOR_A_SetSpeed(-temp); 
//			AX_MOTOR_B_SetSpeed(-temp); 
//			AX_MOTOR_C_SetSpeed(-temp); 
//			AX_MOTOR_D_SetSpeed(-temp); 
//			AX_Delayms(5);
//		}
//		AX_MOTOR_A_SetSpeed(0); 
//		AX_MOTOR_B_SetSpeed(0); 
//		AX_MOTOR_C_SetSpeed(0); 
//		AX_MOTOR_D_SetSpeed(0);  
//		AX_Delayms(1000);
//	}
//}	


///******************************************************************************
//例程名称：正交AB编码器例程
//例程说明：300ms采样5路编码器数值并串口输出显示，电机50%PWM占空比运行
//操作说明：4路编码器均可用，连接编码器后，即可手动转动电机观察串口输出值变化
//         也可将电机连接到电机接口
//*******************************************************************************/
//int AB_Get_Counter,CD_Get_Counter,EF_Get_Counter,GH_Get_Counter;
//int main(void)
//{
//	//设置中断优先级分组
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
//	
//	//调试串口初始化
//	AX_UART_DB_Init(115200); //调试串口
//	printf("  \r\n"); //输出空格，CPUBUG
//	
//	//JTAG口设置
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
//	AX_JTAG_Set(SWD_ENABLE);  //打开SWD接口 可以利用主板的SWD接口调试 		
//	
//	//软件延时初始化
//	AX_DELAY_Init(); 	
//  AX_LED_Init();   
//	
//	//正交编码器初始化
//  AX_ENCODER_AB_Init(60000);  
//	AX_ENCODER_CD_Init(60000); 
//	AX_ENCODER_EF_Init(60000);  
//	AX_ENCODER_GH_Init(60000); 
//	
//	//设定中间值30000
//	AX_ENCODER_AB_SetCounter(30000); 
//	AX_ENCODER_CD_SetCounter(30000); 
//	AX_ENCODER_EF_SetCounter(30000); 
//	AX_ENCODER_GH_SetCounter(30000); 
//	
//	//提示信息
//	printf("四路正交编码器测试\r\n");

//	while (1) 
//	{		
//		printf("*AB:%5d CD:%5d EF:%5d GH:%5d \r\n",
//		AB_Get_Counter = AX_ENCODER_AB_GetCounter(),CD_Get_Counter = AX_ENCODER_CD_GetCounter(),
//		EF_Get_Counter = AX_ENCODER_EF_GetCounter(),GH_Get_Counter = AX_ENCODER_GH_GetCounter());  
//		AX_Delayms(300);
//	}
//}	


///******************************************************************************
//例程名称：直流减速电机PID速度控制
//例程说明：直流电机速度PID控制例程，可通过X-PrintfScope观察目标值与实际值曲线
//操作说明：硬件连接，电机连接控制板电机A接口，编码器连接AB接口
//         通过X-PrintfScope软件观察波形曲线，并进行电机转速设定和PID参数调节
//注意事项：塔克机器人中电机有左右两种，此例程适合其中一种，另一种需要调整电机转向。
//				 具体操作参考视频教程。
//*******************************************************************************/
//int16_t ax_encoder;	//编码器绝对值
//int16_t ax_encoder_delta;	//编码器相对变化值,代表实际速度
//	
//int16_t ax_encoder_delta_target = 0; //编码器目标值，代表目标速度
//int16_t ax_motor_pwm;  //电机PWM速度

//int16_t ax_motor_kp=300;  //PID参数
//int16_t ax_motor_ki=0;  //PID参数
//int16_t ax_motor_kd=200;  //PID参数

//int main(void)
//{
//  int32_t bias,bias_last,bias_integral = 0;
//	uint8_t comdata[32];
//	
//	//设置中断优先级分组
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    
//	
//	//初始化电机
//	AX_MOTOR_Init(10);  //设置电机控制PWM频率为10K

//	//JTAG口设置
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
//	AX_JTAG_Set(SWD_ENABLE);  //打开SWD接口 可以利用主板的SWD接口调试 
//	
//	//软件延时初始化
//	AX_DELAY_Init(); 	
//	AX_LED_Init();  //LED初始化
//	
//	//调试串口初始化
//	AX_UART_DB_Init(115200); //调试串口
//	
//	//定时器初始化
//	AX_TIM6_Init(40000);//设置定时器周期定时时间40ms,25HZ

//  //编码器初始化
//	AX_ENCODER_AB_Init(30000*2);  //正交编码器初始化
//	AX_ENCODER_AB_SetCounter(30000); 		//设置编码器初始值
//	
//	while (1) 
//	{	
//		//执行周期（40ms）25Hz
//		if(AX_TIM6_CheckIrqStatus())
//		{			
//			//计算编码器速度
//			ax_encoder_delta = -(AX_ENCODER_AB_GetCounter()-30000);
//	
//			//设置编码器初始中间值
//			AX_ENCODER_AB_SetCounter(30000);
//			
//			//PID控制
//			bias = ax_encoder_delta_target - ax_encoder_delta;
//			bias_integral += bias;
//			ax_motor_pwm += ax_motor_kp*bias*0.01f + ax_motor_kd*(bias-bias_last)*0.01f + ax_motor_ki*bias_integral*0.01f;
//			
//			bias_last = bias;
//			
//			//限制最大输出
//			if(ax_motor_pwm>2000)   ax_motor_pwm = 2000;
//			if(ax_motor_pwm<-2000)  ax_motor_pwm = -2000;			
//			
//			AX_MOTOR_A_SetSpeed(-ax_motor_pwm);
//			
//			//获取USB串口数据
//			if( AX_UART_DB_GetData(comdata))  
//			{
//				//左右轮速度控制
//				if(comdata[0] == 0x01)
//				{
//					ax_encoder_delta_target = (int16_t)((comdata[1]<<8) | comdata[2]);	
//				}

//				//速度控制PID参数
//				if(comdata[0] == 0x02)
//				{
//					ax_motor_kp = (int16_t)((comdata[1]<<8) | comdata[2]);
//					ax_motor_ki = (int16_t)((comdata[3]<<8) | comdata[4]);
//					ax_motor_kd = (int16_t)((comdata[5]<<8) | comdata[6]);
//				}

//			 }
//			
//				//串口发送电机目标转速和实际转速，编码器数据
//				comdata[0] = (u8)( ax_encoder_delta_target >> 8 );  
//				comdata[1] = (u8)( ax_encoder_delta_target );
//				comdata[2] = (u8)( ax_encoder_delta >> 8 );
//				comdata[3] = (u8)( ax_encoder_delta );
//			 
//				AX_UART_DB_SendPacket(comdata, 4, 0x03);
//				 
//				AX_LED_Green_Toggle();
//		}
//	}
//}

///******************************************************************************
//例程名称：OpenCRP与树莓派串口通信
//例程说明：OpenCRP通过与树莓派40PIN插针连接的串口，进行串口通信
//操作说明：此例程不方便通过连接树莓派观察现象，可以将发送接收引脚短接，实现自发，自收操作
//         通信采用X-Protocol，可使用塔克串口调试助手调试，具体详见视频教程
//*******************************************************************************/
	uint8_t comdata[32];

int main(void)
{
//	uint8_t comdata[32];
	
	//设置中断优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    
	
	//JTAG口设置
	AX_JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
	AX_JTAG_Set(SWD_ENABLE);  //打开SWD接口 可以利用主板的SWD接口调试 
	
	//软件延时初始化
	AX_DELAY_Init(); 	
	AX_LED_Init();  //LED初始化
	
	//调试串口初始化
	AX_UART_DB_Init(115200); //调试串口
	AX_UART_PI_Init(115200); //树莓派串口
	
	while (1) 
	{	
		   //查看树莓派串口是否接收到数据，如果接收到数据，通过USB串口输出数据
			 if( AX_UART_PI_GetData(comdata))  
			 {
				 printf("ID:%d  Data:%d %d %d %d \r\n",comdata[5],comdata[6],comdata[7],comdata[8],comdata[9]);		
			 }
		
			 //封装要发送的数据
			 comdata[0] = 1;  
			 comdata[1] = 10;
			 comdata[2] = 100;
			 comdata[3] = 200;
	     
			 //发送数据
			 AX_UART_PI_SendPacket(comdata, 4, 0x03);
			
       //延时100ms			 
			 AX_Delayms(100);
			 AX_LED_Green_Toggle();
	}
}

/******************* (C) 版权 2019 XTARK **************************************/

