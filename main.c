/**			                                                    
		   ____                    _____ _____  _____        XTARK@���˴���
		  / __ \                  / ____|  __ \|  __ \  
		 | |  | |_ __   ___ _ __ | |    | |__) | |__) |
		 | |  | | '_ \ / _ \ '_ \| |    |  _  /|  ___/ 
		 | |__| | |_) |  __/ | | | |____| | \ \| |     
		  \____/| .__/ \___|_| |_|\_____|_|  \_\_|     
		    		| |                                    
		    		|_|  OpenCRP ��ݮ�� ר��ROS�����˿�����                                   
									 
  ****************************************************************************** 
  *           
  * ��Ȩ���У� XTARK@���˴���  ��Ȩ���У�����ؾ�
  * ������վ�� www.xtark.cn
  * �Ա����̣� https://shop246676508.taobao.com  
  * ����ý�壺 www.cnblogs.com/xtark�����ͣ�
	* ����΢�ţ� ΢�Ź��ںţ����˴��£���ȡ������Ѷ��
  *      
  ******************************************************************************
  * @��  ��  Musk Han@XTARK
  * @��  ��  V1.0
  * @��  ��  2020-7-26
  * @��  ��  OpenCRP����������չʾʾ������
  * 
	******************************************************************************
  * @˵  ��
  *
  * 1.ʾ�����������û�ʹ��OpenCRP���������л����˱�̡�
	* 2.ÿ�����̶��������ԣ�ȡ��ע�ͣ��������ؼ��ɲ鿴Ч����
  * 3.ÿ�����̾��й���˵����
	*  
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>

#include "ax_sys.h"    //ϵͳ����
#include "ax_delay.h"  //�����ʱ
#include "ax_led.h"    //LED�ƿ���
#include "ax_vin.h"    //�����ѹ���
#include "ax_key.h"    //������� 
#include "ax_uart_db.h"  //���Դ���
#include "ax_uart_pi.h"  //��ݮ�ɴ���
#include "ax_motor.h"    //ֱ��������ٿ���
#include "ax_encoder.h"  //����������
#include "ax_servo.h"    //�������
#include "ax_tim.h"      //��ʱ��
#include "ax_mpu6050.h"  //IMU���ٶ������ǲ���
#include "ax_mpu6050_dmp.h"  //DMP���ܺ���

/******************************************************************************
      ��������  �嵥
			
* LED��˸�������������Դ���Printf�������
* VIN�����ѹ������̣����׵�����
* KEY������������̣��������
* MPU6050���ݲɼ�����
* �����������
* ֱ�����PWM�ٶȿ�������
* ���AB��������������
* ֱ�����PID��������
* STM32����ݮ��ͨ��

*******************************************************************************/

/******************************************************************************
�������ƣ�LED��˸�����Դ���Printf���
����˵������������ʾLED���ƺ͵��Դ���Printf����
*******************************************************************************/
//int main(void)
//{
//	uint8_t i=0;
//	
//	//�����ж����ȼ�����
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    

//	//JTAG������
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //�ر�JTAG�ӿ� 
//	AX_JTAG_Set(SWD_ENABLE);  //��SWD�ӿ� �������������SWD�ӿڵ��� 
//	
//	//�����ʱ��ʼ��
//	AX_DELAY_Init(); 	
//	AX_LED_Init();  //LED��ʼ��

//	//���Դ��ڳ�ʼ��
//	AX_UART_DB_Init(115200); //���Դ���
//	printf("  \r\n"); //����ո�CPUBUG
//	
//	//LED����0.5S
//	AX_LED_Green_On();
//	AX_Delayms(500);
//	AX_LED_Green_Off();
//	AX_Delayms(500);
//	
//	//LED����0.5S
//	AX_LED_Red_On();
//	AX_Delayms(500);
//	AX_LED_Red_Off();
//	AX_Delayms(500);
//	
//	AX_LED_Red_On();
//	
//	while (1)
//	{	
//    //���Դ��������Ϣ		
//		printf("Printf������ԣ�%d \r\n",i);
//		i++;
//		AX_Delayms(100);
//		
//		//LED��ת
//	  AX_LED_Green_Toggle();
//	  AX_LED_Red_Toggle();
//	}
//}


///******************************************************************************
//�������ƣ�VIN�����ѹ���
//����˵��������ѭ������ɼ����ĵ�ѹֵ
//*******************************************************************************/
//uint16_t vol;

//int main(void)
//{
////	uint16_t vol;
//	//�����ж����ȼ�����
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    

//	//JTAG������
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //�ر�JTAG�ӿ� 
//	AX_JTAG_Set(SWD_ENABLE);  //��SWD�ӿ� �������������SWD�ӿڵ��� 
//	
//	//�����ʱ��ʼ��
//	AX_DELAY_Init(); 	
//	
//	//���Դ��ڳ�ʼ��
//	AX_UART_DB_Init(115200); //���Դ���
//	printf("  \r\n"); //����ո�CPUBUG
//	
//	//VIN����ʼ��
//	AX_VIN_Init();
//	
//	//��ʾ��Ϣ
//	printf("*ѭ���ɼ�VIN����ڵ�ѹֵ\r\n");
//	
//	while (1) 
//	{	
//		//ÿ100MS���һ�ε�ص�ѹֵ
//		vol = AX_VIN_GetVol_X100();
//		printf("*VIN��ѹ��%d(0.01V)\r\n",vol );	
//		AX_Delayms(100);
//	}
//}


///******************************************************************************
//�������ƣ�KEY���������
//����˵�����������º�LED����˸һ��
//*******************************************************************************/
//int main(void)
//{
//	uint8_t temp;
//	
//	//�����ж����ȼ�����
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    

//	//JTAG������
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //�ر�JTAG�ӿ� 
//	AX_JTAG_Set(SWD_ENABLE);  //��SWD�ӿ� �������������SWD�ӿڵ��� 
//	
//	//�����ʱ��ʼ��
//	AX_DELAY_Init(); 	
//	AX_LED_Init();  	
//	
//	//���Դ��ڳ�ʼ��
//	AX_UART_DB_Init(115200); //���Դ���
//	printf("  \r\n"); //����ո�CPUBUG
//	
//	//��ʼ��
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
//�������ƣ���ȡMPU6050���ݲ����
//����˵������ȡMPU6050���ݣ�ͨ������Printf���
//����˵�����������������ٶȣ��������������ݣ�ת�����������۲����ݱ仯
//         ʹ�����ǵ�X-PrintfScope��������Թ۲쵽��������
//*******************************************************************************/
//int main(void)
//{
//	int16_t ax_acc[3],ax_gyro[3]; //IMU����
//	
//	//�����ж����ȼ�����
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    

//	//JTAG������
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //�ر�JTAG�ӿ� 
//	AX_JTAG_Set(SWD_ENABLE);  //��SWD�ӿ� �������������SWD�ӿڵ��� 
//	
//	//�����ʱ��ʼ��
//	AX_DELAY_Init(); 	
//	AX_LED_Init();  //LED��ʼ��

//	//���Դ��ڳ�ʼ��
//	AX_UART_DB_Init(115200); //���Դ���
//	printf("  \r\n"); //����ո�CPUBUG
//	
//	//MPU6050��ʼ��  
//	AX_MPU6050_Init();    
//	AX_MPU6050_SetAccRange(AX_ACC_RANGE_2G);    //���ü��ٶ�����
//	AX_MPU6050_SetGyroRange(AX_GYRO_RANGE_2000); //��������������
//	AX_MPU6050_SetGyroSmplRate(200);            //���������ǲ�����
//	AX_MPU6050_SetDLPF(AX_DLPF_ACC94_GYRO98);   //���õ�ͨ�˲�������
//	
//	while (1)
//	{	
//		//������̬�������ǡ����ٶ�����
//		AX_MPU6050_GetAccData(ax_acc);  //��ȡ������ٶ�����
//		AX_MPU6050_GetGyroData(ax_gyro);  //��ȡ��������������		
//		
//		//���Դ��������Ϣ
//		printf("@%d %d %d %d %d %d \r\n",
//		ax_acc[0],ax_acc[1],ax_acc[2],ax_gyro[0],ax_gyro[1],ax_gyro[2]);		
//		
//		AX_Delayms(50);
//	}
//}


///******************************************************************************
//�������ƣ������������
//����˵��������8·���60�ȡ�90�ȡ�120�ȼ���˶�
//����˵��������8·����ӿ�������һ·���������������ѭ��60�㡢90�㡢120���˶���
//         ��������·�������������Ť�ش���ע���Դ��������
//				 ע��OpenCRP4Ĭ���ǲ�����ģ���Ҫ��������ñ������ο�ԭ��ͼ
//*******************************************************************************/
//int main(void)
//{
//	//�����ж����ȼ�����
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
//	
//	//���Դ��ڳ�ʼ��
//	AX_UART_DB_Init(115200); //���Դ���
//	printf("  \r\n"); //����ո�CPUBUG
//	
//	//JTAG������
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //�ر�JTAG�ӿ� 
//	AX_JTAG_Set(SWD_ENABLE);  //��SWD�ӿ� �������������SWD�ӿڵ��� 		
//	
//	//�����ʱ��ʼ��
//	AX_DELAY_Init();
//  AX_LED_Init();
//	
//	//��ʼ��
//	AX_SERVO_AB_Init();
//  AX_SERVO_CD_Init();
//	AX_SERVO_EF_Init();
//	AX_SERVO_GH_Init();
//	
//	//��ʾ��Ϣ
//	printf("��·������Ʋ���\r\n");

//	while (1) 
//	{		
//		printf("*60��...... \r\n");		
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
//		printf("*90��...... \r\n");
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
//		printf("*120��...... \r\n");
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
//�������ƣ�ֱ�����PWM���ٿ���
//����˵��������4·������������ͱ��ٷ�ת��������
//����˵�����������������·���ƽӿ��е�����һ·�����Կ�������������������ת�ͷ�ת
//*******************************************************************************/
//int main(void)
//{
//	uint16_t temp;
//	
//	//�����ж����ȼ�����
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
//	
//	//JTAG������
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //�ر�JTAG�ӿ� 
//	AX_JTAG_Set(SWD_ENABLE);  //��SWD�ӿ� �������������SWD�ӿڵ��� 		
//	
//	//�����ʱ��ʼ��
//	AX_DELAY_Init(); 	
//  AX_LED_Init();  
//	
//	//��ʼ��
//	AX_MOTOR_Init(10);  //���õ������PWMƵ��Ϊ10K

//	while (1) 
//	{	
//		//���Ƶ��ת��
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
//		//���Ƶ������ת��
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
//�������ƣ�����AB����������
//����˵����300ms����5·��������ֵ�����������ʾ�����50%PWMռ�ձ�����
//����˵����4·�����������ã����ӱ������󣬼����ֶ�ת������۲촮�����ֵ�仯
//         Ҳ�ɽ�������ӵ�����ӿ�
//*******************************************************************************/
//int AB_Get_Counter,CD_Get_Counter,EF_Get_Counter,GH_Get_Counter;
//int main(void)
//{
//	//�����ж����ȼ�����
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
//	
//	//���Դ��ڳ�ʼ��
//	AX_UART_DB_Init(115200); //���Դ���
//	printf("  \r\n"); //����ո�CPUBUG
//	
//	//JTAG������
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //�ر�JTAG�ӿ� 
//	AX_JTAG_Set(SWD_ENABLE);  //��SWD�ӿ� �������������SWD�ӿڵ��� 		
//	
//	//�����ʱ��ʼ��
//	AX_DELAY_Init(); 	
//  AX_LED_Init();   
//	
//	//������������ʼ��
//  AX_ENCODER_AB_Init(60000);  
//	AX_ENCODER_CD_Init(60000); 
//	AX_ENCODER_EF_Init(60000);  
//	AX_ENCODER_GH_Init(60000); 
//	
//	//�趨�м�ֵ30000
//	AX_ENCODER_AB_SetCounter(30000); 
//	AX_ENCODER_CD_SetCounter(30000); 
//	AX_ENCODER_EF_SetCounter(30000); 
//	AX_ENCODER_GH_SetCounter(30000); 
//	
//	//��ʾ��Ϣ
//	printf("��·��������������\r\n");

//	while (1) 
//	{		
//		printf("*AB:%5d CD:%5d EF:%5d GH:%5d \r\n",
//		AB_Get_Counter = AX_ENCODER_AB_GetCounter(),CD_Get_Counter = AX_ENCODER_CD_GetCounter(),
//		EF_Get_Counter = AX_ENCODER_EF_GetCounter(),GH_Get_Counter = AX_ENCODER_GH_GetCounter());  
//		AX_Delayms(300);
//	}
//}	


///******************************************************************************
//�������ƣ�ֱ�����ٵ��PID�ٶȿ���
//����˵����ֱ������ٶ�PID�������̣���ͨ��X-PrintfScope�۲�Ŀ��ֵ��ʵ��ֵ����
//����˵����Ӳ�����ӣ�������ӿ��ư���A�ӿڣ�����������AB�ӿ�
//         ͨ��X-PrintfScope����۲첨�����ߣ������е��ת���趨��PID��������
//ע��������˻������е�����������֣��������ʺ�����һ�֣���һ����Ҫ�������ת��
//				 ��������ο���Ƶ�̡̳�
//*******************************************************************************/
//int16_t ax_encoder;	//����������ֵ
//int16_t ax_encoder_delta;	//��������Ա仯ֵ,����ʵ���ٶ�
//	
//int16_t ax_encoder_delta_target = 0; //������Ŀ��ֵ������Ŀ���ٶ�
//int16_t ax_motor_pwm;  //���PWM�ٶ�

//int16_t ax_motor_kp=300;  //PID����
//int16_t ax_motor_ki=0;  //PID����
//int16_t ax_motor_kd=200;  //PID����

//int main(void)
//{
//  int32_t bias,bias_last,bias_integral = 0;
//	uint8_t comdata[32];
//	
//	//�����ж����ȼ�����
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    
//	
//	//��ʼ�����
//	AX_MOTOR_Init(10);  //���õ������PWMƵ��Ϊ10K

//	//JTAG������
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //�ر�JTAG�ӿ� 
//	AX_JTAG_Set(SWD_ENABLE);  //��SWD�ӿ� �������������SWD�ӿڵ��� 
//	
//	//�����ʱ��ʼ��
//	AX_DELAY_Init(); 	
//	AX_LED_Init();  //LED��ʼ��
//	
//	//���Դ��ڳ�ʼ��
//	AX_UART_DB_Init(115200); //���Դ���
//	
//	//��ʱ����ʼ��
//	AX_TIM6_Init(40000);//���ö�ʱ�����ڶ�ʱʱ��40ms,25HZ

//  //��������ʼ��
//	AX_ENCODER_AB_Init(30000*2);  //������������ʼ��
//	AX_ENCODER_AB_SetCounter(30000); 		//���ñ�������ʼֵ
//	
//	while (1) 
//	{	
//		//ִ�����ڣ�40ms��25Hz
//		if(AX_TIM6_CheckIrqStatus())
//		{			
//			//����������ٶ�
//			ax_encoder_delta = -(AX_ENCODER_AB_GetCounter()-30000);
//	
//			//���ñ�������ʼ�м�ֵ
//			AX_ENCODER_AB_SetCounter(30000);
//			
//			//PID����
//			bias = ax_encoder_delta_target - ax_encoder_delta;
//			bias_integral += bias;
//			ax_motor_pwm += ax_motor_kp*bias*0.01f + ax_motor_kd*(bias-bias_last)*0.01f + ax_motor_ki*bias_integral*0.01f;
//			
//			bias_last = bias;
//			
//			//����������
//			if(ax_motor_pwm>2000)   ax_motor_pwm = 2000;
//			if(ax_motor_pwm<-2000)  ax_motor_pwm = -2000;			
//			
//			AX_MOTOR_A_SetSpeed(-ax_motor_pwm);
//			
//			//��ȡUSB��������
//			if( AX_UART_DB_GetData(comdata))  
//			{
//				//�������ٶȿ���
//				if(comdata[0] == 0x01)
//				{
//					ax_encoder_delta_target = (int16_t)((comdata[1]<<8) | comdata[2]);	
//				}

//				//�ٶȿ���PID����
//				if(comdata[0] == 0x02)
//				{
//					ax_motor_kp = (int16_t)((comdata[1]<<8) | comdata[2]);
//					ax_motor_ki = (int16_t)((comdata[3]<<8) | comdata[4]);
//					ax_motor_kd = (int16_t)((comdata[5]<<8) | comdata[6]);
//				}

//			 }
//			
//				//���ڷ��͵��Ŀ��ת�ٺ�ʵ��ת�٣�����������
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
//�������ƣ�OpenCRP����ݮ�ɴ���ͨ��
//����˵����OpenCRPͨ������ݮ��40PIN�������ӵĴ��ڣ����д���ͨ��
//����˵���������̲�����ͨ��������ݮ�ɹ۲����󣬿��Խ����ͽ������Ŷ̽ӣ�ʵ���Է������ղ���
//         ͨ�Ų���X-Protocol����ʹ�����˴��ڵ������ֵ��ԣ����������Ƶ�̳�
//*******************************************************************************/
	uint8_t comdata[32];

int main(void)
{
//	uint8_t comdata[32];
	
	//�����ж����ȼ�����
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    
	
	//JTAG������
	AX_JTAG_Set(JTAG_SWD_DISABLE);  //�ر�JTAG�ӿ� 
	AX_JTAG_Set(SWD_ENABLE);  //��SWD�ӿ� �������������SWD�ӿڵ��� 
	
	//�����ʱ��ʼ��
	AX_DELAY_Init(); 	
	AX_LED_Init();  //LED��ʼ��
	
	//���Դ��ڳ�ʼ��
	AX_UART_DB_Init(115200); //���Դ���
	AX_UART_PI_Init(115200); //��ݮ�ɴ���
	
	while (1) 
	{	
		   //�鿴��ݮ�ɴ����Ƿ���յ����ݣ�������յ����ݣ�ͨ��USB�����������
			 if( AX_UART_PI_GetData(comdata))  
			 {
				 printf("ID:%d  Data:%d %d %d %d \r\n",comdata[5],comdata[6],comdata[7],comdata[8],comdata[9]);		
			 }
		
			 //��װҪ���͵�����
			 comdata[0] = 1;  
			 comdata[1] = 10;
			 comdata[2] = 100;
			 comdata[3] = 200;
	     
			 //��������
			 AX_UART_PI_SendPacket(comdata, 4, 0x03);
			
       //��ʱ100ms			 
			 AX_Delayms(100);
			 AX_LED_Green_Toggle();
	}
}

/******************* (C) ��Ȩ 2019 XTARK **************************************/

