#include "DHT11.h"
#include "os.h"
#include "delay.h"

void DHT11_IO_OUT (void){ //�˿ڱ�Ϊ���
	GPIO_InitTypeDef  GPIO_InitStructure; 	
    GPIO_InitStructure.Pin = DHT11_IO; //ѡ��˿ںţ�0~15��all��                        
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP; //ѡ��IO�ӿڹ�����ʽ       
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH; //����IO�ӿ��ٶȣ�2/10/50MHz��    
	HAL_GPIO_Init(DHT11PORT, &GPIO_InitStructure);
}

void DHT11_IO_IN (void){ //�˿ڱ�Ϊ����
	GPIO_InitTypeDef  GPIO_InitStructure; 	
    GPIO_InitStructure.Pin = DHT11_IO; //ѡ��˿ںţ�0~15��all��                        
     GPIO_InitStructure.Mode = GPIO_MODE_INPUT;//ѡ��IO�ӿڹ�����ʽ       
	HAL_GPIO_Init(DHT11PORT, &GPIO_InitStructure);
}

void DHT11_RST (void){ //DHT11�˿ڸ�λ��������ʼ�źţ�IO���ͣ�
	DHT11_IO_OUT();
	HAL_GPIO_WritePin(DHT11PORT,DHT11_IO,GPIO_PIN_RESET); //	
	Delay_us(18000); //��������18ms			
	HAL_GPIO_WritePin(DHT11PORT,DHT11_IO,GPIO_PIN_SET); //							
	Delay_us(30);//��������20~40us
}

uint8_t Dht11_Check(void){ //�ȴ�DHT11��Ӧ������1:δ��⵽DHT11������0:�ɹ���IO���գ�	 
    uint8_t retry=0;
    DHT11_IO_IN();//IO������״̬	 
    while (HAL_GPIO_ReadPin(DHT11PORT,DHT11_IO)&&retry<100){//DHT11������40~80us
        retry++;
        Delay_us(1);
    }	 
    if(retry>=100)return 1; else retry=0;
    while (!HAL_GPIO_ReadPin(DHT11PORT,DHT11_IO)&&retry<100){//DHT11���ͺ���ٴ�����40~80us
        retry++;
        Delay_us(1);
    }
    if(retry>=100)return 1;	    
    return 0;
}

uint8_t Dht11_ReadBit(void){ //��DHT11��ȡһ��λ ����ֵ��1/0
    uint8_t retry=0;
    while(HAL_GPIO_ReadPin(DHT11PORT,DHT11_IO)&&retry<100){//�ȴ���Ϊ�͵�ƽ
        retry++;
        Delay_us(1);
    }
    retry=0;
    while(!HAL_GPIO_ReadPin(DHT11PORT,DHT11_IO)&&retry<100){//�ȴ���ߵ�ƽ
        retry++;
        Delay_us(1);
    }
    Delay_us(40);//�ȴ�40us	//�����жϸߵ͵�ƽ��������1��0
    if(HAL_GPIO_ReadPin(DHT11PORT,DHT11_IO))return 1; else return 0;		   
}

uint8_t Dht11_ReadByte(void){  //��DHT11��ȡһ���ֽ�  ����ֵ������������
    uint8_t i,dat;
    dat=0;
    for (i=0;i<8;i++){ 
        dat<<=1; 
        dat|=Dht11_ReadBit();
    }						    
    return dat;
}

uint8_t DHT11_Init (void){	//DHT11��ʼ��
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, ENABLE); //APB2����ʱ��ʹ��    
GPIO_InitTypeDef  GPIO_InitStructure;
	 GPIO_InitStructure.Pin = DHT11_IO;//LED0?LED1??IO?
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;//??????
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;//100MHz
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;//??
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);//???GPIO 
	DHT11_RST();//DHT11�˿ڸ�λ��������ʼ�ź�
	return Dht11_Check(); //�ȴ�DHT11��Ӧ
}

uint8_t DHT11_ReadData(uint8_t *h){ //��ȡһ������//ʪ��ֵ(ʮ���ƣ���Χ:20%~90%) ���¶�ֵ(ʮ���ƣ���Χ:0~50��)������ֵ��0,����;1,ʧ�� 
    uint8_t buf[5];
    uint8_t i;
    DHT11_RST();//DHT11�˿ڸ�λ��������ʼ�ź�
    if(Dht11_Check()==0){ //�ȴ�DHT11��Ӧ
        for(i=0;i<5;i++){//��ȡ5λ����
            buf[i]=Dht11_ReadByte(); //��������
        }
        if((buf[0]+buf[1]+buf[2]+buf[3])==buf[4]){	//����У��
            *h=buf[0]; //��ʪ��ֵ����ָ��1
			h++;
            *h=buf[2]; //���¶�ֵ����ָ��2
        }
    }else return 1;
    return 0;	    
}
