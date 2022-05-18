#include "stm32f1xx_hal.h"
#include "oled.h"
#include "oledfont.h"
#include "main.h"
#include "os.h"
extern I2C_HandleTypeDef hi2c1;
void WriteCmd(unsigned char I2C_Command)//???
 {

		HAL_I2C_Mem_Write(&hi2c1,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&I2C_Command,1,100);

 }

void WriteDat(unsigned char I2C_Data)//???

 {

		HAL_I2C_Mem_Write(&hi2c1,OLED0561_ADD,DAT,I2C_MEMADD_SIZE_8BIT,&I2C_Data,1,100);

  }

	void OLED_Init(void)
{
	OS_ERR err;
	OSTimeDlyHMSM(0, 0, 0, 1000,OS_OPT_TIME_HMSM_STRICT,&err);
	WriteCmd(0xAE); //display off
	WriteCmd(0x00);	//Set Memory Addressing Mode
	WriteCmd(0x10);	//00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	WriteCmd(0x40);	//Set Page Start Address for Page Addressing Mode,0-7
	WriteCmd(0x81);	//Set COM Output Scan Direction
	WriteCmd(0xCF); //---set low column address
	WriteCmd(0xA1); //---set high column address
	WriteCmd(0xC8); //--set start line address
	WriteCmd(0xA6); //--set contrast control register
	WriteCmd(0xA8); //???? 0x00~0xff
	WriteCmd(0x3F); //--set segment re-map 0 to 127
	WriteCmd(0xD3); //--set normal display
	WriteCmd(0x00); //--set multiplex ratio(1 to 64)
	WriteCmd(0xD5); //
	WriteCmd(0x80); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	WriteCmd(0xD9); //-set display offset
	WriteCmd(0xF1); //-not offset
	WriteCmd(0xDA); //--set display clock divide ratio/oscillator frequency
	WriteCmd(0x12); //--set divide ratio
	WriteCmd(0xDB); //--set pre-charge period
	WriteCmd(0x40); //
	WriteCmd(0x20); //--set com pins hardware configuration
	WriteCmd(0x02);
	WriteCmd(0x8D); //--set vcomh
	WriteCmd(0x14); //0x20,0.77xVcc
	WriteCmd(0xA4); //--set DC-DC enable
	WriteCmd(0xA6); //
	OLED_CLS();
	WriteCmd(0xaf); //--turn on oled panel
}

void OLED_SetPos(unsigned char x, unsigned char y) //???????
{
	WriteCmd(0xb0+y);
	WriteCmd(((x&0xf0)>>4)|0x10);
	WriteCmd((x&0x0f));
}

void OLED_Fill(u8 fill_Data)//????
{
	u8 j,t;
	for(t=0xb0;t<0xb8;t++)
	{
		WriteCmd(t);		//page0-page1
		WriteCmd(0x10);		//low column start address
		WriteCmd(0x00);		//high column start address
		for(j=0;j<132;j++)
			{
				WriteDat(fill_Data);
			}
	}
}


void OLED_CLS(void)//??
{
	OLED_Fill(0x00);
}

void OLED_ON(void)
{
	WriteCmd(0X8D);  //?????
	WriteCmd(0X14);  //?????
	WriteCmd(0XAF);  //OLED??
}

void OLED_OFF(void)
{
	WriteCmd(0X8D);  //?????
	WriteCmd(0X10);  //?????
	WriteCmd(0XAE);  //OLED??
}


// Parameters     : x,y -- ?????(x:0~127, y:0~7); ch[] -- ???????; TextSize -- ????(1:6*8 ; 2:8*16)
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize)
{
	unsigned char c = 0,i = 0,j = 0;
	x = x + 2;
	switch(TextSize)
	{
		case 1:
		{
			while(ch[j] != '\0')
			{
				c = ch[j] - 32;
				if(x > 126)
				{
					x = 0;
					y++;
				}
				OLED_SetPos(x,y);
				for(i=0;i<6;i++)
					WriteDat(F6x8[c][i]);
				x += 6;
				j++;
			}
		}break;
		case 2:
		{
			while(ch[j] != '\0')
			{
				c = ch[j] - 32;
				if(x > 120)
				{
					x = 0;
					y++;
				}
				OLED_SetPos(x,y);
				for(i=0;i<8;i++)
					WriteDat(F8X16[c*16+i]);
				OLED_SetPos(x,y+1);
				for(i=0;i<8;i++)
					WriteDat(F8X16[c*16+i+8]);
				x += 8;
				j++;
			}
		}break;
	}
}


// Parameters     : x,y -- ?????(x:0~127, y:0~7); N:???.h????
// Description    : ??ASCII_8x16.h????,16*16??
void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N)
{
	unsigned char wm=0;
	unsigned int  adder=32*N;
	OLED_SetPos(x , y);
	for(wm = 0;wm < 16;wm++)
	{
		WriteDat(F16x16[adder]);
		adder += 1;
	}
	OLED_SetPos(x,y + 1);
	for(wm = 0;wm < 16;wm++)
	{
		WriteDat(F16x16[adder]);
		adder += 1;
	}
}



// Parameters     : x0,y0 -- ?????(x0:0~127, y0:0~7); x1,y1 -- ?????(???)???(x1:1~128,y1:1~8)
// Description    : ??BMP??
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[])
{
	unsigned int j=0;
	unsigned char x,y;

  if(y1%8==0)
		y = y1/8;
  else
		y = y1/8 + 1;
	for(y=y0;y<y1;y++)
	{
		OLED_SetPos(x0,y);
    for(x=x0;x<x1;x++)
		{
			WriteDat(BMP[j++]);
		}
	}
}






void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 Char_Size)
{
	unsigned char c=0,i=0;
		c=chr-' ';//???????
		if(x>128-1){x=0;y=y+2;}
		if(Char_Size ==16)
			{
			OLED_SetPos(x,y);
			for(i=0;i<8;i++)
			WriteDat(F8X16[c*16+i]);
			OLED_SetPos(x,y+1);
			for(i=0;i<8;i++)
			WriteDat(F8X16[c*16+i+8]);
			}
			else {
				OLED_SetPos(x,y);
				for(i=0;i<6;i++)
				WriteDat(F6x8[c][i]);

			}
}
u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;
	while(n--)result*=m;
	return result;
}


//??2???
//x,y :????
//len :?????
//size:????
//mode:??	0,????;1,????
//num:??(0~4294967295);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size2)
{
	u8 t,temp;
	u8 enshow=0;
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size2/2)*t,y,' ',size2);
				continue;
			}else enshow=1;

		}
	 	OLED_ShowChar(x+(size2/2)*t,y,temp+'0',size2);
	}
}

void OLED0561_Init (void){//OLED屏开显示初始化
	OLED_DISPLAY_OFF(); //OLED关显示
	OLED_DISPLAY_CLEAR(); //清空屏幕内容
	OLED_DISPLAY_ON(); //OLED屏初始值设置并开显示
}
void OLED_DISPLAY_ON (void){//OLED屏初始值设置并开显示
	u8 buf[28]={
	0xae,//0xae:关显示，0xaf:开显示
  0x00,0x10,//开始地址（双字节）       
	0xd5,0x80,//显示时钟频率？
	0xa8,0x3f,//复用率？
	0xd3,0x00,//显示偏移？
	0XB0,//写入页位置（0xB0~7）
	0x40,//显示开始线
	0x8d,0x14,//VCC电源
	0xa1,//设置段重新映射？
	0xc8,//COM输出方式？
	0xda,0x12,//COM输出方式？
	0x81,0xff,//对比度，指令：0x81，数据：0~255（255最高）
	0xd9,0xf1,//充电周期？
	0xdb,0x30,//VCC电压输出
	0x20,0x00,//水平寻址设置
	0xa4,//0xa4:正常显示，0xa5:整体点亮
	0xa6,//0xa6:正常显示，0xa7:反色显示
	0xaf//0xae:关显示，0xaf:开显示
	}; //
	HAL_I2C_Mem_Write(&hi2c1,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,buf,28,100);
}
void OLED_DISPLAY_OFF (void){//OLED屏关显示
	u8 buf[3]={
		0xae,//0xae:关显示，0xaf:开显示
		0x8d,0x10,//VCC电源
	}; //
	HAL_I2C_Mem_Write(&hi2c1,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,buf,3,100);
}

void OLED_DISPLAY_CLEAR(void){//清屏操作
	u8 j,t;
	for(t=0xB0;t<0xB8;t++){	//设置起始页地址为0xB0
		u8 buf[3] = {t,0x10,0x00,};
		HAL_I2C_Mem_Write(&hi2c1,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,buf,3,100);
		for(j=0;j<132;j++){	//整页内容填充
			WriteDat(0x00);
 		}
	}
}

//显示英文与数字8*16的ASCII码
//取模大小为8*16，取模方式为“从左到右从上到下”“纵向8点下高位”
void OLED_DISPLAY_8x16(u8 x, //显示汉字的页坐标（从0到7）（此处不可修改）
						u8 y, //显示汉字的列坐标（从0到128）
						uint16_t w){ //要显示汉字的编号
	u8 j,t,c=0;
	y=y+2; //因OLED屏的内置驱动芯片是从0x02列作为屏上最左一列，所以要加上偏移量
	for(t=0;t<2;t++){
		WriteCmd(0xb0+x);
		WriteCmd(y/16+0x10);
		WriteCmd(y%16);
		for(j=0;j<8;j++){ //整页内容填充
			WriteDat(F8X16[(w*16)+c-512]);
			c++;}x++; //页地址加1
	}
}
//向LCM发送一个字符串,长度64字符之内。
//应用：OLED_DISPLAY_8_16_BUFFER(0," DoYoung Studio"); 
void OLED_DISPLAY_8x16_BUFFER(u8 row,u8 *str){
	u8 r=0;
	while(*str != '\0'){
		OLED_DISPLAY_8x16(row,r*8,*str++);
		r++;
    }	
}
