/*
 * SPM.ino
 *
 * Created: 10/10/2014 9:10:14 AM
 * Author: Дмитрий Владимирович
 */ 

//#include <TimerOne.h>
#include <SPI.h>         
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008

#define TIMER_CLOCK_FREQ 2000000 //2MHz for /8 prescale from 16MHz
#define SpiEthernet 53 //  SPI для ethernet shield
#define STLin1 0 // статус Lin1 в STT
#define STLin2 1 // статус Lin2 в STT
#define PK_IN 46 //пневмоклапан впускной
#define PK_OUT 48//пневмоклапан выпускной

//-----------Настройка сетевого соединения-------------------//
// Enter a MAC address && IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x76 }; //-------------------------- MAC адрес устройства------------------------------//
IPAddress ip(192, 168, 0, 176);		  //---------------------------ip адрес устройства-------------------------------//
IPAddress ipServer(192, 168, 0, 255); //послать всем

unsigned int localPort = 21666;      //------------------номер локального порта для прослушивания--------------------//

unsigned int Send_Time_PUP=6; //--------------------------Время открывания ВПУСКНОГО КЛАПАНА мс----------------------//
unsigned int Send_Time_PDN=6; //--------------------------Время открывания ВЫПУСКНОГО КЛАПАНА мс---------------------//
unsigned int time_PLock=1000; //---------------------Время на сичтивание и усреднение показаний АЦП мс---------------//

EthernetUDP Udp; // Создание экземпляра класса EthernetUDP для отправки и получения UDP-пакетов

byte IPS0, IPS1, IPS2, IPS3; // пременный для действительно IP адреса server
//----------------------------------------------------------------------------//

int ledPin = 13; //мигалка на 13й вход (встроенная, чтоб понятно было, что ничего не повисло)
int PIN17 = 17; //требуется по разводке сделать как вход

int dataIn_1 = 28; //шина данных, можно менять Line_1  						
int clockIn_1 = 18; //шина clock, не трогать, так надо ( attachInterrupt) Line_1   

int dataIn_2 = 26; //шина данных, можно менять Line_2						
int clockIn_2 = 19; //шина clock, не трогать, так надо ( attachInterrupt) Line_2   

int ADCPRSPin = 8;     // номер аналогового входа к которому подключен датчик давления
unsigned int ADCPRS=0, ADCPRSnf=0; //измеренное значение давления
unsigned int XPress[16];
float FK = 0.5; //настройки фильтра ФНЧ

byte isin = 0, isin1 = 0, isin2 = 0; //д=1 мм=0
byte isfs = 0, isfs1 = 0,  isfs2 = 0; //минус
byte index1, index2, CXP; //счётчик битов


unsigned int xData1, xData2 , xDataBuf2, xDataBuf1; //новые показания 
int xDataS, xDataS1Buf, xDataS2Buf;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //буфер считанных данных с UDP
unsigned int t_1ms, t_100ms, resultT1, resultT3;
byte fl_1s, fl_100ms;

byte tTimeout_1 = 0; //таймер 1мс для Timeout Line_1
byte tTimeout_2 = 0; //таймер 1мс для Timeout Line_2
byte Timeout =10; //таймаут чтения битов в мс

struct TUprOut{ unsigned char Flag, KodKom; };
TUprOut UprOut[16];

byte ZagrOut, VbrkOut, CRC;
byte LenBuf;

byte BufOut[256];

byte StartPkt, KodPkt, XC, Vbrk, CRS, Rejim, EnPkt, NumPkt, NumPvt, NumKmd, LenPkt;
int CA;

//переменные для предачи на сервер

byte t_1min;
boolean FLin1Dis=0, FLin2Dis=0; //состояние датчиков Lin
byte STT=0x00, NumStep; //статус устройства, NumStep - номер шага.
unsigned int minutes=0, Km=1, KmL, KmH, Kn=0, KnL, KnH; // minutes - проидено минут с начала шага, Km и Kn - коэффициенты маштабирования и нуля.
float Kmf;

struct TUpr{ unsigned int P, T; };
TUpr AUTO_Press[10];
byte CntKom, CPrm, BufKmd, FStart;
byte *PByte;
byte BufPrm[41];

// переменный для стабилизации

boolean FSetPrsSt, FSetPrsSt300, fl_1min, FTST, fl_RZ, FlStop=0; //флаг установки давления стабилизации, флаг окончательной устновки после 300 задержки
byte NumStepM=0;
unsigned int AUTO_Press_ST; // текущее давление стабилизации
unsigned int CZagrP, CVbrkP, Delta, DINT, CUSTPRESS;
int  XPLin=0xFFF;
int PrirLin[512]; // массив для расчета приращений

// переменные для метода набора давления

boolean  fl_PressUp, fl_Run_PressDn, fl_PressDn, fl_time_PFRZUp , fl_time_PFRZDn;
unsigned int t3_10ms, time_PUP, time_PFRZDn, time_PFRZUp, time_PDN;

void setup(){	
	digitalWrite (dataIn_1, 1);
	digitalWrite (clockIn_1, 1);
	pinMode (dataIn_1, INPUT); //привязываем шину данных на dataIn Line_1
	pinMode (clockIn_1, INPUT); //и clock на 2й вход line_1
	
	digitalWrite (dataIn_2, 1);
	digitalWrite (clockIn_2, 1);
	pinMode (dataIn_2, INPUT); //привязываем шину данных на dataIn Line_2
	pinMode (clockIn_2, INPUT); //и clock на 3й вход Line_2	
	
	attachInterrupt(5,getBit,FALLING); //и аттачим clock также на 2й вход //FALLING
	attachInterrupt(4,getBit_1,FALLING); //и аттачим clock также на 2й вход  //FALLING
	
//	Timer1.initialize(1000); // установить интервал срабатывания 1000 microseconds 
//	Timer1.attachInterrupt( timerIsr ); // устновка функции (обработчика прерывания)
	Timer1_init(1000);	// установить интервал срабатывания 1000 microseconds 
	Timer3_init(1000);  // установить интервал срабатывания 1000 microseconds 
    
	
	pinMode (ledPin, OUTPUT); //Настройка LED как выход
	pinMode (PIN17, INPUT); //Настройка как вход требуется по разводке
	
	pinMode (PK_IN, OUTPUT); // впускной клапан
	pinMode (PK_OUT, OUTPUT); // выпускной клапан
	
	pinMode(SpiEthernet, OUTPUT); //SS настройка как выхода для работы SPIn с Ethernet Shield
	
	// start the Ethernet && UDP:
	Ethernet.begin(mac,ip);
	Udp.begin(localPort);

	Serial.begin(9600);
	delay(500);
	
	BufOut[0]=0x93;
	BufOut[1]=0x6A;
	BufOut[2]=0x00;
	BufOut[3]=0x00;
	BufOut[4]=0x00;
}

//void timerIsr() //прерывание 1ms
ISR(TIMER1_OVF_vect) //прерывание 1ms
{
	TCNT1=resultT1;
	t_1ms++;
	t_100ms++;
	tTimeout_1++;
	tTimeout_2++;
	if (tTimeout_1>=251) {tTimeout_1=250;}
	if (tTimeout_2>=251) {tTimeout_2=250;}
	if (t_1ms>=1000) {fl_1s=1; t_1ms=0; t_1min++;}
	if (t_100ms>=100) {fl_100ms=1; t_100ms=0;}	
	if (t_1min>=60) {t_1min=60;}	
	if ((t_1min>=60)&&(FStart==1)) {minutes++; t_1min=0; fl_1min=1;}
}

ISR(TIMER3_OVF_vect) //прерывание 1ms
{
	TCNT3=resultT3;		
		if (fl_PressUp==1 && time_PUP<Send_Time_PUP && fl_time_PFRZUp==1) { // ВПУСКНОЙ КЛАПАН
			time_PUP++;
			PORTL=PORTL | B00001000; //PL3 установка диапазона стабилзции давления (открыть впускной)
			}	
		else{
//			PORTL=PORTL & B11110111; // (закрыть впускной)
			time_PUP=0;
			fl_time_PFRZUp=0;
		}	
		
		if (fl_time_PFRZUp==0 )	{ //ожидание времени для усреднений значений АЦП
			time_PFRZUp++;
			PORTL=PORTL & B11110111; // (закрыть впускной)
			if (time_PFRZUp>=time_PLock) { time_PFRZUp=0; fl_time_PFRZUp=1; time_PUP=0; }
		}		
//----------	
		if (fl_PressDn==1 && time_PDN<Send_Time_PDN && fl_time_PFRZDn==1) { // ВЫПУСКНОЙ КЛАПАН
			time_PDN++;
			PORTL=PORTL | B00000010; //PL1 установка диапазона стабилзции давления (открыть выпускной)
		}
		else{
//			PORTL=PORTL & B11111101; // (закрыть выпускной)
			time_PDN=0;
			fl_time_PFRZDn=0;
		}		
				
		if (fl_time_PFRZDn==0)	{ //ожидание времени для усреднений значений АЦП
			time_PFRZDn++;
			PORTL=PORTL & B11111101; // (закрыть выпускной)
			if (time_PFRZDn>=time_PLock) { time_PFRZDn=0;  fl_time_PFRZDn=1; time_PDN=0; }
		}		
//	}
}

void loop()
{
	
DataLine();	//Формирование данных линейных датчиков
ReadDatUDP(); // Формирование данных приянтых по UDP
PRSAUTOST() ;//Автоматический режим стабилизации
OutDatUDP();  //--- Формирование данных для отправки по UDP
	
if (fl_100ms==1) {
	DataAdc(); // Чтение АЦП - показания давления
//	fl_100ms=0;
}

if(fl_1s)
{
//	PORTB=PORTB^B10000000; //светодиод D13 мигалка раз в 1 с	
	fl_1s=0;	
	if (Rejim==0){
		UprOut[ZagrOut].Flag=1;
		UprOut[ZagrOut].KodKom=1;
		ZagrOut=0xF &(ZagrOut+1);
		}
	if (Rejim==1){
		UprOut[ZagrOut].Flag=1;
		UprOut[ZagrOut].KodKom=3;
		ZagrOut=0xF &(ZagrOut+1);
		}	
}

if (fl_1min) {
	fl_1min=0;
	Prir(); // расчет приращения за время условной стабилизации	
}

}


void Timer1_init(unsigned long millisecondT1) 
{
	resultT1=65536-TIMER_CLOCK_FREQ*millisecondT1/1000000;
//	cli(); // Запрещаем все прерывания на время инициализации.
	TCCR1A = 0;	// clear control register A  
	TCCR1B = _BV(CS11); // предделитель на 8   
	TIMSK1|=(1<<TOIE1);
    TCNT1=resultT1;
//	sei(); // Закончили инициализацию, разрешаем все прерывания.
}		

void Timer3_init(unsigned long millisecondT3)
{
	resultT3=65536-TIMER_CLOCK_FREQ*millisecondT3/1000000;
	//	cli(); // Запрещаем все прерывания на время инициализации.
	TCCR3A = 0;	// clear control register A
	TCCR3B = _BV(CS31); // предделитель на 8
	TIMSK3|=(1<<TOIE3);
	TCNT3=resultT3;
	//	sei(); // Закончили инициализацию, разрешаем все прерывания.
}

void OutDatUDP()
{
	if (UprOut[VbrkOut].Flag==1)
	  {

		if (UprOut[VbrkOut].KodKom==1)
		{
			BufOut[2]=0x82;     //--- Код Пакета (Команда)
			BufOut[3]=0x00;
			BufOut[4]=0x00;
			BufOut[5]=0x01;     //--- Длина Информационной Части
			BufOut[6]=0x41;     //--- Код Команды (Запрос Регистрации Устройства)
			LenBuf=6;
		}

		if (UprOut[VbrkOut].KodKom==2)
		{
			BufOut[2]=0x81;     //--- Код Пакета (Служебная Квитанция Подтверждения)
			BufOut[3]=0x00;
			BufOut[4]=0x00;			
			BufOut[5]=0x03;     //--- Длина Информационной Части
			BufOut[6]=NumKmd;   //--- Номер Пакета
			BufOut[7]=NumPkt;   //--- Номер Пакета
			BufOut[8]=NumPvt;   //--- Номер Повтора
			LenBuf=8;
			}		

		if (UprOut[VbrkOut].KodKom==3)
		{ // формирование пакета данных	
			BufOut[2]=0x85;      //--- Код Пакета (Параметры Контроллера)
			BufOut[3]=0x00;
			BufOut[4]=0x00;			
			BufOut[5]=0x13;      //--- Длина Информационной Части
			BufOut[6]=isin1;     //--- Признак системы измерения
			BufOut[7]=isfs1;     //--- Знак Числа
			BufOut[8]=xData1;    //--- Значение измерения (Младшая часть)
			BufOut[9]=xData1>>8; //--- Значение измерения (Старшая часть)
			BufOut[10]=isin2;     //--- Признак системы измерения
			BufOut[11]=isfs2;     //--- Знак Числа
			BufOut[12]=xData2;    //--- Значение измерения (Младшая часть)
			BufOut[13]=xData2>>8;	//--- Значение измерения (Старшая часть)
			BufOut[14]=xDataS;		//--- Значение среднее (Младшая часть)
			BufOut[15]=xDataS>>8;	//--- Значение среднее (Старшая часть)	
			BufOut[16]=XPLin;		//--- Дельта (Младшая часть)
			BufOut[17]=XPLin>>8;	//--- Дельта (Старшая часть)					
			BufOut[18]=ADCPRS;		//--- АЦП (Младшая часть)
			BufOut[19]=ADCPRS>>8;   //--- АЦП (Старшая часть)
			BufOut[20]=minutes;     //--- минуты (Младшая часть)
			BufOut[21]=minutes>>8;  //--- минуты (Старшая часть)
			BufOut[22]=t_1min;		//--- секунды (Старшая часть)
			BufOut[23]=NumStep;     // номер шага		
			BufOut[24]=STT;		// ---статус устройств	

// 			BufOut[6]=isin1;     //--- Признак системы измерения				****	
// 			BufOut[7]=isfs1;     //--- Знак Числа								****
// 			BufOut[8]=0x01;    //--- Значение измерения (Младшая часть)			****
// 			BufOut[9]=0x01; //--- Значение измерения (Старшая часть)			****	
// 			BufOut[10]=isin2;     //--- Признак системы измерения				****
// 			BufOut[11]=isfs2;     //--- Знак Числа								****
// 			BufOut[12]=0x02;    //--- Значение измерения (Младшая часть)		****
// 			BufOut[13]=0x02;	//--- Значение измерения (Старшая часть)		****
// 			BufOut[14]=0x03;		//--- Значение среднее (Младшая часть)		****
// 			BufOut[15]=0x03;	//--- Значение среднее (Старшая часть)			****
// 			BufOut[16]=0x04;		//--- АЦП (Младшая часть)					****
// 			BufOut[17]=0x04;   //--- АЦП (Старшая часть)						****																
//			BufOut[24]=STT|0x03;		// ---статус устройств //отладка		****
			
			LenBuf=24;
			}

		if (UprOut[VbrkOut].KodKom==4)
		{ // Конфигурация устройства управления
			BufOut[2]=0x83;      //--- Код Пакета (Параметры Контроллера)
			BufOut[3]=0x00;
			BufOut[4]=0x00;			
			BufOut[5]=4*CntKom;      //--- Длина Информационной Части
			PByte=(byte*) & AUTO_Press[0];
			CA=0;
			while(CA<4*CntKom)
			 {
			 BufOut[6+CA]= *PByte; PByte++;
			 CA++;
			 }
			BufOut[6+CA]=STT;     //--- статус
			LenBuf=6+CA;
		}			

		Udp.beginPacket(ipServer, localPort);
		CA=0; CRC=0;
		while(CA<=LenBuf)
		{
			if (CA>=2) CRC=CRC+BufOut[CA];
			Udp.write(BufOut[CA]);
			CA++;
		}
		Udp.write(CRC);
		Udp.endPacket();

		UprOut[VbrkOut].Flag=0;
		VbrkOut=0xF &(VbrkOut+1);
	}
}

 
void ReadDatUDP()
{
//   Чтение UDP пакетов
//   Проверяем наличие принятых данных
  int packetSize = Udp.parsePacket(); //возвращает количество принятых байт
  if (packetSize)
   {
	   Udp.read(packetBuffer,UDP_TX_PACKET_MAX_SIZE);  // считывание пакета данных в packetBufffer
	   CA=0; CRS=0x00;
	   StartPkt=0; Vbrk=0;

 	   while(CA<packetSize)
 	   {
 		   XC=packetBuffer[CA];
 //			   Serial.write(XC);
 		   if ((StartPkt==2)&&(Vbrk==1))  { KodPkt=XC; CRS=0x00; CPrm=0; }
 		   if ((StartPkt==2)&&(Vbrk==2))  { NumPkt=XC; }
 		   if ((StartPkt==2)&&(Vbrk==3))  { NumPvt=XC; }
 		   if ((StartPkt==2)&&(Vbrk==4))  { LenPkt=XC; }
 		   if ((XC==0x6A)&&(StartPkt==1)) { StartPkt=StartPkt+1; }
 		   if (StartPkt==1) StartPkt=0;
 		   if ((XC==0x93)&&(StartPkt==0)) { StartPkt=StartPkt+1; }
 		   
 		if ((KodPkt==0x01)&&(Vbrk>=1)) //Обработка по приему пакета "команды" 0x01
 		{
			if (Vbrk==1) EnPkt=1;
			if (Vbrk==5) BufKmd=XC;
	 		if (Vbrk==6)
	 		{
		 		if (CRS==XC)	//проверка контрольной суммы
				 { 
//			 		Serial.write(CRS);
//			 		Serial.write(XC);
                    if (BufKmd==0x41)		// "Выполнить авторизацию" перерегестрация устройства на сервере
						{
			 			StartPkt=0; Vbrk=0; EnPkt=0; Rejim=0;
			 			UprOut[ZagrOut].Flag=1; UprOut[ZagrOut].KodKom=2; ZagrOut=0xF &(ZagrOut+1); //подтверждение приема
						ipServer[0]=0xC0; ipServer[1]=0xA8; ipServer[2]=0x00; ipServer[3]=0xFF; //установка IP adres server широковещательный
						}
					if (BufKmd==0x42)		//передать загруженные команды управления и флаги состояния устройства 
						{
						StartPkt=0; Vbrk=0; EnPkt=0; 
						UprOut[ZagrOut].Flag=1; UprOut[ZagrOut].KodKom=2; ZagrOut=0xF &(ZagrOut+1); //подтверждение приема
						UprOut[ZagrOut].Flag=1; UprOut[ZagrOut].KodKom=4; ZagrOut=0xF &(ZagrOut+1); //передать конфигурацию устройства
						}					
                    if (BufKmd==0x43)		// начать испытание
						{
			 			StartPkt=0; Vbrk=0; EnPkt=0;
			 			UprOut[ZagrOut].Flag=1; UprOut[ZagrOut].KodKom=2; ZagrOut=0xF &(ZagrOut+1); //подтверждение приема
						FStart=1; t_1min=0; STT|=0x40;
						}
                    if (BufKmd==0x44)		// Остановить испытание
						{
	                    StartPkt=0; Vbrk=0; EnPkt=0;
	                    UprOut[ZagrOut].Flag=1; UprOut[ZagrOut].KodKom=2; ZagrOut=0xF &(ZagrOut+1); //подтверждение приема
						// окончание испытания
						FStart=0; STT&=0xBF; FSetPrsSt=0; minutes=0; t_1min=0; fl_PressUp=0; fl_PressDn=0;
						FlStop=1; //Произвести спуск системы
						fl_RZ=0; //режим замочки остановлен
						FSetPrsSt300=0; STT&=0xDF; FTST=0; STT&=0xEF; Delta=0; CZagrP=0; CVbrkP=0;
						XPLin=0xFFF; NumStepM=0;
						PORTL=PORTL & B11110111;
						PORTL=PORTL & B11111101;				// закрыть выпускной клапан						
						}
		 		 }
	 		}
 		}
		 
 		if ((KodPkt==0x02)&&(Vbrk>=1)) //Обработка по приему пакета "Подтверждение регистрации усройства на сервере" 0x02
 		{
	 		if (Vbrk==1) EnPkt=1;
	 		if (Vbrk==5) IPS0=XC;
	 		if (Vbrk==6) IPS1=XC;
	 		if (Vbrk==7) IPS2=XC;
	 		if (Vbrk==8) IPS3=XC;  
	 		if (Vbrk==9)
	 		{
		 		if (CRS==XC)	//проверка контрольной суммы
				 { 
			 		//		    	Serial.write(CRS);
			 		//				Serial.write(XC);
			 		Rejim=1;
			 		StartPkt=0; Vbrk=0; EnPkt=0;
					ipServer[0]=IPS0; ipServer[1]=IPS1; ipServer[2]=IPS2; ipServer[3]=IPS3; //установка рефльного IP adres server
			 		UprOut[ZagrOut].Flag=1; UprOut[ZagrOut].KodKom=2; ZagrOut=0xF &(ZagrOut+1); //выставить флаги для отправки подтверждения
		 		 }
	 		}
 		}		 

 		if ((KodPkt==0x03)&&(Vbrk>=1)) //Обработка по приему пакета "Параметры переключения ступеней" 0x03
 		  {
 			if (Vbrk==1) EnPkt=1;
 			if (Vbrk>=5)
 			   {
 				BufPrm[CPrm]=XC;
 			   if ((CRS==XC)&&(CPrm==LenPkt))
 				  { //проверка контрольной суммы
// 					 Serial.write(CRS);
// 					 Serial.write(XC);
 					 PByte=(byte*) &AUTO_Press[0];
 					 CA=0;
 					 while(CA<LenPkt)
 					   {
 						 *PByte=BufPrm[CA]; PByte++;
 						 CA++;
 					   }
 					 while(CA<40)
 				       {
	 					 *PByte=0x00; PByte++;
	 					 CA++;
 					   }					 	
                     CntKom=CPrm >> 2;
 					 StartPkt=0; Vbrk=0; EnPkt=0;
 					 UprOut[ZagrOut].Flag=1; UprOut[ZagrOut].KodKom=2; ZagrOut=0xF &(ZagrOut+1); //выставить флаги для отправки подтверждения принятия команд управления
 			     }
			    CPrm++;
 			 }
 		 }

 		if ((KodPkt==0x04)&&(Vbrk>=1)) //Обработка по приему пакета "Подтверждение регистрации усройства на сервере" 0x02
 		{
	 		if (Vbrk==1) EnPkt=1;
	 		if (Vbrk==5) KmL=XC;
	 		if (Vbrk==6) KmH=XC;
	 		if (Vbrk==7) KnL=XC;
	 		if (Vbrk==8) KnH=XC;  
	 		if (Vbrk==9)
	 		{
		 		if (CRS==XC)	//проверка контрольной суммы
		 		{
//			 				    	Serial.write(KmL);
//			 						Serial.write(KmH);
//			 				    	Serial.write(KnL);
//			 				    	Serial.write(KnH);									 
					Km=KmL|KmH<<8; Kn=KnL|KnH<<8;
					Kmf=float(Km)/float(10000);
			 		UprOut[ZagrOut].Flag=1; UprOut[ZagrOut].KodKom=2; ZagrOut=0xF &(ZagrOut+1); //выставить флаги для отправки подтверждения
		 		}
	 		}
 		}

 
 	   if (Vbrk>=1) CRS=CRS+XC;
 	   if ((StartPkt==2)&&(Vbrk==1)&&(EnPkt==0))  { StartPkt=0; Vbrk=0; }
 	   if (StartPkt>=2) Vbrk++;
 	   CA++;
    }
	   
   }

// 	   Serial.print("Contents:");
// 	   Serial.println(packetBuffer);
}
 	   
  
  
void getBit(){ //чтение битов и флаги
	tTimeout_1 = 0;
	STT|=0x01; //FLin1Dis=1; датчик включен
		if(index1 < 20){
			if(!digitalRead(dataIn_1)==1){
				xDataBuf1|= 1<<index1;
			}
			} else {
			if (index1==20) //минус
			isfs1=!digitalRead(dataIn_1);
			
			if (index1==23) //дюймы
			isin1=!digitalRead(dataIn_1);
		};

		index1++;
		
		if (index1 >23) { //если слово считано полностью
			xData1=xDataBuf1*10; //дынный Line_2, перевод в микроны
			index1=0;
			xDataBuf1=0;
		};		
}

void getBit_1(){ //чтение битов и флаги
	tTimeout_2 = 0;
	STT|=0x02; //FLin2Dis=1;  датчик включен
		if(index2 < 20){
			if(!digitalRead(dataIn_2)==1){
				xDataBuf2|= 1<<index2;
			}
			} else {
			if (index2==20) //минус
			isfs2=!digitalRead(dataIn_2);
			
			if (index2==23) //дюймы
			isin2=!digitalRead(dataIn_2);
		};

		index2++;
		
		if (index2 >23) { //если слово считано полностью
			xData2=xDataBuf2*10; //дынный Line_2, перевод в микроны
			index2=0;
			xDataBuf2=0;
		};
}

void DataLine() { //Формирование дынных линейных датчиков

	if ((index1!=0)&&(tTimeout_1 > Timeout)) { //обнуление по превышению таймаута Line_1
		index1 = 0;
		xData1 = 0;
		isfs1=0;
		isin1=0;
	};

	if ((index2!=0)&&(tTimeout_2 > Timeout)) { //обнуление по превышению таймаута Line_2
		index2 = 0;
		xData2=0;
		isfs2=0;
		isin2=0;
	};
	
	if ((tTimeout_1 > 200)||(xData1==0)) { //датчик Lin1 отключен		 
		if (tTimeout_1 > 200) {xData1=0; isfs1=0; isin1=0; STT&=0xFE;}
			if (isfs2==1) {xDataS=-1*xData2;} else {xDataS=xData2;}
				if (isin2==1) {xDataS=1.27*xDataS;}	else {xDataS=xDataS;}	 } ;
		
	if ((tTimeout_2 > 200)||(xData2==0)) { //датчик Lin2 отключен
		if (tTimeout_2 > 200) {xData2=0; isfs2=0; isin2=0; STT&=0xFD;}
			if (isfs1==1) {xDataS=-1*xData1;} else {xDataS=xData1;}
				if (isin1==1) {xDataS=1.27*xDataS;}	else {xDataS=xDataS;}	 };
	
	if (((STT&0x03)==0x03) && (xData1!=0) && (xData2!=0))	{ //оба датчика подключены	
		if (isfs1==1) {xDataS1Buf=-1*xData1;} else {xDataS1Buf=xData1;}
			if (isfs2==1) {xDataS2Buf=-1*xData2;} else {xDataS2Buf=xData2;}
						
		if ((isin1==1) && (isin2==1)) {xDataS=(1.27*xDataS1Buf+1.27*xDataS2Buf)/2;}
			if ((isin1==1) && (isin2==0)) {xDataS=(1.27*xDataS1Buf+xDataS2Buf)/2;}
				if ((isin1==0) && (isin2==1)) {xDataS=(xDataS1Buf+1.27*xDataS2Buf)/2;}
					if ((isin1==0) && (isin2==0)) {xDataS=(xDataS1Buf+xDataS2Buf)/2;}  };	
																		
}

void DataAdc(){//----Чтение АЦП - показания давления -----//	
		XPress[CXP]=analogRead(ADCPRSPin);
//		ADCPRS=(XPress[0]+XPress[1]+XPress[2]+XPress[3]+XPress[4]+XPress[5]+XPress[6]+XPress[7]
//				+XPress[8]+XPress[9]+XPress[10]+XPress[11]+XPress[12]+XPress[13]+XPress[14]+XPress[15])/16;
//		CXP=0x07 &(CXP+1);						
		CXP=0x0F &(CXP+1);
		ADCPRSnf=find_similar(XPress, 16, 1); //Поиск макс повторяющегося элемента в массиве		
		if (ADCPRSnf>=Kn){
			ADCPRSnf=(ADCPRSnf-Kn)*Kmf;
		}
		else {ADCPRSnf=0;} 
		ADCPRS=float(1.0-FK)*float(ADCPRS)+float(FK)*float(ADCPRSnf);	//ФНЧ
//		ADCPRS=ADCPRSnf;
		fl_100ms=0;			
}

//**************Поиск макс повторяющегося элемента в массиве****************************
uint16_t find_similar(uint16_t *buf, uint8_t size_buff, uint8_t range) 
{
 uint8_t maxcomp=0; //счётчик максимального колличества совпадений
 uint16_t mcn=0;	//максимально часто встречающийся элемент массива
 uint16_t comp;	//временная переменная
 range++;	//допустимое отклонение

	for (uint8_t i=0; i<size_buff; i++) 
	{
		comp=buf[i];	//кладем элемент массива в comp
		uint8_t n=0;	//счётчик совпадении
		for (uint8_t j=0; j<size_buff; j++)	{ if (buf[j]>comp-range && buf[j]<comp+range) n++;} // ищем повторения элемента comp в массиве buf	
		if (n > maxcomp) //если число повторов больше чем было найдено ранее
		{
			maxcomp=n; //сохраняем счетчик повторов
			mcn=comp; //сохраняем повторяемый элемент
		}		
	}
 return mcn;
}


void Prir(){  // расчет приращения за время условной стабилизации
				 
// Serial.write(0xEE);
  PrirLin[CZagrP]=xDataS;
  Delta=0x1FF & (CZagrP+(0x1FF&(0x1FF^CVbrkP))+1);
// Serial.write(CZagrP);Serial.write(CVbrkP);
  if (Delta>=DINT)
	  {
	  XPLin=PrirLin[CZagrP]-PrirLin[CVbrkP]; Serial.write(XPLin); Serial.write(XPLin>>8);
	  CVbrkP=0x1FF &( CVbrkP+1);
	  FTST=1; STT|=0x10;
//	  Serial.write(0xAA);
	  }
  CZagrP=0x1FF & (CZagrP+1);	
	
}

void PRSAUTOST() { //Автоматический режим стабилизации	
	    	 if (FStart==1) // Автоматический режим включен, нажата кн СТАРТ
	    	 {
		//		 Serial.write(0xFE);
				if ((AUTO_Press[NumStepM].P==0) && (AUTO_Press[NumStepM].T>0)) { // режим замочки грунта
					if (fl_RZ==0) {minutes=0; t_1min=0; fl_RZ=1; STT|=0x20; DINT=AUTO_Press[NumStepM].T+10; }
					if (minutes >= AUTO_Press[NumStepM].T) { //если прошло установленное время перейти к след ступени
						minutes=0; t_1min=0; fl_RZ=0; NumStepM=NumStepM+1; 
						FSetPrsSt=0; FSetPrsSt300=0; STT&=0xDF;// убрать 
						Delta=0; CZagrP=0; CVbrkP=0; FTST=0; STT&=0xEF; XPLin=0xFFF; //убрать
						}
					}
				else {	//основной режим стабилизации	
					if ((AUTO_Press[NumStepM].P>0) && (AUTO_Press[NumStepM].T>0))	// проверка условии испытании
					 {			 
		    		 if (FSetPrsSt==0) { AUTO_Press_ST=AUTO_Press[NumStepM].P; DINT=AUTO_Press[NumStepM].T; FSetPrsSt=1; }   //установка значения PressAUTOST.P(давление стабилизации) И времени усл. стаб.

		    		 if (ADCPRS<=(AUTO_Press_ST-10)) { //задвть ворота давления						 
//						 PORTL=PORTL | B00001000; //PL3 установка диапазона стабилзции давления (открыть впускной)
						 fl_PressUp=1;
						}
		    		 else {PORTL=PORTL & B11110111; fl_PressUp=0;}
		    		 if (ADCPRS>=(AUTO_Press_ST+15)){ //задвть ворота давления						 						 
//						 PORTL=PORTL | B00000010; //PL1 установка диапазона стабилзции давления (открыть выпускной)
						 fl_PressDn=1;
						}						 
		    		 else {PORTL=PORTL & B11111101; fl_PressDn=0;}
						 
		    		 if ((ADCPRS>=(AUTO_Press_ST-10)) && (ADCPRS<=(AUTO_Press_ST+15)) && (FSetPrsSt300==0))	//задать ворота давления
		    		 {
						 if (CUSTPRESS>=300)
						 {
						 	 minutes=0; t_1min=0; FTST=0; STT&=0xEF; fl_1min=1; 
						 	 FSetPrsSt300=1; STT|=0x20; CUSTPRESS=0; Delta=0; CZagrP=0; CVbrkP=0;  XPLin=0xFFF;				    	
			    		 }
			    		 CUSTPRESS++;
		    		 }
		    		 if ((ADCPRS>=(AUTO_Press_ST-10)) && (ADCPRS<=(AUTO_Press_ST+15)) && (-100<=XPLin) && (XPLin<=100)) //задать ворота давления
		    		 {
						 if ((AUTO_Press[NumStepM+1].P==0) && (AUTO_Press[NumStepM+1].T==0)) // окончание испытания
						 {
						 	 FStart=0; STT&=0xBF; FSetPrsSt=0; minutes=0; t_1min=0; fl_PressUp=0; fl_PressDn=0;
							 FlStop=1; //Произвести спуск системы
							 fl_RZ=0; //режим замочки остановлен
							 FSetPrsSt300=0; STT&=0xDF; FTST=0; STT&=0xEF; Delta=0; CZagrP=0; CVbrkP=0;
				    		 XPLin=0xFFF; NumStepM=0;
							 PORTL=PORTL & B11110111;
							 PORTL=PORTL & B11111101;				// закрыть выпускной клапан
						 }
						 else
			    		 {
							 FSetPrsSt=0; FSetPrsSt300=0; STT&=0xDF; NumStepM=NumStepM+1;  //если за время условной стаб приращение <0.1мм то перейти к след ступени							 
							 Delta=0; CZagrP=0; CVbrkP=0; FTST=0; STT&=0xEF; XPLin=0xFFF; minutes=0; t_1min=0;
							 PORTL=PORTL & B11110111; // закрыть впускной клапан
							 PORTL=PORTL & B11111101; // закрыть выпускной клапан
			    		 }
		    		 }
		    		 if (NumStepM>=11) { // окончание испытания
										 FStart=0; STT&=0xBF; FSetPrsSt=0; minutes=0; t_1min=0; fl_PressUp=0; fl_PressDn=0;
										 FlStop=1; //Произвести спуск системы
										 fl_RZ=0; //режим замочки остановлен
						 				 FSetPrsSt300=0; STT&=0xDF; FTST=0; STT&=0xEF; Delta=0; CZagrP=0; CVbrkP=0;
						 				 XPLin=0xFFF; NumStepM=0;
						 				 PORTL=PORTL & B11110111;
										 PORTL=PORTL & B11111101;			// закрыть выпускной клапан
										 }				 
				    }				   
				   else { // окончание испытания
					   FStart=0; STT&=0xBF;  FSetPrsSt=0; minutes=0; t_1min=0; fl_PressUp=0; fl_PressDn=0;
					   FlStop=1; //Произвести спуск системы
					   fl_RZ=0; //режим замочки остановлен
					   FSetPrsSt300=0; STT&=0xDF; FTST=0; STT&=0xEF; Delta=0; CZagrP=0; CVbrkP=0;
					   XPLin=0xFFF; NumStepM=0;
					   PORTL=PORTL & B11110111;
					   PORTL=PORTL & B11111101;			// закрыть выпускной клапан
					   } 
					   
				 }				 

	    	 }	
	if ((0x40 & STT)== 0x00) {minutes=0; t_1min=0;} // Автоматический режим выключен, сброс времени
	if ((0x20 & STT)== 0x00) {minutes=0; t_1min=0;} // Выключен режим стабилизации, сброс времени
				 
	if (NumStepM>=10) { // окончание испытания
		FStart=0; STT&=0xBF; FSetPrsSt=0; minutes=0; t_1min=0; fl_PressUp=0; fl_PressDn=0;
		FlStop=1; //Произвести спуск системы
		fl_RZ=0; //режим замочки остановлен
		FSetPrsSt300=0; STT&=0xDF; FTST=0; STT&=0xEF; Delta=0; CZagrP=0; CVbrkP=0;
		XPLin=0xFFF; NumStepM=0;
		PORTL=PORTL & B11110111;
		PORTL=PORTL & B11111101;			// закрыть выпускной клапан
		}
			 
	if (FlStop==1) {	
		PORTL=PORTL & B11110111; // закрыть впускной клапан	
		PORTL=PORTL | B00000010; //открыть выпускной клапан
		if (ADCPRS<=15)	{ FlStop=0; PORTL=PORTL & B11111101; } // закрыть выпускной клапан		
		}		
							 
	NumStep=NumStepM+1;	
}
