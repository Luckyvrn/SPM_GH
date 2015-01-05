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
0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 0, 176); 
IPAddress ipServer(192, 168, 0, 255); //послать всем

unsigned int localPort = 21666;      //  // номер локального порта для прослушивания

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
unsigned int ADCPRS=0; //измеренное значение давления
unsigned int XPress[8];

byte isin = 0, isin1 = 0, isin2 = 0; //д=1 мм=0
byte isfs = 0, isfs1 = 0,  isfs2 = 0; //минус
byte index1, index2, CXP; //счётчик битов


unsigned int xData1, xData2 , xDataBuf2, xDataBuf1; //новые показания 
int xDataS, xDataS1Buf, xDataS2Buf;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //буфер считанных данных с UDP
unsigned int t_1ms, t_100ms, result;
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

//переменный для предачи на сервер

byte t_1min;
boolean FLin1Dis=0, FLin2Dis=0; //состояние датчиков Lin
byte STT=0x00, NumStep; //статус устройства, NumStep - номер шага.
unsigned int minutes=0, Km=1, Kn=0; // minutes - проидено минут с начала шага, Km и Kn - коэффициенты маштабирования и нуля.

struct TUpr{ unsigned int P, T; };
TUpr AUTO_Press[10];
byte CntKom, CPrm, BufKmd, FStart;
byte *PByte;
byte BufPrm[41];

// переменный для стабилизации

boolean FSetPrsSt, FSetPrsSt300, fl_1min, FTST, fl_RZ; //флаг установки давления стабилизации, флаг окончательной устновки после 300 задержки
byte NumStepM=0;
unsigned int AUTO_Press_ST; // текущее давление стабилизации
unsigned int CZagrP, CVbrkP, Delta, DINT, CUSTPRESS;
int  XPLin=0xFF;
unsigned int PrirLin[512]; // массив для расчета приращений

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
	TCNT1=result;
	t_1ms++;
	t_100ms++;
	tTimeout_1++;
	tTimeout_2++;
	if (tTimeout_1>=251) {tTimeout_1=250;}
	if (tTimeout_2>=251) {tTimeout_2=250;}
	if (t_1ms>=1000) {fl_1s=1; t_1ms=0; t_1min++;}
	if (t_100ms>=100) {fl_100ms=1; t_100ms=0;}	
	if (t_1min>=61) {t_1min=60;}	
	if ((t_1min>=60)&&(FStart==1)) {minutes++; t_1min=0; fl_1min=1;}
}

void loop()
{
DataLine();	//Формирование данных линейных датчиков
PRSAUTOST() ;//Автоматический режим стабилизации
ReadDatUDP(); // Формирование данных приянтых по UDP
OutDatUDP();  //--- Формирование данных для отправки по UDP

if (fl_100ms==1) {
	fl_100ms=0;
	DataAdc(); // Чтение АЦП - показания давления
}

if(fl_1s)
{
	PORTB=PORTB^B10000000; //светодиод D13 мигалка раз в 1 с
	
	fl_1s=0;
	
	if (Rejim==0)
	{
		UprOut[ZagrOut].Flag=1;
		UprOut[ZagrOut].KodKom=1;
		ZagrOut=0xF &(ZagrOut+1);
	}
	if (Rejim==1)
	{
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


void Timer1_init(unsigned long millisecond) 
{
	result=65535-TIMER_CLOCK_FREQ*millisecond/1000000;
	cli(); // Запрещаем все прерывания на время инициализации.
	TCCR1A = 0;	// clear control register A  
	TCCR1B = _BV(CS11); // предделитель на 8   
	TIMSK1|=(1<<TOIE1);
    TCNT1=result;
	sei(); // Закончили инициализацию, разрешаем все прерывания.
}		

void OutDatUDP()
{
	if (UprOut[VbrkOut].Flag==1)
	  {

		if (UprOut[VbrkOut].KodKom==1)
		{
			BufOut[2]=0x01;     //--- Код Пакета (Команда)
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
			BufOut[5]=0x10;      //--- Длина Информационной Части
			BufOut[6]=isin1;     //--- Признак системы измерения
			BufOut[7]=isfs1;     //--- Знак Числа
			BufOut[8]=xData1;    //--- Значение измерения (Младшая часть)
			BufOut[9]=xData1>>8; //--- Значение измерения (Старшая часть)
			BufOut[10]=isin2;     //--- Признак системы измерения
			BufOut[11]=isfs2;     //--- Знак Числа
			BufOut[12]=xData2;    //--- Значение измерения (Младшая часть)
			BufOut[13]=xData2>>8; //--- Значение измерения (Старшая часть)
			BufOut[14]=xDataS;    //--- Значение среднее (Младшая часть)
			BufOut[15]=xDataS>>8; //--- Значение среднее (Старшая часть)						
			BufOut[16]=ADCPRS;     //--- АЦП (Младшая часть)
			BufOut[17]=ADCPRS>>8;     //--- АЦП (Старшая часть)
			BufOut[18]=minutes;     //--- минуты (Младшая часть)
			BufOut[19]=minutes>>8;  //--- минуты (Старшая часть)	
			BufOut[20]=NumStep;     // номер шага										
			BufOut[21]=STT;		// ---статус устройств
			LenBuf=21;
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
 		   
 		if ((KodPkt==0x01)&&(Vbrk>=1)) //Обработка по приему пакета "Подтверждение регистрации усройства на сервере" 0x02
 		{
			if (Vbrk==1) EnPkt=1;
			if (Vbrk==5) BufKmd=XC;
	 		if (Vbrk==6)
	 		{
		 		if (CRS==XC){ //проверка контрольной суммы
//			 		Serial.write(CRS);
//			 		Serial.write(XC);
                    if (BufKmd==0x42) //передать загруженные команды управления и флаги состояния устройства
					{
			 		StartPkt=0; Vbrk=0; EnPkt=0;
			 		UprOut[ZagrOut].Flag=1; UprOut[ZagrOut].KodKom=2; ZagrOut=0xF &(ZagrOut+1); //выставить флаги для отправки подтверждения
			 		UprOut[ZagrOut].Flag=1; UprOut[ZagrOut].KodKom=4; ZagrOut=0xF &(ZagrOut+1); //выставить флаги для отправки подтверждения
					}
                    if (BufKmd==0x43)		// начать испытание
                    {
			 		StartPkt=0; Vbrk=0; EnPkt=0;
			 		UprOut[ZagrOut].Flag=1; UprOut[ZagrOut].KodKom=2; ZagrOut=0xF &(ZagrOut+1); //выставить флаги для отправки подтверждения
                    FStart=1; t_1min=0; STT|=0x40;
                    }
                    if (BufKmd==0x44)		// закончить испытание
                    {
	                    StartPkt=0; Vbrk=0; EnPkt=0;
	                    UprOut[ZagrOut].Flag=1; UprOut[ZagrOut].KodKom=2; ZagrOut=0xF &(ZagrOut+1); //выставить флаги для отправки подтверждения
	                    FStart=0; STT&=0xBF;
						NumStepM=0; minutes=0;
                    }
					 if (BufKmd==0x45)		// перерегестрация устройства на сервере
					 {
						 StartPkt=0; Vbrk=0; EnPkt=0; Rejim=0; 
						 UprOut[ZagrOut].Flag=1; UprOut[ZagrOut].KodKom=2; ZagrOut=0xF &(ZagrOut+1); //выставить флаги для отправки подтверждения
					 }
		 		}
	 		}
 		}

 		   if ((KodPkt==0x02)&&(Vbrk>=1)) //Обработка по приему пакета "Команды управления" 0x02
 		   {
 			   if (Vbrk==1) EnPkt=1;
 			   if (Vbrk>=5)
 			   {
 				   BufPrm[CPrm]=XC;
 				   if ((CRS==XC)&&(CPrm==LenPkt))
 				   { //проверка контрольной суммы
// 					   Serial.write(CRS);
// 					   Serial.write(XC);
 					   PByte=(byte*) &AUTO_Press[0];
 					   CA=0;
 					   while(CA<LenPkt)
 					   {
 						   *PByte=BufPrm[CA]; PByte++;
 						   CA++;
 					   }
                       CntKom=CPrm >> 2;
 					   StartPkt=0; Vbrk=0; EnPkt=0;
 					   UprOut[ZagrOut].Flag=1; UprOut[ZagrOut].KodKom=2; ZagrOut=0xF &(ZagrOut+1); //выставить флаги для отправки подтверждения принятия команд управления
 			      }
			    CPrm++;
 		    }
 	  	 }

 		if ((KodPkt==0x82)&&(Vbrk>=1)) //Обработка по приему пакета "Подтверждение регистрации усройства на сервере" 0x02
 		 {
 			if (Vbrk==1) EnPkt=1;
			if (Vbrk==5) IPS0=XC;
			if (Vbrk==6) IPS1=XC;
			if (Vbrk==7) IPS2=XC;
			if (Vbrk==8) {IPS3=XC;  ipServer[0]=IPS0; ipServer[1]=IPS1; ipServer[2]=IPS2; ipServer[3]=IPS3;} //установка рефльного IP adres server  
 			if (Vbrk==9)
 			 {		
 			   if (CRS==XC){ //проверка контрольной суммы
//		    		Serial.write(CRS);
//				Serial.write(XC);
				Rejim=1; 
 				StartPkt=0; Vbrk=0; EnPkt=0;	
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
			xData1=xDataBuf1; //дынный Line_2
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
			xData2=xDataBuf2; //дынный Line_2
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
				if (isin2==1) {xDataS=12.7*xDataS;}	else {xDataS=10*xDataS;}	 } ;
		
	if ((tTimeout_2 > 200)||(xData2==0)) { //датчик Lin2 отключен
		if (tTimeout_2 > 200) {xData2=0; isfs2=0; isin2=0; STT&=0xFD;}
			if (isfs1==1) {xDataS=-1*xData1;} else {xDataS=xData1;}
				if (isin1==1) {xDataS=12.7*xDataS;}	else {xDataS=10*xDataS;}	 };
	
	if (((STT&0x03)==0x03) && (xData1!=0) && (xData2!=0))	{ //оба датчика подключены	
		if (isfs1==1) {xDataS1Buf=-1*xData1;} else {xDataS1Buf=xData1;}
			if (isfs2==1) {xDataS2Buf=-1*xData2;} else {xDataS2Buf=xData2;}
						
		if ((isin1==1) && (isin2==1)) {xDataS=(12.7*xDataS1Buf+12.7*xDataS2Buf)/2;}
			if ((isin1==1) && (isin2==0)) {xDataS=(12.7*xDataS1Buf+10*xDataS2Buf)/2;}
				if ((isin1==0) && (isin2==1)) {xDataS=(10*xDataS1Buf+12.7*xDataS2Buf)/2;}
					if ((isin1==0) && (isin2==0)) {xDataS=(10*xDataS1Buf+10*xDataS2Buf)/2;}  };	
																		
}

void DataAdc(){//----Чтение АЦП - показания давления -----//	
		XPress[CXP]=analogRead(ADCPRSPin);
		ADCPRS=(XPress[0]+XPress[1]+XPress[2]+XPress[3]+XPress[4]+XPress[5]+XPress[6]+XPress[7])/8;
		ADCPRS=(ADCPRS-Kn)*Km;
		CXP=0x07 &(CXP+1);
		fl_100ms=0;			
}

void Prir(){  // расчет приращения за время условной стабилизации
				 
// Serial.write(0xEE);
  PrirLin[CZagrP]=xDataS;
  Delta=0x1FF & (CZagrP+(0x1FF&(0x1FF^CVbrkP))+1);
// Serial.write(CZagrP);Serial.write(CVbrkP);
  if (Delta>=DINT)
	  {
	  XPLin=PrirLin[CZagrP]-PrirLin[CVbrkP]; //Serial.write(XPLin); Serial.write(XPLin>>8);
	  CVbrkP=0x1FF &( CVbrkP+1);
	  FTST=1;
//	  Serial.write(0xAA);
	  }
  CZagrP=0x1FF & (CZagrP+1);	
	
}

void PRSAUTOST() { //Автоматический режим стабилизации	
	    	 if (FStart==1) // Автоматический режим включен, нажата кн СТАРТ
	    	 {
		//		 Serial.write(0xFE);
				if ((AUTO_Press[NumStepM].P==0) && (AUTO_Press[NumStepM].T>0)) { // режим замочки грунта
					if (fl_RZ==0) {minutes=0; t_1min=0; fl_RZ=1;}
					if (minutes >= AUTO_Press[NumStepM].T) { //если прошло установленное время перейти к след ступени
						minutes=0; t_1min=0; fl_RZ=0;NumStepM=NumStepM+1; 
						FSetPrsSt=0; FSetPrsSt300=0; // убрать 
						Delta=0; CZagrP=0; CVbrkP=0; FTST=0; XPLin=0xFF; //убрать
						}
					}
				else {	//основной режим стабилизации	
					if ((AUTO_Press[NumStepM].P>0) && (AUTO_Press[NumStepM].T>0))	// проверка условии испытании
					 {			 
		    		 if (FSetPrsSt==0) { AUTO_Press_ST=AUTO_Press[NumStepM].P; DINT=AUTO_Press[NumStepM].T; FSetPrsSt=1; }   //установка значения PressAUTOST.P(давление стабилизации) И времени усл. стаб.
//					 Serial.write(0xFF); Serial.write(AUTO_Press_ST); Serial.write(AUTO_Press_ST>>8); Serial.write(DINT); Serial.write(DINT>>8);
		    		 if (ADCPRS<=(AUTO_Press_ST-15)) {						 
						 PORTL=PORTL | B00001000; //PL3 установка диапазона стабилзции давления (открыть впускной)
//						 fl_PressUp=1;
					 }
		    		 else {PORTL=PORTL & B11110111;}
		    		 if (ADCPRS>=(AUTO_Press_ST+25)) PORTL=PORTL | B00000010; //PL1 установка диапазона стабилзции давления (открыть выпускной)
		    		 else {PORTL=PORTL & B11111101;}
		    		 if ((ADCPRS>=(AUTO_Press_ST-15)) && (ADCPRS<=(AUTO_Press_ST+25)) && (FSetPrsSt300==0))
		    		 {
						 if (CUSTPRESS>=300)
						 {
						 	 minutes=0; t_1min=0; FTST=0; fl_1min=1; 
						 	 FSetPrsSt300=1; CUSTPRESS=0; Delta=0; CZagrP=0; CVbrkP=0;  XPLin=0xFF;				    	
			    		 }
			    		 CUSTPRESS++;
		    		 }
		    		 if ((ADCPRS>=(AUTO_Press_ST-15)) && (ADCPRS<=(AUTO_Press_ST+25)) && (-10<=XPLin) && (XPLin<=10))
		    		 {
						 if ((AUTO_Press[NumStepM+1].P==0) && (AUTO_Press[NumStepM+1].T==0)) // окончание испытания
						 {
						 	 FStart=0; STT&=0xBF; FSetPrsSt=0; minutes=0; t_1min=0;
							 FSetPrsSt300=0; FTST=0; Delta=0; CZagrP=0; CVbrkP=0;
				    		 XPLin=0xFF; 
							 PORTL=PORTL & B11110111;
							 PORTL=PORTL & B11111101;
						 }
						 else
			    		 {
							 FSetPrsSt=0; FSetPrsSt300=0; NumStepM=NumStepM+1;  //если за время условной стаб приращение <0.1мм то перейти к след ступени
							 Delta=0; CZagrP=0; CVbrkP=0; FTST=0; XPLin=0xFF; minutes=0; t_1min=0;
							 PORTL=PORTL & B11110111;
							 PORTL=PORTL & B11111101;
			    		 }
		    		 }
		    		 if (NumStepM>=11) { // окончание испытания
										 FStart=0; STT&=0xBF; FSetPrsSt=0; minutes=0; t_1min=0;
						 				 FSetPrsSt300=0; FTST=0; Delta=0; CZagrP=0; CVbrkP=0;
						 				 XPLin=0xFF; NumStepM=0;
						 				 PORTL=PORTL & B11110111;
						 				 PORTL=PORTL & B11111101;}				 
				    }				   
				   else { // окончание испытания
					   FStart=0; STT&=0xBF;  FSetPrsSt=0; minutes=0; t_1min=0;
					   FSetPrsSt300=0; FTST=0; Delta=0; CZagrP=0; CVbrkP=0;
					   XPLin=0xFF; NumStepM=0;
					   PORTL=PORTL & B11110111;
					   PORTL=PORTL & B11111101;} 
				 }

	    	 }	
	NumStep=NumStepM+1;	
}
