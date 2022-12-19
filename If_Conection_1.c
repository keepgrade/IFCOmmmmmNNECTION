//Header File config
/********************************************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#include <util/delay.h>

//Define value
#define m_sec 100

//Mobility Status Data - It is used as an interface and check bit.
/********************************************************************************************************/
volatile char emergency_button  = 0;
volatile char door_status       = 0;
volatile char air_con_status    = 0;
volatile char rgb_led_status    = 0;
volatile char clcd_status       = 0;
volatile char passenger         = 0;
volatile char motion_detect_val = 0;

//Message transmission using ASCII code
/********************************************************************************************************/

//There was an emergency. Please check Mobility IF-1.
volatile char emergency_message[51] = {0x54, 0x68, 0x65, 0x72, 0x65, 0x20, 0x77, 0x61, 0x73, 0x20, 0x61, 0x6e, 0x20, 0x65, 
0x6d, 0x65, 0x72, 0x67, 0x65, 0x6e, 0x63, 0x79, 0x2E, 0x20, 0x50, 0x6C, 0x65, 0x61, 0x73, 0x65, 0x20, 0x63, 0x68, 0x65, 0x63, 
0x6B, 0x20, 0x4D, 0x6F, 0x62, 0x69, 0x6C, 0x69, 0x74, 0x79,0x20,0x49,0x46,0x2D,0x31,0x2E};

//The occupant is not moving. Please check Mobility IF-1.
volatile char not_moving_message[55] = {0x54, 0x68, 0x65, 0x20, 0x6F, 0x63, 0x63, 0x75, 0x70, 0x61, 0x6E, 0x74, 0x20, 0x69,
0x73, 0x20, 0x6e, 0x6f, 0x74, 0x20, 0x6d, 0x6f, 0x76, 0x69, 0x6e, 0x67, 0x2e, 0x20, 0x50, 0x6c, 0x65, 0x61, 0x73, 0x65, 0x20,
0x63, 0x68, 0x65, 0x63, 0x6b, 0x20, 0x4d, 0x6f, 0x62, 0x69, 0x6c, 0x69, 0x74, 0x79, 0x20, 0x49, 0x46, 0x2d, 0x31, 0x2e};

//Motion is detected on IF-1. Please check Mobility IF-1.
volatile char motion_detected_message[55] = {0x4d, 0x6f, 0x74, 0x69, 0x6f, 0x6E, 0x20, 0x69, 0x73, 0x20, 0x64,  0x65, 0x74,
0x65,  0x63, 0x74, 0x65, 0x64, 0x20, 0x6f, 0x6e, 0x20, 0x49, 0x46, 0x2d, 0x31, 0x2e, 0x20, 0x50, 0x6c, 0x65, 0x61, 0x73, 0x65, 0x20,
0x63, 0x68, 0x65, 0x63, 0x6b, 0x20, 0x4d, 0x6f, 0x62, 0x69, 0x6c, 0x69, 0x74, 0x79, 0x20, 0x49, 0x46, 0x2d, 0x31, 0x2e};



//Function Declaration
/********************************************************************************************************/
void CLCD_Init( );
void CLCD_write_instruction(unsigned char data);
void CLCD_write_data(unsigned char data);
void CLCD_door_get_in();
void CLCD_door_get_out();
void CLCD_door_open();
void CLCD_door_close();
void CLCD_led_On();
void CLCD_led_Off();
void CLCD_air_con_on();
void CLCD_air_con_off();
void RGB_LED_setColor(int r,int g,int b);
void RGB_LED_init();
void RGB_LED_output();
void Servo_open_door();
void Servo_close_door();
void Init_interrupt(void);
void Init_Timer(void);
void Init_PWM(void);
void Pin_Config(void);
void Bluetooth_Config();
void Sensor_module_Config();

//Use Interrupt
//UART Interrupt Func - connectivity
/********************************************************************************************************/

//use index
volatile char idx  = 0;

// Periodic vehicle status information transmission and motion detection
ISR(TIMER1_OVF_vect)
{
   cli();
   //Set timer cycle
   TCNT1H = 0xC2;
   TCNT1L = 0xF6;

   emergency_button  = 0;

   //PIR(Passive Infrared Sensor) Func -Motion detection
   /*****************************************************************************************************/
   motion_detect_val = 0;

   if(PINF & 0x01) // PF0에 값이 들어오면
   {
      motion_detect_val = 1;
   }
   else
   {
      motion_detect_val = 0;
   }

   //탑승자가 탑승했지만 모션이 감지되지 않을때
   if(passenger ==1 && motion_detect_val == 0)
   {  
      for(idx = 0; idx <55; idx++)
      {
         while(!(UCSR1A& 0x20));
         UDR1 = not_moving_message[idx];
      }

   }
   //탑승자가 탑승하지 않았지만 모션이 감지 될때
   else if(passenger ==0 && motion_detect_val == 1)
   {  
      for(idx = 0; idx <55; idx++)
      {
         while(!(UCSR1A& 0x20));
         UDR1 = motion_detected_message[idx];
      }

   }
   //모빌리티의 정보 보내주기
   else
   {
      for(idx = 0; idx < 2; idx++)
      {
         while(!(UCSR1A & 0x20));
         UDR1 = door_status;  //Mobility status 변수 지정 필요 
      }
   }

   sei();
}

ISR(SIG_UART1_RECV){
   cli();

   while(!(UCSR1A & 0x20)); //Data is Empty?
   
   if(UDR1==0x00){ // Door Close
         CLCD_door_close();
         Servo_close_door();        
         door_status = 0;
      }
   else if(UDR1 == 0x01){ // Door Open
         CLCD_door_open();
         Servo_open_door();
         door_status = 1;
      }
   else if(UDR1 == 0x10) // Air con OFF
      {
         //모터를 통한 선풍기
       CLCD_air_con_off();
         Servo360_aircon_off();
         air_con_status = 0;
      }
   else if(UDR1 == 0x11) // Air con ON
      {
         //모터를 통한 선풍기
       CLCD_air_con_on();
         Servo360_aircon_on();
         air_con_status = 1;
      }
   else if(UDR1 == 0x20) // RGB_LED OFF
      {
         rgb_led_status = 0;
         RGB_LED_output();
         CLCD_led_Off();
      }
   else if(UDR1 == 0x21) // RGB_LED White
      {
         rgb_led_status = 1;
         RGB_LED_output();
         CLCD_led_On();
      }
   else if(UDR1 == 0x22) // RGB_LED RED
      {
         rgb_led_status =2;
         RGB_LED_output();
         CLCD_led_On();
      }
   else if(UDR1 == 0x23) // RGB_LED Green
      {
         rgb_led_status = 3;
         RGB_LED_output();
         CLCD_led_On();
      }
   else if(UDR1 == 0x24) // RGB_LED Blue
      {
         rgb_led_status = 4;
         RGB_LED_output();
         CLCD_led_On();
      }
   else if(UDR1 == 0x25) // RGB_LED Violet
      {
         rgb_led_status = 5;
         RGB_LED_output();
         CLCD_led_On();
      }
   else if(UDR1 == 0x30) // CLCD OFF
      {
         CLCD_Init();
         clcd_status = 0;
      }
   else if(UDR1 == 0xF0) // 내림
      {
         CLCD_get_out();
         passenger =0;
      }
   else if(UDR1 == 0xF1) // 탑승
      {
         CLCD_get_in();
         passenger =1;
      }
            
   sei();
}

//Button Func - emergency button
/********************************************************************************************************/

ISR(INT0_vect) { 

cli();

emergency_button ++;
   if(emergency_button == 1){

      for(idx = 0; idx <51; idx++){
         
         while(!(UCSR1A& 0x20));
         UDR1 = emergency_message[idx];
      }
   }

sei();

}

//CLCD Func - Display
/********************************************************************************************************/

//CLCD config
#define ENABLE  PORTE |= 0x01;
#define DISABLE PORTE &= ~0x01;
#define RS_SET  PORTE |= 0x04;
#define RS_CLI  PORTE &= ~0x04;
#define RW_SET  PORTE |= 0x02;
#define RW_CLI  PORTE &= ~0x02;

//CLCD Init
void CLCD_Init( ){
 
_delay_ms(5);
CLCD_write_instruction(0x30);
_delay_ms(5);
CLCD_write_instruction(0x30);
_delay_ms(5);
CLCD_write_instruction(0x30);
_delay_ms(5);
CLCD_write_instruction(0x38);
_delay_ms(5);
CLCD_write_instruction(0x08);
_delay_ms(5);
CLCD_write_instruction(0x01);
_delay_ms(5);
CLCD_write_instruction(0x06);
_delay_ms(5);
CLCD_write_instruction(0x0C);
_delay_ms(5);
}

//CLCD Location
void CLCD_write_instruction(unsigned char data){

RS_CLI
RW_CLI

ENABLE
_delay_us(1);
PORTC = data;
_delay_us(1);
DISABLE

}

//CLCD Data write
void CLCD_write_data(unsigned char data){

RS_SET
RW_CLI
ENABLE
PORTC = data;
_delay_us(1);
DISABLE
_delay_us(1);

_delay_ms(10);

}

void CLCD_get_in(){
   
   CLCD_Init();
   //Welcome
   //Have a nice ride!
   CLCD_write_instruction(0x84); // 위치 바꾸기 
    
   CLCD_write_data('W');
   CLCD_write_data('W'); 
   CLCD_write_data('e'); 
   CLCD_write_data('l'); 
   CLCD_write_data('c'); 
   CLCD_write_data('o'); 
   CLCD_write_data('m'); 
   CLCD_write_data('e'); 

   CLCD_write_instruction(0xC0); // 위치 바꾸기 

   CLCD_write_data('H');
   CLCD_write_data('H'); 
   CLCD_write_data('a'); 
   CLCD_write_data('v'); 
   CLCD_write_data('e');
   CLCD_write_data('_');
   CLCD_write_data('a');
   CLCD_write_data('_');
   CLCD_write_data('n'); 
   CLCD_write_data('i'); 
   CLCD_write_data('c');
   CLCD_write_data('e');
   CLCD_write_data('_');
   CLCD_write_data('D'); 
   CLCD_write_data('a'); 
   CLCD_write_data('y');

}

void CLCD_get_out(){
   
   CLCD_Init();

   //Bye
   //Thank you!
   CLCD_write_instruction(0x86); // 위치 바꾸기 
   
   CLCD_write_data('B');
   CLCD_write_data('B'); 
   CLCD_write_data('y'); 
   CLCD_write_data('e'); 

   CLCD_write_instruction(0xC3); // 위치 바꾸기 

   CLCD_write_data('T');
   CLCD_write_data('T'); 
   CLCD_write_data('h'); 
   CLCD_write_data('a'); 
   CLCD_write_data('n');
   CLCD_write_data('k');
   CLCD_write_data('_'); 
   CLCD_write_data('y'); 
   CLCD_write_data('o');
   CLCD_write_data('u');
   CLCD_write_data('!');

}

void CLCD_door_open(){
   
   CLCD_Init();
   //Welcome
   //Have a nice ride!
   CLCD_write_instruction(0x84); // 위치 바꾸기 
    
   CLCD_write_data('D');
   CLCD_write_data('D'); 
   CLCD_write_data('o'); 
   CLCD_write_data('o'); 
   CLCD_write_data('r'); 

   CLCD_write_instruction(0xC0); // 위치 바꾸기 

   CLCD_write_data('O');
   CLCD_write_data('O'); 
   CLCD_write_data('p'); 
   CLCD_write_data('e'); 
   CLCD_write_data('n');

}

void CLCD_door_close(){
   
   CLCD_Init();

   //Bye
   //Thank you!
   CLCD_write_instruction(0x86); // 위치 바꾸기 
   
   CLCD_write_data('D');
   CLCD_write_data('D'); 
   CLCD_write_data('o'); 
   CLCD_write_data('o'); 
   CLCD_write_data('r'); 

   CLCD_write_instruction(0xC3); // 위치 바꾸기 

   CLCD_write_data('C');
   CLCD_write_data('C'); 
   CLCD_write_data('l'); 
   CLCD_write_data('o'); 
   CLCD_write_data('s');
   CLCD_write_data('e');

}

void CLCD_led_On(){
   
   CLCD_Init();
   
   //LED ON
    CLCD_write_instruction(0x85); // 위치 바꾸기 
 
    
    CLCD_write_data('L');
	CLCD_write_data('L'); 
    CLCD_write_data('E'); 
    CLCD_write_data('D'); 
    CLCD_write_data('-'); 
    CLCD_write_data('O'); 
    CLCD_write_data('N'); 

}

void CLCD_led_Off(){
   
   CLCD_Init();
   
   //LED OFF
    CLCD_write_instruction(0x85); // 위치 바꾸기 
 

   CLCD_write_data('L'); 
   CLCD_write_data('L');
   CLCD_write_data('E'); 
   CLCD_write_data('D'); 
   CLCD_write_data('-'); 
   CLCD_write_data('O'); 
   CLCD_write_data('F'); 
   CLCD_write_data('F'); 

}

void CLCD_air_con_on(){
   
   CLCD_Init();

   //Air Con On
    CLCD_write_instruction(0x83); // 위치 바꾸기 
 
    
   CLCD_write_data('A'); 
   CLCD_write_data('A');
   CLCD_write_data('i'); 
   CLCD_write_data('r'); 
   CLCD_write_data('c'); 
   CLCD_write_data('o'); 
   CLCD_write_data('n'); 
   CLCD_write_data('-'); 
   CLCD_write_data('O'); 
   CLCD_write_data('n'); 

}

void CLCD_air_con_off(){

   CLCD_Init();
   //Air Con Off
   
    CLCD_write_instruction(0x83); // 위치 바꾸기 
 
   CLCD_write_data('A');
   CLCD_write_data('A'); 
   CLCD_write_data('i'); 
   CLCD_write_data('r'); 
   CLCD_write_data('c'); 
   CLCD_write_data('o'); 
   CLCD_write_data('n'); 
   CLCD_write_data('-'); 
   CLCD_write_data('O'); 
   CLCD_write_data('f'); 
   CLCD_write_data('f'); 
}

//RGB LED Func -ambient light
/********************************************************************************************************/

void RGB_LED_setColor(int r,int g,int b){
OCR3A=r;
OCR3B=g;
OCR3C=b;
}

void RGB_LED_init(){   // LED_Off

   RGB_LED_setColor(0,0,0); 

}

void RGB_LED_output(){ // led 색깔 전체 모아두고 rgb_led_status 값에 따라 색 변화

   if( rgb_led_status == 0)// 흰색의 16진수
      RGB_LED_setColor(0,0,0);
   else if( rgb_led_status == 1) //LED_ON
      RGB_LED_setColor(255,255,255);
   else if( rgb_led_status == 2) //LED_RED
      RGB_LED_setColor(255,0,0);
   else if( rgb_led_status == 3) //LED_Green
      RGB_LED_setColor(0,255,0);
   else if( rgb_led_status == 4) //LED_Blue
      RGB_LED_setColor(0,0,255);
   else if( rgb_led_status == 5) //LED_Violet
      RGB_LED_setColor(255,0,255); 

}

//Motor Func - airconditioner
/********************************************************************************************************/
void Servo360_aircon_on(){

   OCR0 = 0x28; // 

   }

void Servo360_aircon_off(){

   OCR0 = 0x00;
   }

//Servo Motor Func - Door
/********************************************************************************************************/
void Servo_open_door(){

   OCR1A = 40; // 90도 문 열기
   _delay_ms(1000);

   }

void Servo_close_door(){

   OCR1A = 25; // 0도 문 닫기
   _delay_ms(1000);
   }

//MCU Init
/********************************************************************************************************/
void Init_interrupt(void){
   //Interrupt Set
   cli();
   EICRA = (0<<ISC31) | (0<<ISC30) | (0<<ISC21) | (0<<ISC20) | (1<<ISC11) | (0<<ISC11) | (0<<ISC01) | (0<<ISC00); // Init Interrupt "Rising Edge"
   EIMSK = (0<<INT7) | (0<<INT6) | (0<<INT5) | (0<<INT4) | (0<<INT3) | (0<<INT2) | (0<<INT1) | (1<<INT0);           // Button Interrupt use '0
   sei();
}

void Init_Timer(void){
   //Timer Set
   cli();
    TIMSK  = 0x04; //use interupput
    TCCR1A = 0x82; // OC1A
    TCCR1B = 0x1D; //prescail 1024
    TCNT1H = 0xFF; //
    TCNT1L = 0xFE; //
    TCCR3A=0xFE;
    TCCR3B=0x1A;

    sei();
}

void Init_PWM(void){
   //Past PWM Set, prescale 1024 
   cli();
   TCCR0 = (0<<FOC0) | (1<<WGM00) | (1<<COM01) | (0<<COM00) | (1<<WGM01) | (1<<CS02) | (1<<CS01) | (1<<CS00);
   
   sei();
}

void Pin_Config(void){
   //Pin Cofig
   cli();
    DDRA = 0x07; //PA0~2    Motor Driver
    DDRB = 0x30; //PB5      Servo Motor      4 360servo 
    DDRC = 0xFF; //PC0~7    CLCD output
    DDRD = 0x0A; //PD0      Button           2~3   Bluetooth // 블루투스는 설정할 필요 없나봄
    DDRE = 0x3F; //PE0~2    CLCD             3~5   RGL_LED
    DDRF = 0x00; //PF0      PIR              1     FSR   
   sei();
}

void Bluetooth_Config(){
   //Bluetooth 
   //RX TX 1
   cli();
   UCSR1C = 0x07; // parity 사용 x, 8비트사용
   UCSR1B = 0x98; // TX + RX 둘다 사용
   UBRR1L = 0x67; // baud rate 103으로 설정 (9600)
   sei();

}

void Sensor_module_Config(){
   //CLCD Set

   //RGB LED Set
   PORTE |= 0x0F; //CLCD out 0~2 , Servo 3
   ICR3 = 255; //RGB_LED Range (top)

   //PIR Set
    PORTF = 0x01; //PIR 0 pin Use

   //FSR Set 

   //Servo motor 
   ICR1 = 311; // top

}

void Set_main()
{
   cli();
   //Init
   Init_interrupt();
   Init_Timer();
   Init_PWM();
//Cofig
   Pin_Config();
   Bluetooth_Config();
   Sensor_module_Config();
//
   CLCD_Init(); 
   sei();
}

//Main
/********************************************************************************************************/
int main()
{
   cli();

   Set_main();

   sei();
   do{

   }while(1);

}