#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/portpins.h>
#include <avr/pgmspace.h>

//FreeRTOS include files
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"
#include "usart_ATmega1284.h"

unsigned char arr[] = {0x01, 0x03, 0x02, 0x06, 0x04, 0x0C, 0x08, 0x09};
#define multiplier(x) x*64/5.625
#define uart_port_0 0
#define threshold_open 180
#define threshold_close 50
#define motor_rh PINC
#define motor_lh PORTC
#define motor_garage PORTB
#define simu_light_rh PIND
#define simu_light_lh PORTD
unsigned char bluetooth_receive = 0x00;
unsigned short phototransistor_input_0;
unsigned short phototransistor_input_1;

void A2D_init() {
    ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADATE);
    // ADEN: Enables analog-to-digital conversion
    // ADSC: Starts analog-to-digital conversion
    // ADATE: Enables auto-triggering, allowing for constant
    //      analog to digital conversions.
}

// Pins on PORTA are used as input for A2D conversion
//    The default channel is 0 (PA0)
// The value of pinNum determines the pin on PORTA
//    used for A2D conversion
// Valid values range between 0 and 7, where the value
//    represents the desired pin for A2D conversion
void Set_A2D_Pin(unsigned char pinNum) {
    ADMUX = (pinNum <= 0x07) ? pinNum : ADMUX;
    // Allow channel to stabilize
    static unsigned char i = 0;
    for ( i=0; i<15; i++ ) { asm("nop"); }
}

//-------------------------BLUETOOTH State Machine-------------------
enum bluetooth_tick{bluetooth_wait} bluetooth_state;

void BLUETOOTH_Init() {
    bluetooth_state = bluetooth_wait;
}

void BLUETOOTH_Tick() {
    switch(bluetooth_state) {
        case bluetooth_wait:
        default:
            if(USART_HasReceived(uart_port_0)){
                bluetooth_receive = USART_Receive(uart_port_0);//unsigned char
                USART_Flush(uart_port_0);
            }
            break;
    }
}

void BLUETOOTH_Task() {
    BLUETOOTH_Init();
    for(;;)
    {
        BLUETOOTH_Tick();
        vTaskDelay(2);
    }
}
//-------------------------DOOR State Machine------------------------
enum door_tick{door_wait, door_enter} door_state;

void DOOR_Init() {
    door_state = door_wait;
}

void DOOR_Tick() {
    static unsigned char dir_flag = 0x00;
    static unsigned char dir = 0;
    static unsigned int count = 0;
    static unsigned int mult = 180;

    switch(door_state) {
        case door_enter:
            if (dir_flag == 0x02) { //open
                if(dir<=0)dir = 7;
                else dir -=1;

            }
            else { //close
                if (dir < 7) dir = dir + 1;
                else dir = 0;

            }

            count++;
            if(count>=multiplier(mult)){
                count = 0;
                door_state = door_wait;
            }

            //first 4 ports
            //mask
            motor_lh = (arr[dir] | 0xF0) & motor_rh;
            break;
        case door_wait:
            if(bluetooth_receive == 0x32) { //open
                if (dir_flag != 0x01) {
                    dir_flag = 0x01;
                    door_state = door_enter;
                }
            }
            else if (bluetooth_receive == 0x33) {//close
                if (dir_flag != 0x02) {
                    dir_flag = 0x02;
                    door_state = door_enter;
                }
            }
        default:
            break;
    }
}

void DOOR_Task() {
    DOOR_Init();
    for(;;)
    {
        DOOR_Tick();
        vTaskDelay(2);
    }
}
//-------------------------PHOTO-TRANSIT State Machine-------------------
enum photo_transit_tick {wait_photo_transit} photo_transit_state;

void PHOTO_TRANSIT_Init() {
    photo_transit_state = wait_photo_transit;
}

void PHOTO_TRANSIT_Tick() {
    switch(photo_transit_state) {
        case wait_photo_transit:
        default:
            Set_A2D_Pin(0);
            phototransistor_input_0 = ADC;
            Set_A2D_Pin(1);
            phototransistor_input_1 = ADC;
            break;
    }
}

void PHOTO_TRANSIT_Task() {
    PHOTO_TRANSIT_Init();
    for(;;)
    {
        PHOTO_TRANSIT_Tick();
        vTaskDelay(100);
    }
}
//-------------------------GARAGE DOOR State Machine-------------------
enum garage_tick{garage_wait, garage_enter} garage_state;

void GARAGE_Init() {
    garage_state = garage_wait;
}

void GARAGE_Tick() {
  static unsigned char dir_flag = 0x00;
  static unsigned char dir = 0;
  static unsigned int count = 0;

  switch(garage_state) {
      case garage_enter:
          if (dir_flag == 0x01) {//close
              if(dir<=0)dir = 7;
              else dir -=1;

              if (phototransistor_input_1 <= threshold_close) {
                  garage_state = garage_wait;
              }
          }
          else { //open
              if (dir < 7) dir = dir + 1;
              else dir = 0;


              if (phototransistor_input_0 > threshold_open) {
                  garage_state = garage_wait;
              }
          }
          PORTB = arr[dir];
          break;
      case garage_wait:
          if(bluetooth_receive == 0x34){ //open
              if (dir_flag != 0x01) {
                  dir_flag = 0x01;
                  garage_state = garage_enter;
              }
          }
          else if (bluetooth_receive == 0x35) { //close
              if (dir_flag != 0x02) {
                  dir_flag = 0x02;
                  garage_state = garage_enter;
              }
          }
      default:
          break;
  }
}

void GARAGE_Task() {
    GARAGE_Init();
    for(;;)
    {
        GARAGE_Tick();
        vTaskDelay(2);
    }
}
//---------------------LIGHT State Machine-----------------------
enum light_tick{light_wait, light_enter} light_state;

void LIGHT_Init() {
    light_state = light_wait;
}

void LIGHT_Tick() {
    static unsigned char dir_flag = 0x00;
    static unsigned char dir = 0;
    static unsigned int count = 0;
    static unsigned int mult = 180;

    switch(light_state) {
        case light_enter:
            if (dir_flag == 0x02) { //on
                if(dir<=0)dir = 7;
                else dir -=1;

            }
            else { //off
                if (dir < 7) dir = dir + 1;
                else dir = 0;
            }

            count++;
            if(count>=multiplier(mult)){
                count = 0;
                light_state = light_wait;
                //turns on simulation light in PORTD
                if (dir_flag == 0x02) {
                  simu_light_lh = simu_light_rh | 0x40; //turn on
                }
                else {
                  simu_light_lh = simu_light_rh & 0xBF;
                }
            }

            //last 4 ports
            unsigned char tmp = arr[dir];
            //bit shift the arr[dir]
            tmp <<= 4;
            motor_lh = (tmp | 0x0F) & motor_rh ;

            break;
        case light_wait:
            if(bluetooth_receive == 0x30){ //turn off
                if (dir_flag != 0x01) {
                    dir_flag = 0x01;
                    light_state = light_enter;
                }
            }
            else if (bluetooth_receive == 0x31) { //turn on
                if (dir_flag != 0x02) {
                    dir_flag = 0x02;
                    light_state = light_enter;
                }
            }
        default:
            break;
    }
}

void LIGHT_Task() {
    LIGHT_Init();
    for(;;)
    {
        LIGHT_Tick();
        vTaskDelay(2);
    }
}

void StartSecPulse(unsigned portBASE_TYPE Priority)
{
    xTaskCreate(BLUETOOTH_Task, (signed portCHAR *)"Bluetooth", configMINIMAL_STACK_SIZE, NULL, Priority, NULL );
    xTaskCreate(DOOR_Task, (signed portCHAR *)"Door", configMINIMAL_STACK_SIZE, NULL, Priority, NULL );
    xTaskCreate(PHOTO_TRANSIT_Task, (signed portCHAR *)"Photo-Transistor", configMINIMAL_STACK_SIZE, NULL, Priority, NULL );
    xTaskCreate(GARAGE_Task, (signed portCHAR *)"Garage Door", configMINIMAL_STACK_SIZE, NULL, Priority, NULL );
    xTaskCreate(LIGHT_Task, (signed portCHAR *)"Light", configMINIMAL_STACK_SIZE, NULL, Priority, NULL );

}
int main(void)
{
    //phototransistor
    DDRA = 0x00; PORTA = 0x00;
    // //stepper motor
    DDRB = 0XFF; PORTB = 0x00;
    DDRC = 0XFF; PORTC = 0x00;
    //usart
    DDRD = 0xFF;

    A2D_init();
    initUSART(uart_port_0);
    //Start Tasks
    StartSecPulse(1);
    //RunSchedular
    vTaskStartScheduler();

    return 0;
}
