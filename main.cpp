/* main.cpp */
/* v1.1
 * Copyright (C) 2015 nimbelink.com, MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "mbed.h"
#include "LPS331.h"
#include "LIS3DH.h"
#include "LM75B.h"
#include "hts221.h"

//  #define DeviceID "A100003E4226B3"  //Freeboard DweetIO unique ID
// dynamically assigning this based on modem's unique identifier below
char DeviceID[15];

DigitalOut skywire_en(PTD3);    //K64 FRDM
DigitalOut skywire_rts(PTD2);    //K64 FRDM
DigitalOut skywire_dtr(PTD0);    //K64 FRDM

DigitalOut led_red(LED_RED);
DigitalOut led_green(LED_GREEN);
DigitalIn sw2(SW2);
DigitalIn sw3(SW3);
Serial pc(USBTX, USBRX);
Serial skywire(PTC17, PTC16);    //K64 FRDM

I2C i2c(PTE25,PTE24);
char msg[3];

LPS331 pressure(i2c);
LM75B LM75_temp(PTE25,PTE24);
LIS3DH accel(i2c, LIS3DH_V_CHIP_ADDR, LIS3DH_DR_NR_LP_100HZ, LIS3DH_FS_2G);
HTS221 humidity(PTE25, PTE24);

char str[255];

float latitude = 0;
float longitude = 0;
int number;

volatile int rx_in=0;
volatile int rx_out=0;
const int buffer_size = 255;
char rx_buffer[buffer_size+1];

char rx_line[buffer_size];

// Operator APN name
char Operator[10] = "webe";

void Skywire_Rx_interrupt() {
// Loop just in case more than one character is in UART's receive FIFO buffer
// Stop if buffer full
    while ((skywire.readable()) && (((rx_in + 1) % buffer_size) != rx_out)) {
        rx_buffer[rx_in] = skywire.getc();
        rx_in = (rx_in + 1) % buffer_size;
    }
    return;
}

void read_line() {
    int i;
    i = 0;
// Start Critical Section - don't interrupt while changing global buffer variables
    __disable_irq();
// Loop reading rx buffer characters until end of line character
    while ((i==0) || (rx_line[i-1] != '\n')) {
// Wait if buffer empty
        if (rx_in == rx_out) {
// End Critical Section - need to allow rx interrupt to get new characters for buffer
            __enable_irq();
            while (rx_in == rx_out) {
              //pc.printf("..Read_line entered2.5\r\n");
            }
// Start Critical Section - don't interrupt while changing global buffer variables
            __disable_irq();
        }
        rx_line[i] = rx_buffer[rx_out];
        i++;
        rx_out = (rx_out + 1) % buffer_size;
    }
// End Critical Section
    __enable_irq();
    rx_line[i-1] = 0;
    return;
}

int WaitForResponse(char* response, int num) {
    do {
        read_line();
        pc.printf("Waiting for: %s, Recieved: %s\r\n", response, rx_line);
    } while (strncmp(rx_line, response, num));
    return 0;
}

void blinkRG(int blinkduration)
{
    // blinkduration is measured in seconds
    led_red=1;
    led_green=0;
    while (blinkduration-- >=1){
        led_red = led_green;
        led_green=!led_red;
        wait(1);
    }
    // leave the function with LEDs both off
    led_red=1;
    led_green=1;

}

void check_sw3(void)
{
    if (sw3 == 0) {
        pc.printf("SW3 button pressed. \r\n");
        // this button press is used to pause transmissions (to save data if the device is on forever);
        pc.printf("Transmissions are paused. \r\n");
        // turn on RED LED
        led_green = 1;
        led_red = 0;
        // wait a few seconds for user to release the button.
        wait(3);

        // wait until user presses the button again to unpause
        while(sw3 ==1);
       pc.printf("Transmissions resumed. \r\n");
         // turn LEDs off
        led_green = 1;
        led_red = 1;
    }
}

int main() {

    led_red = 1;
    led_green = 1;
    float axis[3];
    float press;
    float temp;
    float humi;
    float dummy_temp;

    // turn on Skywire modem
    skywire_en = 1;

    // setup skywire UART
    skywire_rts=0;
    skywire_dtr=0;
    skywire.baud(115200);
    skywire.format(8, Serial::None, 1);

    // setup Skywire UART interrupt
    skywire.attach(&Skywire_Rx_interrupt, skywire.RxIrq);

    pc.baud(115200);
    pc.printf("Hello World from FRDM-K64F board.\r\n");
    pc.printf("Starting Demo...\r\n");
    pc.printf("Waiting for Skywire to Boot...\r\n");

    // wait 10 seconds for modem to boot up and connect to network
    blinkRG(10);
    pc.printf("Waiting complete...\r\n");

    LM75_temp.open();

    //Turn off echo
    skywire.printf("ATE0\r\n");
    WaitForResponse("OK", 2);

    // Set Report Mobile Equipment Error (CMEE)
    skywire.printf("AT+CMEE=1\r\n");
    WaitForResponse("OK", 2);

    // Set 3GPP Network
    skywire.printf("AT+WS46=25\r\n");
    WaitForResponse("OK", 2);

    // Check whether SIM is ready
    skywire.printf("AT+CPIN?\r\n");
    WaitForResponse("+CPIN: READY", 12);

    // Get IMEI
    skywire.printf("AT#CGSN\r\n");
    WaitForResponse("OK", 2);
    read_line();
    read_line();
    sscanf(rx_line, "%*s %14s,", DeviceID);
    pc.printf("Device IMEI: %s \r\n", DeviceID);

#ifdef ADDCHECK
    // Get IMSI
    skywire.printf("AT#CIMI\r\n");
    WaitForResponse("#CIMI", 5);

    // Check whether signal is received
    skywire.printf("AT+CSQ\r\n");
    WaitForResponse("+CSQ", 4);

    // Check GPRS Context Activation if GPRS is ready
    skywire.printf("AT#GPRS?\r\n");
    WaitForResponse("#GPRS", 5);
#endif

    // Check Network Registration Status
    skywire.printf("AT+CREG?\r\n");
    // Make sure connect to Home Network
    WaitForResponse("+CREG: 0,1", 10);

    // Set the PDN context for Operator Network
    skywire.printf("AT+CGDCONT=1,\"IP\",\"%s\"\r\n", Operator);
    WaitForResponse("OK", 2);
    skywire.printf("AT+CGDCONT?\r\n");
    WaitForResponse("+CGDCONT: 1", 11);
    WaitForResponse("+CGDCONT: 2", 11);
    WaitForResponse("+CGDCONT: 3", 11);

    pc.printf("Connecting to Network...\r\n");
    // get IP address
    skywire.printf("AT#SGACT=1,1\r\n");
    WaitForResponse("#SGACT", 6);
    WaitForResponse("OK", 2);
    // use LEDs to indicate we're connected to the newtork
    led_red=0;
    led_green=0;

#ifdef DNSQ
    blinkRG(10);
    // Query DNS
    skywire.printf("AT#QDNS=\"google.com\"\r\n");
    WaitForResponse("#QDNS", 5);
#endif

    wait(3);

    while(1) {
        temp = (float)LM75_temp;
        temp = temp *9 /5 + 32;
        pc.printf("Temp = %.3f\r\n", temp);
        press=(float)pressure.value() / 4096;
        pc.printf("Pressure = %.3f\r\n", press);
        humidity.ReadTempHumi(&dummy_temp, &humi);
        pc.printf("Humidity = %.3f\r\n", humi);
        accel.read_data(axis);
        pc.printf("Accel = %.3f, %.3f, %.3f\r\n", axis[0], axis[1], axis[2]);

        wait(0.25);
        // turn LED Green to indicate transmission
        led_red=1;
        led_green = 0;

        // Socket Dial to HTTP server
        skywire.printf("AT#SD=1,0,80,\"dweet.io\"\r\n");
        WaitForResponse("CONNECT", 7);
        // HTTP POST to dweet.io
        skywire.printf("POST /dweet/for/%s?temp=%.3f&press=%.3f&humi=%.3f&X=%.3f&Y=%.3f&Z=%.3f&Latitude=%f&Longitude=%f HTTP/1.1\r\n\r\n", DeviceID, temp, press, humi, axis[0], axis[1], axis[2], latitude, longitude);
        // Wait for Carrier Return
        WaitForResponse("NO CARRIER", 10);
        // Close connection
        skywire.printf("AT#SH=1\r\n");
        WaitForResponse("OK", 2);

        //Report Sensor Data to dweet.io
        led_green = 1;
        wait(2);
        // see if user has requested to pause transmissions
        check_sw3();
    }
}
