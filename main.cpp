#include "mbed.h"
#include "LPS331.h"
#include "LIS3DH.h"
#include "LM75B.h"


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


char str[255];




float latitude;
float longitude;
int number;

volatile int rx_in=0;
volatile int rx_out=0;
const int buffer_size = 255;
char rx_buffer[buffer_size+1];

char rx_line[buffer_size];

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

void check_sw2(void)
{
    if (sw2 == 0) {
        pc.printf("SW2 button pressed. \r\n");
        // turn RED LED on to indicate provisioning process has started
        led_red = 0;
        led_green = 1;
        wait(1);
        // turn LEDs off
        led_red = 1;
        led_green = 1;      
        skywire.printf("ATD*22899;\r\n");
        WaitForResponse("#OTASP: 0", 9);
        pc.printf("Provisioning: #OTASP: 0 reached. \r\n");        
        // turn RED LED on
        led_red = 0;
        led_green = 1;       
        WaitForResponse("#OTASP: 1", 9);
        pc.printf("Provisioning: #OTASP: 1 reached. \r\n");          
        // turn both LEDs on
        led_red = 0;
        led_green = 0;            
        WaitForResponse("#OTASP: 2", 9);
        pc.printf("Provisioning: #OTASP: 2 reached. \r\n");         
        // turn GREEN LED on
        led_red = 1;
        led_green = 0;           
        WaitForResponse("NO CARRIER", 10);
        pc.printf("Provisioning successfully completed. \r\n");
 
        skywire.printf("AT#REBOOT");
        // wait 10 seconds 
        wait(10);       
        
        
    }
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
    
    // check to see if switch 2 is pressed, if so, execute provisioning sequence
    check_sw2();


    // read out MEID number to use as unique identifier
    skywire.printf("AT#MEIDESN?\r\n");
    wait(2);
    read_line(); 
    read_line(); 
    sscanf(rx_line, "%*s %14s,", DeviceID);

    pc.printf("Device MEID: %s \r\n", DeviceID); 

    pc.printf("Connecting to Network...\r\n");
    // get IP address
    skywire.printf("AT#SGACT=1,1\r\n");
    WaitForResponse("#SGACT", 6);
    WaitForResponse("OK", 2);
    // use LEDs to indicate we're connected to the newtork
    led_red=0;
    led_green=0; 
       
    // connect to dweet.io
    skywire.printf("AT#HTTPCFG=1,\"dweet.io\",80,0\r\n");
    WaitForResponse("OK", 2);
    
    //get location approximation from cell tower information
    skywire.printf("AT#AGPSSND\r\n");
    WaitForResponse("#AGPSRING:", 10);

    //debug_pc.printf("Skywire says: %s\r\n", rx_line);
    sscanf(rx_line, "%s %d,%f,%f,", str, &number, &latitude, &longitude);
    //debug_pc.printf("Location: Latt:%f, Long:%f\r\n", latitude, longitude);
    wait(3);
    
    while(1) {
      temp = (float)LM75_temp;
      temp = temp *9 /5 + 32;
        pc.printf("Temp = %.3f\r\n", temp);
         press=(float)pressure.value() / 4096;
        pc.printf("Pressure = %.3f\r\n", press);
        accel.read_data(axis);
        pc.printf("Accel = %.3f, %.3f, %.3f\r\n", axis[0], axis[1], axis[2]);
        
        wait(0.25);
        // turn LED Green to indicate transmission
        led_red=1;
        led_green = 0;  

          
        //Report Sensor Data to dweet.io
        skywire.printf("AT#HTTPQRY=1,0,\"/dweet/for/%s?temperature=%.3f&pressure=%.3f&X=%.3f&Y=%.3f&Z=%.3f&Latitude=%f&Longitude=%f\"\r\n", DeviceID, temp, press, axis[0], axis[1], axis[2], latitude, longitude);
        WaitForResponse("#HTTPRING", 9);
        skywire.printf("AT#HTTPRCV=1\r\n");
        WaitForResponse("OK", 2);
        led_green = 1;  
        wait(2);
        // see if user has requested to pause transmissions
        check_sw3();
    }


    
    
}
