
#include "platform.h"
#include "xparameters.h"
#include "xgpio.h"
#include "xuartlite.h"
#include "xuartlite_l.h"
#include "xstatus.h"
#include "sleep.h"
#include "xil_io.h"
#include <string.h>

/* ===== Hardware ===== */
#define LED_ID    XPAR_GPIO_0_DEVICE_ID
#define KEY_ID    XPAR_GPIO_1_DEVICE_ID
#define LIGHT_ID  XPAR_GPIO_2_DEVICE_ID
#define MOTOR_ID  XPAR_GPIO_3_DEVICE_ID
#define UART_ID   XPAR_UARTLITE_0_DEVICE_ID

#define CH 1

/* Light DO=0 means bright (change to 1 if needed) */
#define BRIGHT_LEVEL 0

#define MOTOR_STOP 0
#define MOTOR_FWD  1
#define MOTOR_REV  2

static XGpio led, key, light, motor;
static XUartLite uart;

static unsigned int light_state;
static unsigned int led_state;
static unsigned int motor_state = 0;

static void service_board(void);
static void command_byte(char c);
static void command_line(const char *s);
static int client = -1;

/* UART / AT parser */
static char line[80];
static unsigned int lp = 0;
static int ipd_count = 0;
static int ipd_client = -1;

static int got_ok = 0;
static int got_error = 0;
static int got_prompt = 0;
static int got_send_ok = 0;

/* Message waiting to be sent */
static const char *pending = 0;


/* ===== UART transmit ===== */

static void tx_char(char c)
{
    XUartLite_SendByte(
        uart.RegBaseAddress, (unsigned char)c
    );
}

static void tx_string(const char *s)
{
    while (*s)
        tx_char(*s++);
}

static void tx_number(unsigned int n)
{
    char b[12];
    int i = 0;

    do {
        b[i++] = '0' + n % 10;
        n /= 10;
    } while (n && i < 11);

    while (i)
        tx_char(b[--i]);
}

/* ===== GPIO ===== */

static void set_led(unsigned int v)
{
    led_state = v & 15;
    XGpio_DiscreteWrite(&led, CH, led_state);
}

static void set_motor(unsigned int v)
{
    motor_state = v & 3;
    XGpio_DiscreteWrite(&motor, CH, motor_state);
}

static void update_auto_led(void)
{
    if (light_state == BRIGHT_LEVEL)
        set_led(0);
    else
        set_led(15);
}

/* ===== Compare strings without libc ===== */

static int same(const char *a, const char *b)
{
    while (*a && *b)
    {
        if (*a != *b)
            return 0;
        a++;
        b++;
    }
    return (*a == 0 && *b == 0);
}

static int starts(const char *a, const char *b)
{
    while (*b)
    {
        if (*a++ != *b++)
            return 0;
    }
    return 1;
}

/* ===== ESP8266 response parser ===== */

static void reset_commands(void);
static void process_line(void)
{
    int id;

    line[lp] = 0;

    if (same(line, "OK"))
        got_ok = 1;

    if (same(line, "ERROR") ||
        same(line, "FAIL"))
        got_error = 1;

    if (same(line, "SEND OK"))
        got_send_ok = 1;

    /* Incoming TCP connection */
    if (line[0] >= '0' && line[0] <= '4')
    {
        id = line[0] - '0';

        if (same(line + 1, ",CONNECT"))
        {
            if (client < 0) client = id;
            pending = 0;
        }

        if (same(line + 1, ",CLOSED"))
        {
            if (client == id)
            {
                client = -1;
                reset_commands();
                pending = 0;
                set_motor(MOTOR_STOP);
            }
        }
    }
}

/* Parse +IPD,ID,LEN: header */
static void parse_ipd(void)
{
    unsigned int p = 5;
    unsigned int id = 0;
    unsigned int len = 0;

    if (!starts(line, "+IPD,"))
        return;

    if (line[p] < '0' || line[p] > '9')
        return;

    while (line[p] >= '0' && line[p] <= '9')
    {
        id = id * 10 + line[p] - '0';
        p++;
    }

    if (line[p++] != ',')
        return;

    if (line[p] < '0' || line[p] > '9')
        return;

    while (line[p] >= '0' && line[p] <= '9')
    {
        len = len * 10 + line[p] - '0';
        p++;
    }

    if (line[p] != ':')
        return;

    if (id <= 4 && len > 0 && len <= 2048)
    {
        ipd_client = (int)id;
        ipd_count = (int)len;
        if (client < 0) client = (int)id;
    }
}

/* Receive one UART byte */
static void rx_byte(unsigned char c)
{
    if (ipd_count > 0)
    {
        if (ipd_client == client)
            command_byte((char)c);

        ipd_count--;
        return;
    }

    if (c == '>')
    {
        got_prompt = 1;
        lp = 0;
        return;
    }

    if (c == '\r')
        return;

    if (c == '\n')
    {
        if (lp > 0)
            process_line();

        lp = 0;
        return;
    }

    if (lp >= sizeof(line) - 1)
    {
        lp = 0;
        return;
    }

    line[lp++] = (char)c;

    if (c == ':' && lp >= 7)
    {
        line[lp] = 0;

        if (starts(line, "+IPD,"))
        {
            parse_ipd();
            lp = 0;
        }
    }
}

static void poll_uart(void)
{
    service_board();
    while (!XUartLite_IsReceiveEmpty(
               uart.RegBaseAddress))
    {
        unsigned char c;

        c = (unsigned char)XUartLite_ReadReg(
            uart.RegBaseAddress,
            XUL_RX_FIFO_OFFSET
        );

        rx_byte(c);
    }
}

/* ===== AT commands ===== */

static void clear_flags(void)
{
    got_ok = 0;
    got_error = 0;
    got_prompt = 0;
    got_send_ok = 0;
}

static int wait_flag(int type, int timeout_ms)
{
    int i;

    for (i = 0; i < timeout_ms; i++)
    {
        poll_uart();

        if (got_error)
            return 0;

        if (type == 0 && got_ok)
            return 1;

        if (type == 1 && got_prompt)
            return 1;

        if (type == 2 && got_send_ok)
            return 1;

        usleep(1000);
    }

    return 0;
}

static int at_cmd(const char *cmd, int timeout)
{
    poll_uart();
    clear_flags();

    tx_string(cmd);
    tx_string("\r\n");

    return wait_flag(0, timeout);
}

/* ===== Initialize ESP8266 ===== */

static int wifi_init(void)
{
    int i;

    sleep(3);
    poll_uart();

    for (i = 0; i < 5; i++)
    {
        if (at_cmd("AT", 1500))
            break;

        sleep(1);
    }

    if (i == 5)
        return 0;

    at_cmd("ATE0", 1500);

    if (!at_cmd("AT+CWMODE=2", 3000))
        return 0;

    if (!at_cmd(
        "AT+CWSAP=\"MicroBlaze_WIFI\",\"12345678\",1,3",
        5000))
        return 0;

    if (!at_cmd("AT+CIPMODE=0", 2000))
        return 0;

    if (!at_cmd("AT+CIPMUX=1", 2000))
        return 0;

    /* Ignore error if no old server exists */
    at_cmd("AT+CIPSERVER=0", 2000);

    if (!at_cmd("AT+CIPSERVER=1,8086", 5000))
        return 0;

    return 1;
}

/* ===== Send TCP data to phone ===== */

static int wifi_send(const char *s)
{
    unsigned int len = 0;
    const char *p;

    if (client < 0 || s == 0)
        return 0;

    p = s;
    while (*p++)
        len++;

    if (len == 0)
        return 0;

    poll_uart();
    clear_flags();

    tx_string("AT+CIPSEND=");
    tx_number((unsigned int)client);
    tx_char(',');
    tx_number(len);
    tx_string("\r\n");

    if (!wait_flag(1, 3000))
        return 0;

    clear_flags();
    tx_string(s);

    return wait_flag(2, 5000);
}


/* Sixth-test protocol uses newline-delimited H6Q52C0 frames. */
#define MAILBOX_BASE 0x40040000U
static unsigned int toggle;
static unsigned int date_value,time_value,valid,now_ms;
static int temperature,humidity,threshold=280,manual=0,requested=0;
static int actual=0,goal=0,dead=0,refresh=1;
static unsigned int dead_start,last_service,last_report;
static unsigned long last_seq;
static char input[160],reply[128];static unsigned int input_pos;static int dropping;
static unsigned long reply_seq;static int reply_pending;

static int mailbox(unsigned int op,unsigned int payload,unsigned int *value)
{
 unsigned int r;int i;
 toggle^=0x80000000U;
 Xil_Out32(MAILBOX_BASE,toggle|(op<<24)|(payload&0xffffffU));
 for(i=0;i<(op==18?20000:200000);i++) {
  if(op==18)usleep(100);
  r=Xil_In32(MAILBOX_BASE+8);
  if((r&0x80000000U)==toggle) {if(value)*value=r&0xffffffU;return !(r&0x40000000U);}
 }
 return 0;
}
static unsigned int age(unsigned int a,unsigned int b){return (a-b)&0xffffffU;}
static void motor_write(int v){actual=v;set_motor((unsigned int)v);}
static void control(void)
{
 int target=manual?requested:((valid&2)&&light_state==BRIGHT_LEVEL&&temperature>threshold?MOTOR_FWD:MOTOR_STOP);
 if(target==MOTOR_STOP){goal=0;dead=0;motor_write(0);return;}
 if(target!=goal){goal=target;if(actual&&actual!=target){motor_write(0);dead=1;dead_start=now_ms;}}
 if(dead){if(age(now_ms,dead_start)<100)return;dead=0;}
 if(actual!=target)motor_write(target);
}
static void service_board(void)
{
 unsigned int v,d,t,h,f;
 // Called during UART waits too. No UART functions here (avoids recursion).
 if(!refresh){if(!mailbox(0,0,0)||!mailbox(6,0,&v))return;if(age(v,last_service)<10)return;}
 refresh=0;
 if(!mailbox(0,0,0)||!mailbox(1,0,&d)||!mailbox(2,0,&t)||!mailbox(3,0,&v)) {valid=0;motor_write(0);return;}
 temperature=(short)v;
 if(!mailbox(4,0,&h)||!mailbox(5,0,&f)||!mailbox(6,0,&v)){valid=0;motor_write(0);return;}
 date_value=d;time_value=t;humidity=(int)h;valid=f;now_ms=v;last_service=v;
 light_state=XGpio_DiscreteRead(&light,CH)&1;
 update_auto_led();control();
}
/* Bounded integer-only serialization: avoids newlib printf/scanf machinery. */
typedef struct { char *p; unsigned int left; int ok; } Text;
static void text_char(Text *b,char c)
{
 if(b->left>1){*b->p++=c;b->left--;*b->p=0;}
 else b->ok=0;
}
static void text_string(Text *b,const char *s)
{
 while(*s)text_char(b,*s++);
}
static void text_unsigned(Text *b,unsigned long v,unsigned int width)
{
 char digits[11];unsigned int n=0;
 do{digits[n++]=(char)('0'+v%10);v/=10;}while(v&&n<sizeof(digits));
 while(width>n){text_char(b,'0');width--;}
 while(n)text_char(b,digits[--n]);
}
static void text_signed(Text *b,int v)
{
 unsigned int magnitude;
 if(v<0){text_char(b,'-');magnitude=(unsigned int)(-(v+1))+1U;}
 else magnitude=(unsigned int)v;
 text_unsigned(b,magnitude,0);
}
static void ack(unsigned long seq,const char *result)
{
 Text b={reply,sizeof(reply),1};reply[0]=0;
 text_string(&b,"H6Q52C0|A|");text_unsigned(&b,seq,0);
 text_char(&b,'|');text_string(&b,result);text_char(&b,'\n');
 reply_seq=seq;reply_pending=b.ok;
}
static int bcd(int v){return ((v/10)<<4)|(v%10);}
static int unpack(int v){return (v>>4)*10+(v&15);}
static int date_valid(int y,int m,int d,int h,int n,int s)
{
 static const int days[]={31,28,31,30,31,30,31,31,30,31,30,31};int max;
 if(y<2000||y>2099||m<1||m>12||h<0||h>23||n<0||n>59||s<0||s>59)return 0;
 max=days[m-1]+(m==2&&y%4==0);return d>=1&&d<=max;
}
static int decimal(const char *s,int *v)
{
 unsigned int a=0,d;int negative=0;
 if(*s=='-'){negative=1;s++;}
 if(!*s)return 0;
 while(*s){
  if(*s<'0'||*s>'9')return 0;
  d=(unsigned int)(*s++-'0');
  if(a>(100000U-d)/10U)return 0;
  a=a*10U+d;
 }
 *v=negative?-(int)a:(int)a;return 1;
}
static int sequence_number(const char *s,unsigned long *value)
{
 unsigned long a=0,d;
 if(!*s)return 0;
 while(*s){
  if(*s<'0'||*s>'9')return 0;
  d=(unsigned long)(*s++-'0');
  if(a>(0xffffffffUL-d)/10UL)return 0;
  a=a*10UL+d;
 }
 if(!a)return 0;
 *value=a;return 1;
}
static void command_line(const char *s)
{
 char local[160],*p[12],*q;int count=0,v,y,m,d,h,n,sec;unsigned long seq;
 if(strlen(s)>=sizeof(local))return;
 strcpy(local,s);p[count++]=local;
 for(q=local;*q;q++)if(*q=='|'){*q=0;if(count>=12)return;p[count++]=q+1;}
 if(count<4||strcmp(p[0],"H6Q52C0")||strcmp(p[1],"C"))return;
 if(!sequence_number(p[2],&seq))return;
 last_seq=seq;
 if(!strcmp(p[3],"GET")&&count==4){last_report=(now_ms-1000)&0xffffffU;ack(seq,"OK");return;}
 if(!strcmp(p[3],"LIMIT")&&count==5&&decimal(p[4],&v)&&v>=-400&&v<=800){threshold=v;control();ack(seq,"OK");return;}
 if(!strcmp(p[3],"MODE")&&count==5&&decimal(p[4],&v)&&(v==0||v==1)){
  manual=v;requested=0;control();ack(seq,"OK");return;
 }
 if(!strcmp(p[3],"FAN")&&count==5&&decimal(p[4],&v)&&v>=0&&v<=2){
  if(!manual){ack(seq,"MODE_REQUIRED");return;}requested=v;control();ack(seq,"OK");return;
 }
 if(!strcmp(p[3],"CLOCK")&&count==10&&decimal(p[4],&y)&&decimal(p[5],&m)&&decimal(p[6],&d)&&decimal(p[7],&h)&&decimal(p[8],&n)&&decimal(p[9],&sec)&&date_valid(y,m,d,h,n,sec)){
  unsigned int a=(bcd(y-2000)<<16)|(bcd(m)<<8)|bcd(d),b=(bcd(h)<<16)|(bcd(n)<<8)|bcd(sec);
  if(mailbox(16,a,0)&&mailbox(17,b,0)&&mailbox(18,0,0)){ack(seq,"OK");refresh=1;}
  else ack(seq,"RTC_IO_ERROR");
  return;
 }
 ack(seq,"BAD_COMMAND");
}
static void reset_commands(void){input_pos=0;dropping=0;manual=0;requested=0;refresh=1;reply_pending=0;}
static void command_byte(char c)
{
 if(c=='\r')return;
 if(c=='\n') {if(!dropping){input[input_pos]=0;command_line(input);}input_pos=0;dropping=0;return;}
 if(dropping)return;
 if(input_pos>=sizeof(input)-1){dropping=1;input_pos=0;return;}
 input[input_pos++]=c;
}
static int format_status(char *buffer,unsigned int size)
{
 Text b={buffer,size,1};
 if(!size)return 0;
 buffer[0]=0;
 text_string(&b,"H6Q52C0|S|");text_unsigned(&b,now_ms,0);text_char(&b,'|');
 text_unsigned(&b,2000U+(unsigned int)unpack((date_value>>16)&255),4);text_char(&b,'-');
 text_unsigned(&b,(unsigned int)unpack((date_value>>8)&255),2);text_char(&b,'-');
 text_unsigned(&b,(unsigned int)unpack(date_value&255),2);text_char(&b,'T');
 text_unsigned(&b,(unsigned int)unpack((time_value>>16)&255),2);text_char(&b,':');
 text_unsigned(&b,(unsigned int)unpack((time_value>>8)&255),2);text_char(&b,':');
 text_unsigned(&b,(unsigned int)unpack(time_value&255),2);text_char(&b,'|');
 text_signed(&b,temperature);text_char(&b,'|');
 text_unsigned(&b,(unsigned int)humidity,0);text_char(&b,'|');
 text_unsigned(&b,light_state==BRIGHT_LEVEL,0);text_char(&b,'|');
 text_signed(&b,threshold);text_char(&b,'|');
 text_unsigned(&b,(unsigned int)manual,0);text_char(&b,'|');
 text_unsigned(&b,(unsigned int)actual,0);text_char(&b,'|');
 text_unsigned(&b,valid,0);text_char(&b,'\n');
 return b.ok;
}
static void report(void)
{
 char b[240];
 if(format_status(b,sizeof(b)))wifi_send(b);
}
int main(void)
{
 int ok;unsigned long sending_seq;char sending[128];
 init_platform();
 if(XGpio_Initialize(&led,LED_ID)!=XST_SUCCESS||XGpio_Initialize(&key,KEY_ID)!=XST_SUCCESS||
 XGpio_Initialize(&light,LIGHT_ID)!=XST_SUCCESS||XGpio_Initialize(&motor,MOTOR_ID)!=XST_SUCCESS||XUartLite_Initialize(&uart,UART_ID)!=XST_SUCCESS)return -1;
 XGpio_SetDataDirection(&led,CH,0);XGpio_SetDataDirection(&key,CH,31);
 XGpio_SetDataDirection(&light,CH,1);XGpio_SetDataDirection(&motor,CH,0);motor_write(0);
 Xil_Out32(MAILBOX_BASE+4,0);Xil_Out32(MAILBOX_BASE+12,0xffffffffU);
 toggle=Xil_In32(MAILBOX_BASE+8)&0x80000000U;
 for(;;){
  ok=wifi_init(); // sensor/display continue in PL if WiFi is unavailable
  if(ok)break;
  service_board();
 }
 for(;;){
  poll_uart();
  if(client>=0&&reply_pending){
   strcpy(sending,reply);sending_seq=reply_seq;
   // Clear before sending: commands arriving during send can queue a new ACK.
   reply_pending=0;wifi_send(sending);(void)sending_seq;
  }
  if(client>=0&&age(now_ms,last_report)>=1000){last_report=now_ms;report();}
  usleep(1000);
 }
}
