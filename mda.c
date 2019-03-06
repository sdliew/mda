/*
 * mda.c:
 * 
 * Program for controlling shift register to trigger relays.
 *
 *
 * Compile: gcc -I/usr/include/mysql -lmysqlclient mda.c -o mda -lwiringPi -Wall
 *
 * Shift register control, works on 74HC595 and TLC5917 Tri-color LED driver.
 */

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <ctype.h>
#include <sys/ioctl.h>
#include <string.h>
#include "mysql.h"
#include <wiringPi.h>
#include <sr595.h>


#define BUFF_SIZE 128 

/* The mysql descriptor needs to be declared as global, 
   as the descriptor is needed throughout the process runtime. 
*/ 
MYSQL mysql;
int sd;
int sleeptime = 50000;

int open_com_port(char *interface)
{
    int status;
    struct termios options;

    if (interface == NULL)
	interface = "/dev/ttyUSB0";

    fprintf(stderr,"Initializing serial port %s ... ",interface);

    /* Open COM port */
    if ((sd = open(interface, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) < 0) {
	perror(interface);
	exit(0);
    }

    fcntl(sd, F_SETFL, O_RDWR);

    tcgetattr(sd, &options);
    cfmakeraw(&options); 
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    /* |= disable; &= ~ enable */
    options.c_cflag |= (CLOCAL | CREAD);                /* Set local mode and enable receiver */
    options.c_cflag &= ~PARENB;                         /* Disable parity bit */
    options.c_cflag |= CSTOPB;                          /* 2 stop bits */
//    options.c_cflag &= ~CSTOPB;                          /* 1 stop bits */
    options.c_cflag &= ~CSIZE;                          /* Bit mask for data bits */
    options.c_cflag |= CS8;                             /* 8 data bits */
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /* Enable raw input and echo input characters */
    options.c_iflag |= (IXON | IXOFF | IXANY);          /* Enable software flow control */
    options.c_oflag |= OPOST;                           /* Postprocess output */
//    options.c_oflag &= ~OPOST;                           /* Postprocess output */
    
    tcsetattr(sd, TCSANOW, &options);                   /* Make changes now */

    sleep(2);
    tcflush(sd, TCIOFLUSH);

    ioctl(sd, TIOCMGET, &status);

    status |= TIOCM_DTR;
    status |= TIOCM_RTS;

    ioctl(sd, TIOCMSET, &status);

//    fcntl(sd, F_SETFL, FNDELAY);                        /* Set read() to return immediately */

    printf("done\n");
    
    return sd;
}

void close_com_port()
{
    close(sd);
}

void send_message(char *optarg)
{
   int length = 0;
   char buffer[BUFF_SIZE];

   memset(buffer, 0, BUFF_SIZE);

   printf("send_message(\"%s\")\n",optarg);

   snprintf(buffer,BUFF_SIZE,"%s\r\n%n",optarg,&length);

   if (write(sd, buffer, length) != length)
     perror("write()");
    
}

int read_char(int fd, char *buffer, int length)
{
  int count = 0;

  memset(buffer, 0, length);

  do {

    count += read(fd, buffer + count, 1);

  } while ((count < length) && (*(buffer + count - 1) != '\x0a'));
   /* Data string from Agilent devices end with \x0d\x0a */

  return count;
}

int wait_for_reply(int fd, char *buffer, int length)
{
  int count = 0;
  int found = 0;
  
  fd_set rfds;

  struct timeval tv;

  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);

  /* Waiting time in seconds and micro-seconds. */
  tv.tv_sec = 2;   /*34401A takes longer time to determine open circuit */
  tv.tv_usec = 0;
  
    /* The first argument in select() specifies the I/O or descriptor
       to keep track on, such as input from keyboard, data receive
       from network connection.  select() returns 1 when successful, 0
       when time limit expires, and -ive values when encounters
       error. 
    */
    found = select(fd + 1, &rfds, NULL, NULL, &tv);
  
    if (found == 0) { /* Timeout */
      write(0,"Timeout!!!\n",11);
      return found;
    } else if (found < 0) {
      perror("select()");
      return found;
    } else {
      count = read_char(fd, buffer, length);
    }

  return count;
}

int receive_message(char *buffer)
{
  int i;
  int length = 0;

  memset(buffer, 0, BUFF_SIZE);

    if ((length = wait_for_reply(sd, buffer, BUFF_SIZE)) > 0) {

       for (i=0; (buffer[i] != 0) && (i < length); i++) {

          if (isprint(buffer[i]))
  	     printf("%c", buffer[i]);
          else
             printf("[%.2x]", buffer[i]);
       }

       if (length != 0)
          printf("\n");
    }

  return length;

}


int serial_init(int serialSpeed){
    struct termios options;
    int speed;
    int status;
    int sd;

    const char serialPort[] = "/dev/ttyUSB0";

        switch(serialSpeed){
                case     50:    speed =     B50 ; break ;
                case     75:    speed =     B75 ; break ;
                case    110:    speed =    B110 ; break ;
                case    134:    speed =    B134 ; break ;
                case    150:    speed =    B150 ; break ;
                case    200:    speed =    B200 ; break ;
                case    300:    speed =    B300 ; break ;
                case    600:    speed =    B600 ; break ;
                case   1200:    speed =   B1200 ; break ;
                case   1800:    speed =   B1800 ; break ;
                case   2400:    speed =   B2400 ; break ;
                case   9600:    speed =   B9600 ; break ;
                case  19200:    speed =  B19200 ; break ;
                case  38400:    speed =  B38400 ; break ;
                case  57600:    speed =  B57600 ; break ;
                case 115200:    speed = B115200 ; break ;
                default:        speed = B230400 ; break ;
        }


        if ((sd = open(serialPort, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1){
                fprintf(stderr,"Unable to open the serial port %s - \n", serialPort);
                exit(-1);
        } else {
                fprintf(stderr,"Initializing serial port %s ... ", serialPort);
        }

        fcntl (sd, F_SETFL, O_RDWR) ;

        tcgetattr(sd, &options);
        cfmakeraw(&options);
        cfsetispeed (&options, speed);
        cfsetospeed (&options, speed);

        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;

	options.c_cflag |= CS8;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag |= (IXON | IXOFF | IXANY);          /* Enable software flow control */
        options.c_oflag &= ~OPOST;

        tcsetattr (sd, TCSANOW, &options);

        sleep(2); //required to make flush work, for some reasons
        tcflush(sd,TCIOFLUSH);

        ioctl (sd, TIOCMGET, &status);

        status |= TIOCM_DTR;
        status |= TIOCM_RTS;

        ioctl (sd, TIOCMSET, &status);

        usleep(10000);

        fprintf(stderr,"done\n");

        return sd;
}

void Keysight_34401A_setup(int sd)
{
   char buffer[BUFF_SIZE];
   int length = 0; 

   send_message(":SYST:REM");

/*
   send_message("*RST;*CLR");
*/

   send_message("*IDN?");
   receive_message(buffer);

   do {
     send_message(":SYST:ERR?");

     length = receive_message(buffer);

     usleep(sleeptime);

   } while ((strncmp(buffer,"+0",2) != 0) && (length > 0));
   /* +0 means no error */

}

int continuity()
{
  int length;
  char buffer[BUFF_SIZE];

  send_message("MEAS:CONT?");

  length = receive_message(buffer);

  return length;

}

int resistor()
{
  int length;
  char buffer[BUFF_SIZE];

  send_message("MEAS:RES?");

  length = receive_message(buffer);

  return length;

}

int diode()
{
  int length;
  char buffer[BUFF_SIZE];

  send_message("MEAS:DIOD?");

  length = receive_message(buffer);

  return length;

}

int voltage()
{
  int length;
  char buffer[BUFF_SIZE];

  send_message("MEAS:VOLT:DC?");

  length = receive_message(buffer);

  return length;

}

void go(int pina, int pinb, char *T)
{
   int basepin = 64;

   digitalWrite (basepin+pina, 1);
   digitalWrite (basepin+pinb, 1);

   /* Wait for relays to get a good contact before sending SCPI command */
   usleep(sleeptime);
    
   switch(*T)
   {
   case 'C':
   case 'T':
     continuity();
     break;
   case 'D':
     diode();
     break;
   case 'R':
     resistor();
     break;
   }

   digitalWrite (basepin+pina, 0);
   digitalWrite (basepin+pinb, 0);

}

void setup()
{
   int pins = 8; /* Number of register pins */

   printf ("MDA ICT test\n") ;

   wiringPiSetup () ;

   /*
    Use wiringPi pins 0, 1 & 2 for data, clock and latch respectively
    sr595Setup(basepin_min_is_64, num_of_pins, data, clock, latch) ;
   */
   sr595Setup(64, pins, 12, 14, 2);

}

MYSQL_RES *mysql_setup()
{

   MYSQL_RES *res;

   char query[128];
   char *server = "localhost";
   char *user = "root";
   char *password = "raspberrypi";
   char *database = "mda";
   char *table = "ict_main";
   int ql = 0;

   mysql_init(&mysql);

   /* Connect to database */
   if (!mysql_real_connect(&mysql, server, user, password, database, 0, NULL, 0)) {
      fprintf(stderr, "%s\n", mysql_error(&mysql));
      exit(1);
   }

   sprintf(query,"SELECT * FROM %s%n", table,&ql);

   /*mysql_real_query(&mysql,query,(unsigned int)strlen(query));*/
   mysql_real_query(&mysql,query,ql);

   res = mysql_use_result(&mysql);

   return res;
}

void loop(MYSQL_RES *res)
{
  MYSQL_FIELD *field;
  MYSQL_ROW row;

  unsigned int num_fields;
  num_fields = mysql_num_fields(res);
  
  while((field = mysql_fetch_field(res))) 
    {
      printf("%s ", field->name);
    }	
  printf("\n");
   
  while((row = mysql_fetch_row(res))) {
    
    unsigned long *lengths;
    int i;
    lengths = mysql_fetch_lengths(res);
    
    for(i = 0; i < num_fields; i++)
      {
	printf("[%.*s] ", (int) lengths[i], ((int) lengths[i]>0 ? row[i] : "NULL"));
      }

    /* minus 1 because the first item is 0 */
    go(atoi(row[8])-1, atoi(row[9])-1, row[3]);
      
    /* Exchange the polarity of the probe for testing diode*/
    if (strncmp(row[3],"D",1) == 0)
      go(atoi(row[9])-1, atoi(row[8])-1, row[3]);
  }

}

int main()
{
  MYSQL_RES *res;

  setup();

  sd = serial_init(9600);

/*
  open_com_port(NULL);
*/

  Keysight_34401A_setup(sd);

  res = mysql_setup();

  loop(res);
   
  mysql_free_result(res);

  return 0;
} 

