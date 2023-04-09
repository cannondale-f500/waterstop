/*******************************
* Waterstop
* Description: This program is programmed to communicate via RS232 with the Judo Zewa-Wasserstop
* First release: 20.03.2019
* Version: 0.1
* Copyright (C) 2019 by Thomas Schütz
*    This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*
*************************/
  

#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <mysql.h>
#include <string.h>
#include <stdbool.h>
//Serial Port HowTo: https://www.cmrr.umn.edu/~strupp/serial.html

void help();
int valvetoggle();
int read_bdata();
void printdata();

unsigned char bdata[100];
char write_akt1[2] = {0xAA,0x01};
char write_akt2[2] = {0xAA,0x02};
int fd;


int main(int argc, char* argv[])
{
	int statbdata;
	int i=0;
	int j=0;
	bool opt_S=0;
	bool opt_v=0;
	bool opt_t=0;
	bool opt_c=0;
	bool opt_o=0;
	struct termios SerialPortSettings;

	for(i=0;i<argc;i++)
	{	//printf("\n%s",argv[i]);
		//if(strpbrk(argv[i],"-h")||strpbrk(argv[i],"-H"))
		if(strcmp(argv[i],"-h")==0||strcmp(argv[i],"-H")==0)
		{
			help();
			return 0;
		}
		if(strcmp(argv[i],"-S")==0)
		{
			opt_S=1;;
		}
                if(strcmp(argv[i],"-v")==0)
                {
                        opt_v=1;
                }
                if(strcmp(argv[i],"-t")==0)
                {
                        opt_t=1;
                }
                if(strcmp(argv[i],"-c")==0)
                {
			opt_c=1;
                }
                if(strcmp(argv[i],"-o")==0)
                {
                        opt_o=1;
                }
	}
	//printf("\n%d",argc);
	fd = open(argv[1],O_RDWR|O_NOCTTY|O_NDELAY);//|O_NOCTTY|O_NDELAY); "/dev/ttyUSB0"
	if(fd == -1)
		printf("\n Error! in Opening %s",argv[i]);
	else
	{	printf("\n %s Opened Successfully",argv[1]);
  		tcgetattr(fd, &SerialPortSettings);//Get Settings of Port
		cfsetispeed(&SerialPortSettings,B9600);//Set Baudrate of Read Speed
		cfsetospeed(&SerialPortSettings,B9600);//Set Baudrate of Write Speed
		SerialPortSettings.c_cflag &= ~PARENB; //No Parity
		SerialPortSettings.c_cflag &= ~CSTOPB; //Stop bits = 1
		SerialPortSettings.c_cflag &= ~CSIZE;  //Clears the Mask
		SerialPortSettings.c_cflag |= CS8;    //Set the data bits = 8
		SerialPortSettings.c_cflag &= ~CRTSCTS;//Turn Off RTS/CTS
		SerialPortSettings.c_cflag |= CREAD | CLOCAL;//Turn on Serial Port receiver
		SerialPortSettings.c_lflag &= ~(IXON | IXOFF | IXANY);//Turn off software based flow control
		SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);//NON Cannonical mode is crecommended (Raw Input)
		SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/
//		SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ISIG);//NON Cannonical mode is crecommended (Raw Input)
//		SerialPortSettings.c_lflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
//		SerialPortSettings.c_cc[VMIN] = 1; /* Read at least 10 characters */
//		SerialPortSettings.c_cc[VTIME] = 0; /* Wait indefinetly   */

		tcflush(fd,TCIFLUSH);
		if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0)//Set Settings
		{	printf("\n ERROR ! in Settings attributes");
		}
		else
		{
			printf("\n Baud_Rate=9600 StopBits =1 Parity = none");
		}

		statbdata=read_bdata();//Read Betriebsdaten

		if(statbdata==1)//Check if Checksum was good
		{
			printf("\nChecksum OK");

			if(opt_t==1)//Toggle Valve
        	        {       //valvetoggle(); DEAKTIVIERT
	                }
			if(opt_v==1)//Print Data
			{	printdata();
			}
			if(opt_c==1)
			{       if(bdata[2]&&0b00000001==1)
				{	//Do nothing if valve is closed
				}
				else
				{	//valvetoggle(); DEAKTIVIERT
				}
			}
                        if(opt_o==1)
                        {       if(bdata[2]&&0b00000001==1)
                                {       //valvetoggle(); DEAKTIVIERT
                                }
                                else
                                {       //Do nothing if valve is opened
                                }
                        }

			if(opt_S==1)//Write to MySQL
			{
				MYSQL *con = mysql_init(NULL);

				if (con == NULL) 
  				{
      					fprintf(stderr, "\n%s", mysql_error(con));
      					return -1;
	  			}

  				if (mysql_real_connect(con, "bree.lan", "<name>", "<password>", NULL, 0, NULL, 0) == NULL) 
  				{
      					fprintf(stderr, "\n%s", mysql_error(con));
      					mysql_close(con);
	      				return 0;
  				}

  				if (mysql_query(con, "CREATE DATABASE waterstop")) 
  				{
	      				fprintf(stderr, "\n%s", mysql_error(con));
      					mysql_close(con);
      					return 0;
  				}

				mysql_close(con);
			}

		}
		else
		{
			printf("\nChecksum Error");
		}

	}
	close(fd);
	printf("\n");
	return 0;
}

int valvetoggle()
{
	int  bytes_written  =  0 ;
	bytes_written = write(fd,write_akt2,sizeof(write_akt2)); //Aktion Ventil Öffnen/Schließen
}

int read_bdata()
{
	int bytes_read = 0;
	unsigned char read_buffer[100];
	unsigned char checksum=0;
	int  bytes_written  =  0 ;
	int i=0;
	int j=0;

        bytes_written = write(fd,write_akt1,sizeof(write_akt1)); //Aktion Betriebsdaten lesen

//        fcntl(fd, F_SETFL, FNDELAY);

        bytes_read=read(fd,&read_buffer,sizeof(read_buffer));
        while(i<48)
        {
	        if(bytes_read>0)
                {
//                	printf("\nRead_Buffer: %d ",bytes_read);
                        for(j=0;j<bytes_read;j++)
                        {       bdata[i]=read_buffer[j];
//                	        printf("%02hhx ",read_buffer[j]);
//                                printf("%02hhx ",bdata[i]);
                                i++;
                        }
                }
                bytes_read=read(fd,&read_buffer,sizeof(read_buffer));
//                bytes_read=fcntl(fd,&read_buffer,8);
	}
        printf("\nDaten:  %d ",i);
        for(j=0;j<=48;j++)
        {
        	printf("%02hhx ", bdata[j]);
        }

        for(j=1;j<=46;j++)
        {       checksum=checksum+bdata[j];
        }

	printf("\n%02hhx %02hhx", bdata[47],checksum);

	if(checksum==bdata[47])
		return 1;
	else
		return 0;
}

void printdata()
{
        unsigned long int Wassermenge;
        float Batspg, Notspg;
	int j;

        Wassermenge=(bdata[40]+(bdata[41]*256)+(bdata[42]*2562)+(bdata[43]*2563))*100;
        printf("\nGesamtwassermenge: %d l", Wassermenge);
        Batspg=bdata[4] * 0.07906;
        printf("\nBatteriespannung: %f V", Batspg);
        Notspg=bdata[5] * 0.1556;
        printf("\nNotstrommodulspannung: %f V", Notspg);

        for(j=6;j<12;j++)
        {
        	printf("\ngespeicherte Abschaltungen Wassermenge Einstellung%d:  %d",j-5,bdata[6]);
        }
        for(j=12;j<18;j++)
        {
        	printf("\ngespeicherte Abschaltungen Durchfluss Einstellung%d:   %d",j-11,bdata[6]);
        }
        for(j=18;j<24;j++)
        {
                printf("\ngespeicherte Abschaltungen Entnahmezeit Einstellung%d: %d",j-17,bdata[6]);
        }

        printf("\nAnzahl Abschaltungen im Urlaubsmodus: %d", bdata[24]);
        printf("\nAnzahl der Abschaltungen durch Leckagesensor: %d", bdata[25]);
        printf("\nAnzahl Störmeldungen Fehler2 (Motor oder Nockenschalter defekt): %d", bdata[26]);
        printf("\nAnzahl Störmeldungen Fehler3 (Verbindungsfehler Notstrom-ZEWA): %d", bdata[27]);

        printf("\n Wassermenge[l] Durchfluss[l/h] Entnahmezeit[0,5s]");
        printf("\nmax.:     %05d           %05d              %05d ",(bdata[28]+bdata[29]*256),(bdata[30]+bdata[31]*256),(bdata[32]+bdata[33]*256));
        printf("\nMesswert: %05d           %05d              %05d ",(bdata[34]+bdata[35]*256),(bdata[36]+bdata[37]*256),(bdata[38]+bdata[39]*256));

                        //Statusbyte 0
                        printf("\nKugelventil:       ");
                        if(bdata[2]&&0b00000001==1)
                                printf("geschlossen");
                        else
                                printf("offen");
                        printf("\nWassermenge:       ");
                        if(bdata[2]&&0b00000010==1)
                                printf("Überschreitung");
                        else
                                printf("keine Überschreitung");
                        printf("\nDurchfluss:        ");
                        if(bdata[2]&&0b00000100==1)
                                printf("Überschreitung");
                        else
                                printf("keine Überschreitung");
                        printf("\nEntnahmedauer:     ");
                        if(bdata[2]&&0b00001000==1)
                                printf("Überschreitung");
                        else
                                printf("keine Überschreitung");
                        printf("\nUrlaubsmodus:      ");
                        if(bdata[2]&&0b00010000==1)
                                printf("aktiv");
                        else
                                printf("nicht aktiv");
                        printf("\nStandby-Modus:     ");
                        if(bdata[2]&&0b00100000==1)
                                printf("aktiv");
                        else
                                printf("nicht aktiv");
                        printf("\nStörung:           ");
                        if(bdata[2]&&0b01000000==1)
                                printf("Störung");
                        else
                                printf("keine Störung");
                        printf("\nKV-Motor:          ");
                        if(bdata[2]&&0b10000000==1)
                                printf("eingeschaltet");
                        else
                                printf("ausgeschaltet");

                        //Statusbyte 1
                        printf("\nSpannungsvers.:    ");
                        if(bdata[3]&&0b00000001==1)
                                printf("Batterieversorgung");
                        else
                                printf("Netzversorgung");
                        printf("\nBatteriezustand:   ");
                        if(bdata[3]&&0b00000010==1)
                                printf("Batterie leer");
                        else
                                printf("Batterie i.O.");
                        printf("\nDatenverbindung:   ");
                        if(bdata[3]&&0b00000100==1)
                                printf("keine Datenverbindung");
                        else
                                printf("Datenverbindung i.O.");
                        printf("\nLeckagesensor:     ");
                        if(bdata[3]&&0b00001000==1)
                                printf("Abschaltung");
                        else
                                printf("keine Abschaltung");
                        printf("\next. I/O:          ");
                        if(bdata[3]&&0b00010000==1)
                                printf("Eingangssignal");
                        else
                                printf("kein Eingangssignal");
                        printf("\nRel 1 ext. auf/zu: ");
                        if(bdata[3]&&0b00100000==1)
                                printf("KV geschlossen");
                        else
                                printf("KV geöffnet");
                        printf("\nRel 2 Störung:     ");
                        if(bdata[3]&&0b01000000==1)
                                printf("Betrieb");
                        else
                                printf("Störung, keine Span.");
                        printf("\nRel 3 100l Imp.:   ");
                        if(bdata[3]&&0b10000000==1)
                                printf("Impuls nach 100l");
                        else
                                printf("kein Impuls");

                        printf("\n");

}

void help()
{	printf("\nUsage:");
	printf("\n waterstop <COM-Port> [-c] [-o] [-v] [-S]");
	printf("\n -c   Close Valve");
	printf("\n -o   Open Valve");
	printf("\n -t   Toggle Valve");
	printf("\n -v   Show operating data");
	printf("\n -S   Write to mySQLServer");
	printf("\nExample:");
	printf("\n ./waterstop /dev/ttyUSB0 ");
	printf("\n");
	printf("\nWaterstop Version:");
	printf("\n0.0.0 ");
	printf("\n12.06.2019");
	printf("\n(C) Thomas Schütz");
        printf("\n");
}
