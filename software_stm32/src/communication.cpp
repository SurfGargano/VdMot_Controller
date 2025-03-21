/**HEADER*******************************************************************
  project : VdMot Controller
  author : Lenti84
  Comments:
  Version :
  Modifcations :
***************************************************************************
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE DEVELOPER OR ANY CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
**************************************************************************
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License.
  See the GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  Copyright (C) 2021 Lenti84  https://github.com/Lenti84/VdMot_Controller
*END************************************************************************/

#define SERIAL_TX_BUFFER_SIZE 1024
#define SERIAL_RX_BUFFER_SIZE 1024

#include <Arduino.h>
#include "hardware.h"
#include "app.h"
#include "motor.h"
#include "communication.h"
#include "owDevices.h"
#include "eeprom.h"
#include "DallasTemperature.h"
#include <string.h>
#include <stdio.h>

// DEBUG
#ifdef commDebug
	#define commdbg_print(format, ...) COMM_DBG.print(format, ##__VA_ARGS__)
	#define commdbg_println(format, ...) COMM_DBG.println(format, ##__VA_ARGS__)
#else
	#define commdbg_print(format, ...)	(void)0
	#define commdbg_println(format, ...)	(void)0
#endif

#define COMM_MAX_CMD_LEN		15			// max length of a command without arguments
#define COMM_ARG_CNT			 2			// number of allowed command arguments
#define COMM_MAX_ARG_LEN		20			// max length of one command argument
#define SEND_BUFFER_LEN			100


void setValveIDSensor (uint8_t idx,char* pString, uint8_t* pSensor)
{
	uint8_t currAddress[8];
	uint16_t sum=0;

	if (idx >= 0 && idx < ACTUATOR_COUNT) {
		// first sensor address
		if(strlen(pString)==23) {
			// from end to beginning
			for(uint8_t xx=0;xx<8;xx++) {
				char* cmdptr=strchr(pString,'-');
				uint8_t code=strtol(pString, &cmdptr, 16);
				currAddress[xx] = code;
				sum+=code;
				pString+=3;
			}
			
			if (sensors.validAddress(currAddress)) {
				memcpy(pSensor,currAddress,8);	
			} else {
				// match sensor address to valve struct at runtime, otherwise restart needed
				if (sum==0) memset(pSensor,0x0,8);
			}
			eeprom_changed();
			//app_match_sensors();
		}
	}
}

void sensorID2Ascii (uint8_t* sID, char* idBuf)
{
	char valBuffer[10];
	// 8 Byte adress
	for (uint8_t i = 0; i < 8; i++)
	{
		memset(valBuffer,0x0,sizeof(valBuffer));
		if (*sID < 16) strcat(idBuf, "0");
		itoa(*sID, valBuffer, 16);      
		strcat(idBuf, valBuffer);
		if (i<7) strcat(idBuf, "-");
		sID++;
	}
}

void sendValvesTempsId (uint8_t idx, char delimiter)
{
	char mySendBuffer[30];
	// 1st sensor 8 Byte adress
	uint8_t y = myvalves[idx].sensorindex1;	
				
	if(y!=VALVE_SENSOR_UNKNOWN) {
		memset (mySendBuffer,0x0,sizeof(mySendBuffer));				// reset sendbuffer
		// 8 Byte adress
		sensorID2Ascii (&tempsensors[y].address[0],mySendBuffer);
		COMM_SER.print(mySendBuffer);
	}
	else {
		COMM_SER.print("00-00-00-00-00-00-00-00");
	}
	COMM_SER.print(delimiter);

	// 2nd sensor 8 Byte adress
	y = myvalves[idx].sensorindex2;
	if(y!=VALVE_SENSOR_UNKNOWN) {
		memset (mySendBuffer,0x0,sizeof(mySendBuffer));				// reset sendbuffer
		// 8 Byte adress
		sensorID2Ascii (&tempsensors[y].address[0],mySendBuffer);
		COMM_SER.print(mySendBuffer);
	}
	else {
		COMM_SER.print("00-00-00-00-00-00-00-00");
	}
}


void communication_setup (void) {
	
	// UART to ESP32
	COMM_SER.setRx(PA10);			//STM32F401 blackpill USART1 RX PA10
	COMM_SER.setTx(PA9);			//STM32F401 blackpill USART1 TX PA9
	COMM_SER.begin(115200, SERIAL_8N1);
	while(!COMM_SER);
	//COMM_SER.println("alive");COMM_SER.flush();
	#ifdef commDebug
		COMM_DBG.println("SERIAL_BUFFER_SIZE TX="+String(SERIAL_TX_BUFFER_SIZE)+" RX="+String(SERIAL_RX_BUFFER_SIZE)); 
	#endif
}


/**
  * @brief  Handler
  * @param  None
  * @retval None
  */
int16_t communication_loop (void) {
	static char buffer[1000];
    static char *bufptr = buffer;
    static unsigned int buflen = 0;
    int availcnt = 0;
    unsigned int found = 0;

	char*       cmdptr;
    char*	    cmdptrend;
    char        cmd[10];
	
    char		arg0[ARG_SIZE];	
    char        arg1[ARG_SIZE];		
	char        arg2[ARG_SIZE];
	char        arg3[ARG_SIZE];	
	char        arg4[ARG_SIZE];	

	char *argptr[NO_OF_ARGS]={arg0,arg1,arg2,arg3,arg4};

	uint8_t		argcnt = 0;
    
	uint16_t	x = 0;
	uint32_t	xu32 = 0;
	uint16_t	y = 0;
	char sendbuffer[60];
    char valbuffer[10];

	DeviceAddress currAddress;
	uint8_t numberOfDevices = 0;
	uint8_t calibration = 0;

	availcnt = COMM_SER.available(); 
    if(availcnt>0)
    {    
		if(availcnt>990) availcnt = 990;			// limit massive data streams, should not happen

        for (int c = 0; c < availcnt; c++)
        {           
            *bufptr++ = (char) COMM_SER.read();
            buflen++;
        }
        if (buflen>=sizeof(buffer)) {
            buffer[sizeof(buffer)-1] = '\n';
        }
    }

	// if there is a little in the buffer
    if(buflen >= 5) 
    {
        for (unsigned int c = 0; c < buflen; c++)
        {           
            if(buffer[c] == '\n') 
            {
				if(buffer[c-1]=='\r') buffer[c-1] = '\0';
                else buffer[c] = '\0';
                //COMM_SER.print("recv "); COMM_SER.println(buffer);
                found = 1;

                buflen = 0;           	// reset counter
                bufptr = buffer;    	// reset ptr
            }
        }
    }

	// was something received
	if(found) {

		// devide buffer into command and data
		// ****************************************
		cmdptr = buffer;

		for(uint8_t xx=0;xx<NO_OF_ARGS+1;xx++){
			cmdptrend = strchr(cmdptr,' ');
			if (cmdptrend!=NULL) {
				*cmdptrend = '\0';
				if(xx==0) strncpy(cmd,cmdptr,sizeof(cmd)-1);		// command
				else  { 
					strncpy(argptr[xx-1],cmdptr,ARG_SIZE-1); 
					argcnt=xx;	
				} 	// arguments

				/*else if(xx==1) { strncpy(argptr[0],cmdptr,ARG_SIZE-1); argcnt=1;	} 	// 1st argument
				else if(xx==2) { strncpy(argptr[1],cmdptr,ARG_SIZE-1); argcnt=2;	} 	// 2nd argument
				else if(xx==3) { strncpy(argptr[2],cmdptr,ARG_SIZE-1); argcnt=3;	} 	// 3nd argument
				else if(xx==4) { strncpy(argptr[3],cmdptr,ARG_SIZE-1); argcnt=4;	} 	// 4th argument*/
				cmdptr = cmdptrend + 1;
			}
		}

		// evaluate data
		// ****************************************************************************************
		// clear sendbuffer, otherwise there is something from older commands
		memset (sendbuffer,0x0,sizeof(sendbuffer));	
	/*	#ifdef commDebug
			COMM_DBG.print("cmd ");
			COMM_DBG.println(cmd); 
		#endif
	*/
		// set target position
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		if(memcmp(APP_PRE_SETTARGETPOS,&cmd[0],5) == 0) {
			#ifdef commDebug
				COMM_DBG.println("set target pos"); 
			#endif

			x = atoi(argptr[0]);
			y = atoi(argptr[1]);

			if(argcnt == 2) {				
				#ifdef commDebug 
					COMM_DBG.println("comm: set target position"); 
				#endif
				if (y >= 0 && y <= 100 && x < ACTUATOR_COUNT) 
				{
					if (!myvalvemots[x].calibration)  // wdu ???
						myvalvemots[x].target_position = (byte) y;
					COMM_SER.println(APP_PRE_SETTARGETPOS);
				}
			}
			else {
				#ifdef commDebug 
					COMM_DBG.println("to few arguments");
				#endif
			}
		}


		// get target position
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		if(memcmp(APP_PRE_GETTARGETPOS,&cmd[0],5) == 0) {
			#ifdef commDebug 
				COMM_DBG.println("get target pos");
			#endif

			x = atoi(argptr[0]);

			if(argcnt == 1) {				
				#ifdef commDebug 
					COMM_DBG.println("comm: got target position request");
				#endif
				if (x >= 0 && x < ACTUATOR_COUNT) 
				{	
					COMM_SER.print(APP_PRE_GETTARGETPOS);
					COMM_SER.print(" ");
					COMM_SER.print(x, DEC);
					COMM_SER.print(" ");
					COMM_SER.print(myvalvemots[x].target_position, DEC);
					COMM_SER.println(" ");
				}
			}
			else {
				#ifdef commDebug 
					COMM_DBG.println("to few arguments");
				#endif
			}
		}


		// get valve data (actual position, target position, meancurrent, status, temperature)
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_GETVLVDATA,&cmd[0],5) == 0) {
			#ifdef commDebug  
				//COMM_DBG.println("get valve data");
			#endif

			x = atoi(argptr[0]);

			if(argcnt == 1) {				
				if (x < ACTUATOR_COUNT) 
				{
					memset (sendbuffer,0x0,sizeof(sendbuffer));			// sendbuffer reset
					//COMM_SER.println("sending new target value");
                    
					strcat(sendbuffer, APP_PRE_GETVLVDATA);
					strcat(sendbuffer, " ");
                    
					itoa(x, valbuffer, 10);		// valve nr
					strcat(sendbuffer, valbuffer);
					strcat(sendbuffer, " ");       
                    
					itoa(myvalvemots[x].actual_position, valbuffer, 10);      
					strcat(sendbuffer, valbuffer);
					strcat(sendbuffer, " ");
                    
					itoa(myvalvemots[x].meancurrent, valbuffer, 10);      
					strcat(sendbuffer, valbuffer);
					strcat(sendbuffer, " ");
                    
					calibration = myvalvemots[x].status;
					if (myvalvemots[x].calibration) calibration|= 0x80;

					itoa(calibration, valbuffer, 10);      
					strcat(sendbuffer, valbuffer);
					strcat(sendbuffer, " ");
                    
					if(myvalves[x].sensorindex1<MAXONEWIRECNT)
					{
						itoa(tempsensors[myvalves[x].sensorindex1].temperature, valbuffer, 10);      
						strcat(sendbuffer, valbuffer);
					}
					else strcat(sendbuffer, "-500");
					
					strcat(sendbuffer, " ");

					if(myvalves[x].sensorindex2<MAXONEWIRECNT)
					{
						itoa(tempsensors[myvalves[x].sensorindex2].temperature, valbuffer, 10);      
						strcat(sendbuffer, valbuffer);
					}
					else strcat(sendbuffer, "-500");

					strcat(sendbuffer, " ");
					itoa(myvalves[x].movements, valbuffer, 10);  
					strcat(sendbuffer, valbuffer);

					strcat(sendbuffer, " ");
					itoa(myvalvemots[x].opening_count, valbuffer, 10);  
					strcat(sendbuffer, valbuffer);
					strcat(sendbuffer, " ");
					itoa(myvalvemots[x].closing_count, valbuffer, 10);  
					strcat(sendbuffer, valbuffer);
					strcat(sendbuffer, " ");
					itoa(myvalvemots[x].deadzone_count, valbuffer, 10);  
					strcat(sendbuffer, valbuffer);
					strcat(sendbuffer, " ");
					itoa(myvalvemots[x].calibRetries, valbuffer, 10);  
					strcat(sendbuffer, valbuffer);
					// end
					strcat(sendbuffer, " ");
					COMM_SER.println(sendbuffer);			
				}
			}
			else { 
				#ifdef commDebug 
					COMM_DBG.println("to few arguments");
				#endif
			}
		}

		// get valve present
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_GETVLSTATUS,&cmd[0],5) == 0) {
			#ifdef commDebug 
				COMM_DBG.println("cmd: get sensor status");
			#endif
			
			COMM_SER.print(APP_PRE_GETVLSTATUS);
			COMM_SER.print(" ");
			COMM_SER.print(ACTUATOR_COUNT, DEC);
			COMM_SER.print(" ");

			for (uint8_t xx=0;xx<ACTUATOR_COUNT;xx++) {
				memset(sendbuffer,0x0,sizeof(sendbuffer));
				itoa(myvalvemots[xx].status, sendbuffer, 10);				
				COMM_SER.print(sendbuffer);
				if (x<ACTUATOR_COUNT-1) COMM_SER.print(",");
			}	
			COMM_SER.println(" ");
		}

		// get temp onewire sensor count
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_GETONEWIRECNT,&cmd[0],5) == 0) {
			
			if(argcnt == 0) {
				COMM_SER.print(APP_PRE_GETONEWIRECNT);
				COMM_SER.print(" ");			
				COMM_SER.print(noOfDS18Devices, DEC);
				COMM_SER.println(" ");
			}	
			if(argcnt == 1) {	
				x = atoi(argptr[0]);
				if (x==255) {
					// get all onewire detected sensor
					COMM_SER.print(APP_PRE_GETONEWIRECNT);
					COMM_SER.print(" ");		
					COMM_SER.print(noOfDS18Devices, DEC);
					if (noOfDS18Devices>0) COMM_SER.print(" ");
					for (uint8_t i=0;i<noOfDS18Devices;i++){
						memset (sendbuffer,0x0,sizeof(sendbuffer));				// reset sendbuffer
						// 8 Byte adress
						sensorID2Ascii (&tempsensors[i].address[0],sendbuffer);
						COMM_SER.print(sendbuffer);
						if (i<noOfDS18Devices-1) COMM_SER.print(",");
					}	
					COMM_SER.println(" ");
				}
			
			} 		
		}

		// get temp onewire sensor data of sensor x (adress and temperature)
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_GETONEWIREDATA,&cmd[0],5) == 0) {

			x = atoi(argptr[0]);

			if(argcnt == 1) {

				if(x<MAXONEWIRECNT)
				{
					memset (sendbuffer,0x0,sizeof(sendbuffer));				// reset sendbuffer
					// 8 Byte adress
					sensorID2Ascii (&tempsensors[x].address[0],sendbuffer);
					
					strcat(sendbuffer," ");
					// temperature
					itoa(tempsensors[x].temperature, valbuffer, 10);      
					strcat(sendbuffer, valbuffer);
				}
				else strcat(sendbuffer, "0");

			}
			else strcat(sendbuffer, "0");

			COMM_SER.print(APP_PRE_GETONEWIREDATA);
			COMM_SER.print(" ");			
			COMM_SER.print(sendbuffer);
			COMM_SER.println(" ");			
		}


		// get 1st and 2nd onewire sensor address for valve x
		// example answer: gvlon 1 28-84-37-94-97-FF-03-23 00-00-00-00-00-00-00-00
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_GETONEWIRESETT,&cmd[0],5) == 0) {
			#ifdef commDebug 
				COMM_DBG.print("cmd: get 1st and 2nd onewire sensor addresses");
			#endif

			x = atoi(argptr[0]);

			if(argcnt == 1) {

				if(x<ACTUATOR_COUNT) {
					// answer begin
					COMM_SER.print(APP_PRE_GETONEWIRESETT);
					COMM_SER.print(" ");

					// valve index
					COMM_SER.print(x, DEC);
					COMM_SER.print(" ");
					
					sendValvesTempsId(x,' ');
					// finish answer
					COMM_SER.println(" ");

				}
				else {
					if (x==255) {
						// answer begin
						COMM_SER.print(APP_PRE_GETONEWIRESETT);
						COMM_SER.print(" ");
						// valve index
						uint8_t z=ACTUATOR_COUNT;
						//z=4;
						COMM_SER.print(z, DEC);
						COMM_SER.print(" ");
						// 1st sensor 8 Byte adress
						for (uint8_t i=0;i<z;i++) {
							sendValvesTempsId(i,',');
							if (i<z-1) COMM_SER.print(",");
						}
						// finish answer
						COMM_SER.println(" ");
					}
					else {
						#ifdef commDebug 
							COMM_DBG.println(" - error");
						#endif

						COMM_SER.print(APP_PRE_GETONEWIREDATA);
						COMM_SER.println(" error ");
					}
				}
			}
			else {
				#ifdef commDebug 
					COMM_DBG.println(" - error");
				#endif
				COMM_SER.print(APP_PRE_GETONEWIREDATA);
				COMM_SER.println(" error ");			
			}			
		}

		// get volt onewire sensor count
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_GETOWVOLTCNT,&cmd[0],5) == 0) {
			if(argcnt == 0) {
				COMM_SER.print(APP_PRE_GETOWVOLTCNT);
				COMM_SER.print(" ");			
				COMM_SER.print(noOfDS2438Devices, DEC);
				COMM_SER.println(" ");
			}	
			if(argcnt == 1) {	
				x = atoi(argptr[0]);
				if (x==255) {
					// get all onewire detected sensor
					COMM_SER.print(APP_PRE_GETOWVOLTCNT);
					COMM_SER.print(" ");		
					COMM_SER.print(noOfDS2438Devices, DEC);
					if (noOfDS2438Devices>0) COMM_SER.print(" ");
					for (uint8_t i=0;i<noOfDS2438Devices;i++){
						memset (sendbuffer,0x0,sizeof(sendbuffer));				// reset sendbuffer
						// 8 Byte adress
						sensorID2Ascii (&voltsensors[i].address[0],sendbuffer);
						COMM_SER.print(sendbuffer);
						if (i<noOfDS2438Devices-1) COMM_SER.print(",");
					}	
					COMM_SER.println(" ");
				}
			
			} 		
		}

		// get volt onewire sensor data of sensor x (adress and temperature)
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_GETOWVOLTDATA,&cmd[0],5) == 0) {

			x = atoi(argptr[0]);

			if(argcnt == 1) {

				if(x<MAXDS2438CNT)
				{
					memset (sendbuffer,0x0,sizeof(sendbuffer));				// reset sendbuffer
					// 8 Byte adress
					sensorID2Ascii (&voltsensors[x].address[0],sendbuffer);
					
					strcat(sendbuffer," ");
					// volt
					itoa(voltsensors[x].vad, valbuffer, 10);      
					strcat(sendbuffer, valbuffer);
				}
				else strcat(sendbuffer, "0");

			}
			else strcat(sendbuffer, "0");

			COMM_SER.print(APP_PRE_GETOWVOLTDATA);
			COMM_SER.print(" ");			
			COMM_SER.print(sendbuffer);
			COMM_SER.println(" ");			
		}



		// start new onewire sensor search
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_SETONEWIRESEARCH,&cmd[0],5) == 0) {
			#ifdef commDebug 
				COMM_DBG.println("start new 1-wire search");
			#endif
			temp_command(TEMP_CMD_NEWSEARCH);
			COMM_SER.println(APP_PRE_SETONEWIRESEARCH);
		}
		

		// set valve learning time
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_SETLEARNTIME,&cmd[0],5) == 0) {
			#ifdef commDebug 
				COMM_DBG.print("set valve learning time to ");
			#endif
			xu32 = atol(argptr[0]);

			if(argcnt == 1 && xu32 >= 0) {
				if(app_set_learntime(xu32) == 0) {
					COMM_SER.println(APP_PRE_SETLEARNTIME);
					#ifdef commDebug 
						COMM_DBG.println(xu32, DEC);
					#endif
				}
				else { 
					#ifdef commDebug 
						COMM_DBG.println("- error");
					#endif
				}
			}
			else {
				#ifdef commDebug 
					COMM_DBG.println("- error");
				#endif
			}
		}


		// set valve learning movements
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_SETLEARNMOVEM,&cmd[0],5) == 0) {
			#ifdef commDebug
				COMM_DBG.print("set valve learning movements to ");
			#endif
			x = atoi(argptr[0]);

			if(argcnt == 1 && x >= 0) {
				if(app_set_learnmovements(x) == 0) {
					#ifdef commDebug 
						COMM_DBG.println(x, DEC);
					#endif
					eep_content.numberOfMovements=x;
					eeprom_changed();
					COMM_SER.println(APP_PRE_SETLEARNMOVEM);
				}
				else { 
					#ifdef commDebug 
						COMM_DBG.println("- error");
					#endif
				}
			}
			else {
				#ifdef commDebug 
					COMM_DBG.println("- error");
				#endif
			}
		}

		// get valve learning movements
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_GETLEARNMOVEM,&cmd[0],5) == 0) {
			#ifdef commDebug 
				COMM_DBG.print("get valve learning movements");
			#endif
			COMM_SER.print(APP_PRE_GETLEARNMOVEM);
			COMM_SER.print(" ");			
			COMM_SER.print(learning_movements, DEC);
			COMM_SER.println(" ");			
			
		}


		// ESPalive
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp("ESP",&cmd[0],3) == 0) {	
			#ifdef commDebug 
				COMM_DBG.println("received ESPalive 22");
			#endif		
			if (buflen >= 8 && memcmp("ESPalive",cmd,8) == 0) {
				#ifdef commDebug 
					COMM_DBG.println("received ESPalive");
				#endif
			}			 
		}


		// set first sensor index of valve
		// x - valve index
		// y - temp sensor index
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_SET1STSENSORINDEX,&cmd[0],5) == 0) {
			x = atoi(argptr[0]); // valve index
			y = atoi(argptr[1]); // temp sensor index

			if(argcnt == 2) {				
				#ifdef commDebug 
					COMM_DBG.println("comm: set 1st sensor index");
				#endif
				if (x >= 0 && x < ACTUATOR_COUNT && y < MAXONEWIRECNT) 
				{
					if (noOfDS18Devices > 0) 
					{
						// write sensor address code to eeprom layout mirror
						sensors.getAddress(currAddress, y);
						eep_content.owsensors1[x].familycode = currAddress[0];
						eep_content.owsensors1[x].romcode[0] = currAddress[1];
						eep_content.owsensors1[x].romcode[1] = currAddress[2];
						eep_content.owsensors1[x].romcode[2] = currAddress[3];
						eep_content.owsensors1[x].romcode[3] = currAddress[4];
						eep_content.owsensors1[x].romcode[4] = currAddress[5];
						eep_content.owsensors1[x].romcode[5] = currAddress[6];
						eep_content.owsensors1[x].crc = currAddress[7];

						// write index to valves structure
						myvalves[x].sensorindex1 = (byte) y;

						eeprom_changed();
						COMM_SER.println(APP_PRE_SET1STSENSORINDEX);
					}					
				}
			}
			else {
				#ifdef commDebug 
					COMM_DBG.println("to few arguments");
				#endif
			}
		}


		// set second sensor index of valve
		// x - valve index
		// y - temp sensor index
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_SET2NDSENSORINDEX,&cmd[0],5) == 0) {
			x = atoi(argptr[0]); // valve index
			y = atoi(argptr[1]); // temp sensor index

			if(argcnt == 2) {				
				#ifdef commDebug 
					COMM_DBG.println("comm: set 2nd sensor index");
				#endif
				if (x >= 0 && x < ACTUATOR_COUNT && y < MAXONEWIRECNT) 
				{
					if (noOfDS18Devices > 0) 
					{
						// write sensor address code to eeprom layout mirror
						sensors.getAddress(currAddress, y);
						eep_content.owsensors2[x].familycode = currAddress[0];
						eep_content.owsensors2[x].romcode[0] = currAddress[1];
						eep_content.owsensors2[x].romcode[1] = currAddress[2];
						eep_content.owsensors2[x].romcode[2] = currAddress[3];
						eep_content.owsensors2[x].romcode[3] = currAddress[4];
						eep_content.owsensors2[x].romcode[4] = currAddress[5];
						eep_content.owsensors2[x].romcode[5] = currAddress[6];
						eep_content.owsensors2[x].crc = currAddress[7];

						// write index to valves structure
						myvalves[x].sensorindex2 = (byte) y;	

						eeprom_changed();	
						COMM_SER.println(APP_PRE_SET2NDSENSORINDEX);				
					}
				}
			}
			else {
				#ifdef commDebug 
					COMM_DBG.println("to few arguments");
				#endif
			}
		}


		// set valve sensors
		// x - valve index
		// arg1 - 8 byte hex address of 1st 1-wire sensor
		// arg2 - 8 byte hex address of 2nd 1-wire sensor
		// if hex address == 00-00-00-00-00-00-00-00 this sensor will be ignored
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_SETVLVSENSOR,&cmd[0],5) == 0) {
			x = atoi(argptr[0]); // valve index
			
			if(argcnt == 3) {				
				#ifdef commDebug 
					COMM_DBG.println("comm: set valve sensors");
				#endif
				setValveIDSensor (x,argptr[1],(uint8_t*) &eep_content.owsensors1[x]);
				setValveIDSensor (x,argptr[2],(uint8_t*) &eep_content.owsensors2[x]);
				COMM_SER.print(APP_PRE_SETVLVSENSOR);
				COMM_SER.print(" ");
				COMM_SER.println(x,DEC);
			}
			else {
				#ifdef commDebug 
					COMM_DBG.println("to few arguments");
				#endif
			}
		}


		// open all valves request
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_SETALLVLVOPEN,&cmd[0],5) == 0) {
			#ifdef commDebug 
				COMM_DBG.print("got open valve request for ");
			#endif

			x = atoi(argptr[0]);

			if(argcnt == 1 && x >= 0) {
				if( app_set_valveopen(x) == 0) {
					COMM_SER.print(APP_PRE_SETALLVLVOPEN);
					COMM_SER.println(" ");
					#ifdef commDebug 
						COMM_DBG.println(x, DEC);
					#endif
				}
				else {
					#ifdef commDebug 
						COMM_DBG.println("- error");
					#endif
				}
			}
			else {
				#ifdef commDebug 
					COMM_DBG.println("- error");
				#endif
			}
		}


		// learn valve x request
		// if x is 255 all valves will be learned
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_SETVLLEARN,&cmd[0],5) == 0) {
			#ifdef commDebug 
				COMM_DBG.print("start learning for valve ");
			#endif
			
			x = atoi(argptr[0]);

			if(argcnt == 1 && x >= 0) {
				if(app_set_valvelearning(x) == 0) {
					COMM_SER.println(APP_PRE_SETVLLEARN);
					#ifdef commDebug 
						COMM_DBG.println(x, DEC);
					#endif
				}
				else {
					#ifdef commDebug 
						COMM_DBG.println("- error");
					#endif
				}
			}
			else {
				#ifdef commDebug 
					COMM_DBG.println("- error");
				#endif
			}
		}

		// set motor characteristics
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_SETMOTCHARS,&cmd[0],5) == 0) {
			#ifdef commDebug 
				COMM_DBG.print("got set motor characteristics request ");
			#endif

			if (argcnt >= 3) {
				x = atoi(argptr[0]);
				y = atoi(argptr[1]);

				if (x>=5 && x<=50 && y>=5 && y<=50) {
					currentbound_low_fac = atoi(argptr[0]);
					currentbound_high_fac = atoi(argptr[1]);
					startOnPower = atoi (argptr[2]);

					eep_content.currentbound_low_fac = currentbound_low_fac;
					eep_content.currentbound_high_fac = currentbound_high_fac;
					eep_content.startOnPower = startOnPower;
					if (argcnt >= 4) {
						noOfMinCounts = atoi (argptr[3]);
						eep_content.noOfMinCounts = noOfMinCounts;	
					}
					if (argcnt == 5) {
						maxCalibRetries = atoi (argptr[4]);
						eep_content.maxCalibRetries = maxCalibRetries;	
					}
					eeprom_changed();
					COMM_SER.println(APP_PRE_SETMOTCHARS);
					#ifdef commDebug 
						COMM_DBG.println("- valid");
					#endif
				}
				else {
					#ifdef commDebug 
						COMM_DBG.println("- values out of bounds");
					#endif
				}
			}
			else {
				#ifdef commDebug 
					COMM_DBG.println("- error");
				#endif
			}
		}


		// get motor characteristics
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_GETMOTCHARS,&cmd[0],5) == 0) {
			#ifdef commDebug 
				COMM_DBG.println("got get motor characteristics request ");
			#endif
			COMM_SER.print(APP_PRE_GETMOTCHARS);
			COMM_SER.print(" ");			
			COMM_SER.print(currentbound_low_fac, DEC);
			COMM_SER.print(" ");		
			COMM_SER.print(currentbound_high_fac, DEC);
			COMM_SER.print(" ");		
			COMM_SER.print(startOnPower, DEC);
			COMM_SER.print(" ");		
			COMM_SER.print(noOfMinCounts, DEC);
			COMM_SER.print(" ");		
			COMM_SER.print(maxCalibRetries, DEC);
			COMM_SER.println(" ");			
		} 


		// detect valve status
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_SETDETECTVLV,&cmd[0],5) == 0) {
			commdbg_print("got detect valve status request");

			x = atoi(argptr[0]);

			if(argcnt == 1 && x >= 0) {
				// reset status of all valves so they will be detected again
				if( x == 255) {
					app_scan_valves();
					commdbg_println(" - reset all valves");
				}
				else commdbg_println(" - error");
				
				COMM_SER.print(APP_PRE_SETDETECTVLV);
				COMM_SER.println(" ");
			}
			else commdbg_println(" - error");		
		} 


		// get version request
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_GETVERSION,&cmd[0],5) == 0) {
			#ifdef commDebug 
				COMM_DBG.println("got version request");
			#endif

			COMM_SER.print(APP_PRE_GETVERSION);
			COMM_SER.print(" ");			

			COMM_SER.print(FIRMWARE_VERSION);
			#ifdef HARDWARE_VERSION
				COMM_SER.print("_");
				COMM_SER.print(HARDWARE_VERSION);
			#endif

			#ifdef FIRMWARE_BUILD
				COMM_SER.print(" ");
				COMM_SER.print(FIRMWARE_BUILD);
  			#endif

			COMM_SER.println(" ");	
		}

		// get hw Info request
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_GETHWINFO,&cmd[0],5) == 0) {
			#ifdef commDebug 
				COMM_DBG.println("got hw info request");
			#endif
			COMM_SER.print(APP_PRE_GETHWINFO);
			COMM_SER.print(" ");			
			// read ID
			COMM_SER.print(HAL_GetDEVID());
			COMM_SER.println(" ");	
		}


		// match sensors request
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_MATCHSENS,&cmd[0],5) == 0) {
			#ifdef commDebug 
				COMM_DBG.println("got match sensors request");
			#endif

			app_match_sensors();

			COMM_SER.print(APP_PRE_MATCHSENS);
			COMM_SER.println(" ");

		}


		// software reset request - untested
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_SOFTRESET,&cmd[0],5) == 0) {
			#ifdef commDebug 
				COMM_DBG.println("got software reset request");
			#endif

			COMM_SER.print(APP_PRE_SOFTRESET);
			COMM_SER.println(" ");

			delay(200);

			reset_STM32();
		}

		// get eeprom state
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else if(memcmp(APP_PRE_EEPSTATE,&cmd[0],5) == 0) {
			#ifdef commDebug 
				COMM_DBG.println("got get eeprom status request ");
			#endif
			COMM_SER.print(APP_PRE_EEPSTATE);
			COMM_SER.print(" ");			
			COMM_SER.print(eeprom_free(), DEC);
			COMM_SER.println(" ");			
		} 

		// unknown command
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		else {
			//COMM_DBG.println("unknown command received from ESP");
		}

	}
	else return -1;


return 0;
}


