/**HEADER*******************************************************************
  project : VdMot Controller

  author : SurfGargano, Lenti84

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



#include <stdint.h>
#include <VdmNet.h>
#include <WiFi.h>
#include <WiFiClient.h>

#include "globals.h"
#include "mqtt.h"

#include "WT32AsyncOTA.h"

#include "web.h"
#include "tfs.h"

#include "time.h"

#include "stm32.h"
#include "stm32ota.h"
#include <ESPmDNS.h>
#include <WiFiUdp.h>

#include "VdmTask.h"
#include "VdmSystem.h"
#include "mqtt.h"
#include "ServerServices.h"

#include <AsyncJson.h>
#include <ArduinoJson.h>
#include "esp32-hal-time.c"

#include <FS.h>
#ifdef USE_LittleFS
  #define SPIFFS LittleFS
  #include <LITTLEFS.h> 
#else
  #include <SPIFFS.h>
#endif 

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udpClient;

CVdmNet VdmNet;

 // Create a new empty syslog instance
 Syslog syslog(udpClient, SYSLOG_PROTO_IETF);

// server handles --------------------------------------------------

CVdmNet::CVdmNet()
{
}

void CVdmNet::init()
{
  serverIsStarted = false;
  wifiState = wifiIdle;
  ethState = ethIdle;
  dataBrokerIsStarted = false;
  sntpActive = false;
  sntpReachable=false;
  if (!SPIFFS.begin(true)) {
    #ifdef EnvDevelop
      UART_DBG.println("An Error has occurred while mounting SPIFFS");
    #endif
    return;
  }
  #ifdef EnvDevelop
    UART_DBG.println("SPIFFS booted");
  #endif
}

void CVdmNet::setup() 
{
  
  #ifdef netDebug
    UART_DBG.print("Interface type ");
    UART_DBG.println(VdmConfig.configFlash.netConfig.eth_wifi);
  #endif
  
  switch (VdmConfig.configFlash.netConfig.eth_wifi) {
    case interfaceAuto :
      {
        setupEth(); 
        if (wifiState!=wifiDisabled) setupWifi();
        break;
      }
      case interfaceEth :
      {
        setupEth();
        break;
      }
      case interfaceWifi :
      {
        setupWifi();
        break;
      }
  }
}

void CVdmNet::setupEth() 
{
  #ifdef netDebug
    UART_DBG.print("Setup Eth ");
    UART_DBG.println (ethState);
    UART_DBG.print("Server started ");
    UART_DBG.println (serverIsStarted);
  #endif
  if (!serverIsStarted) {
    switch (ethState) {
      case wifiIdle :
      {  
        WT32_ETH01_onEvent();
        ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER);
        if (VdmConfig.configFlash.netConfig.dhcpEnabled==0) {
          ETH.config(VdmConfig.configFlash.netConfig.staticIp, 
          VdmConfig.configFlash.netConfig.gateway, 
          VdmConfig.configFlash.netConfig.mask,VdmConfig.configFlash.netConfig.dnsIp);
        }
        if (strlen(VdmConfig.configFlash.systemConfig.stationName)>0) ETH.setHostname(VdmConfig.configFlash.systemConfig.stationName);
        ethState=ethIsStarting;
        break;
      }
      case ethIsStarting :
      {
        if (WT32_ETH01_isConnected()) {
          #ifdef netDebug
            UART_DBG.println("Setup Eth cable is connected");
          #endif
          ServerServices.initServer();
          UART_DBG.println(ETH.localIP());
          serverIsStarted=true; 
          ethState=ethConnected;
          networkInfo.interfaceType=currentInterfaceIsEth;
          networkInfo.dhcpEnabled=VdmConfig.configFlash.netConfig.dhcpEnabled;
          networkInfo.ip=ETH.localIP();
          networkInfo.gateway=ETH.gatewayIP();
          networkInfo.dnsIp=ETH.dnsIP();
          networkInfo.mask=ETH.subnetMask();
          networkInfo.mac=ETH.macAddress();
          wifiState=wifiDisabled; 
          WiFi.disconnect(); 
        }
        break;
      }
      case ethConnected : break;
      case ethDisabled : break;
    }
  }  
}


void CVdmNet::setupWifi() 
{
  #ifdef netDebug
  UART_DBG.print("Setup Wifi ");
  UART_DBG.println (wifiState);
  #endif

  if (!serverIsStarted) {
    switch (wifiState) {
      case wifiIdle :
      {
        if ((strlen(VdmConfig.configFlash.netConfig.ssid)==0) || (strlen(VdmConfig.configFlash.netConfig.pwd)==0)) {
          wifiState=wifiDisabled;
          UART_DBG.println("wifi : no ssid or no pathword");
          break;
        }
        
        #ifdef netDebugWIFI
        UART_DBG.print("wifi : ssid ");
        UART_DBG.println(VdmConfig.configFlash.netConfig.ssid);
        UART_DBG.print("wifi : pw ");
        UART_DBG.println(VdmConfig.configFlash.netConfig.pwd);
        #endif
        WiFi.mode(WIFI_MODE_STA); 
		    if (VdmConfig.configFlash.netConfig.dhcpEnabled==0) {
          WiFi.config(VdmConfig.configFlash.netConfig.staticIp, 
          VdmConfig.configFlash.netConfig.gateway, 
          VdmConfig.configFlash.netConfig.mask,VdmConfig.configFlash.netConfig.dnsIp);
        }
        WiFi.begin(VdmConfig.configFlash.netConfig.ssid, VdmConfig.configFlash.netConfig.pwd);
        if (strlen(VdmConfig.configFlash.systemConfig.stationName)>0) WiFi.setHostname(VdmConfig.configFlash.systemConfig.stationName);

        wifiState=wifiIsStarting;
        break;
      }
      case wifiIsStarting :
      {
        if (WiFi.status() == WL_CONNECTED) {
          ServerServices.initServer();
          setupNtp();
          UART_DBG.println(WiFi.localIP());
          serverIsStarted=true; 
          wifiState=wifiConnected;
          networkInfo.interfaceType=currentInterfaceIsWifi;
          networkInfo.dhcpEnabled=VdmConfig.configFlash.netConfig.dhcpEnabled;
          networkInfo.ip=WiFi.localIP();
          networkInfo.gateway=WiFi.gatewayIP();
          networkInfo.dnsIp=WiFi.dnsIP();
          networkInfo.mask=WiFi.subnetMask();
          networkInfo.mac=WiFi.macAddress();
        }
        break;
      }
      case wifiConnected : break;
      case wifiDisabled : break;
    }
  }
}

bool CVdmNet::checkSntpReachable()
{
  u8_t ret = sntp_getreachability(0);
 // UART_DBG.println("checkSntpReachable "+String(ret)+':'+String(sntp_getservername(0)));
  return (ret==1);
}

void CVdmNet::configTzTime(const char* tz, const char* server1, const char* server2, const char* server3)
{
    //tcpip_adapter_init();  // Should not hurt anything if already inited
    esp_err_t err=esp_netif_init();
    if(sntp_enabled()){
        sntp_stop();
    }
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, (char*)server1);
    if (server2!=NULL) sntp_setservername(1, (char*)server2);
    if (server3!=NULL) sntp_setservername(2, (char*)server3);
    sntp_init();
    int senv=setenv("TZ", tz, 1);
    tzset();
    UART_DBG.println("configTzTime "+String(err)+" , "+String(senv));
}

void CVdmNet::setupNtp() 
{
  // Init and get the time
  VdmNet.sntpActive=strlen(VdmConfig.configFlash.netConfig.timeServer)>0; 
  if (VdmNet.sntpActive)
  {
    UART_DBG.println("Get time from server "+String(VdmConfig.configFlash.netConfig.timeServer)+":"+String(sntp_enabled()));
    configTzTime(VdmConfig.configFlash.timeZoneConfig.tzCode ,VdmConfig.configFlash.netConfig.timeServer);
    VdmSystem.getLocalTime(&startTimeinfo);
  }
}

void CVdmNet::startBroker()
{
  switch (VdmConfig.configFlash.protConfig.dataProtocol) {
    case mqttProtocol:
    case mqttProtocolHA:
    {
      Mqtt.mqtt_setup(VdmConfig.configFlash.protConfig.brokerIp,VdmConfig.configFlash.protConfig.brokerPort);
      VdmTask.startMqtt(VdmConfig.configFlash.protConfig.brokerInterval);
      dataBrokerIsStarted=true;
      break;
    }
  }
}

void CVdmNet::mqttBroker()
{
   Mqtt.mqtt_loop();
}

void CVdmNet::checkNet() 
{
  if (serverIsStarted) {
    VdmTask.deleteTask(VdmTask.taskIdCheckNet);

    VdmSystem.getSystemInfo();
    #ifdef netUseMDNS
      if (MDNS.begin("esp32")) {
        UART_DBG.println("MDNS responder started");
      }
    #endif
   
    startSysLog();
    VdmTask.startServices();
   
  } else {
    // check if net is connected
    setup();
  }
}

void CVdmNet::startSysLog()
{
  // prepare syslog configuration here (can be anywhere before first call of 
  // log/logf method)
  if (VdmConfig.configFlash.netConfig.syslogLevel>0) {
      if (!syslogStarted) {
      #ifdef netDebug
        UART_DBG.println("start syslog server : level = "+ String(VdmConfig.configFlash.netConfig.syslogLevel));
      #endif
      syslog.server(IPAddress(VdmConfig.configFlash.netConfig.syslogIp), VdmConfig.configFlash.netConfig.syslogPort);
      syslog.deviceHostname(DEVICE_HOSTNAME);
      syslog.appName(APP_NAME);
      syslog.defaultPriority(LOG_KERN);
      syslogStarted=true;
    }
  }
}

void CVdmNet::checkWifi()
{
  uint8_t WifiStatus = WiFi.status();
  #ifdef netDebug
    UART_DBG.println("Wifi status "+ String(WifiStatus));
  #endif
  if (wifiState==wifiConnected) {
      if (WifiStatus!= WL_CONNECTED) {
      WiFi.disconnect();
      WiFi.reconnect();
      UART_DBG.println("Wifi reconnect");
    }
  }
}  
