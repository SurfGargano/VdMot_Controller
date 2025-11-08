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
#include "ETH.h"

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

static void _eth_phy_power_enable(bool enable)
{
    pinMode(ETH_PHY_POWER, OUTPUT);
    digitalWrite(ETH_PHY_POWER, enable);
    delay(1);
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
    UART_DBG.print("Net setup : interface type ");
    UART_DBG.println(String(VdmConfig.configFlash.netConfig.eth_wifi));
    UART_DBG.print("Net setup : state ");
    UART_DBG.print(String(ethState));
    UART_DBG.print(" ");
    UART_DBG.print(String(wifiState));
    UART_DBG.print(" ");
    UART_DBG.println(String(serverIsStarted));
  #endif
  
  switch (VdmConfig.configFlash.netConfig.eth_wifi) {
    case interfaceAuto :
      {
        setupEth(); 
        if (wifiState!=wifiDisabled) {
          setupWifi();
        }
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
  if (!serverIsStarted) {
    switch (ethState) {
      case ethIdle :
      {  
        #ifdef netDebug
          UART_DBG.println("setupEth : WT32_ETH01_onEvent ");
        #endif
        WT32_ETH01_onEvent();
        ethState=ethBegin;
        checkETHCounter=0;
        ethResetCounter=0;
        break;
      }
      case ethBegin :
      {
        #ifdef netDebug
          UART_DBG.println("setupEth : ETH.begin ");
        #endif
        ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER);
        ethState=ethConfig;
        #ifdef netDebug
          UART_DBG.println("setupEth : state ethIdle done  ");
        #endif
        break;
      }
      case ethConfig :
      {
        #ifdef netDebug
          UART_DBG.println("Eth ethConfig");
        #endif
        if (VdmConfig.configFlash.netConfig.dhcpEnabled==0) {
          ETH.config(VdmConfig.configFlash.netConfig.staticIp, 
          VdmConfig.configFlash.netConfig.gateway, 
          VdmConfig.configFlash.netConfig.mask,VdmConfig.configFlash.netConfig.dnsIp);
        }
        ethState=ethIsStarting;
        #ifdef netDebug
          UART_DBG.println("Eth ethConfig done");
        #endif
        break;
      }
      case ethIsStarting :
      {
        #ifdef netDebug
          UART_DBG.print("Eth IsStarting : Eth cable connected = ");
          UART_DBG.print(String(ETH.linkUp()));
          UART_DBG.print(" EIP= ");
          UART_DBG.print(ETH.localIP().toString());
          UART_DBG.print(":");
          UART_DBG.println(String(ETH.localIP()));
        #endif
        
        if (ETH.linkUp() && ((uint32_t) ETH.localIP()!=0)) {
          networkInfo.interfaceType=currentInterfaceIsEth;
          networkInfo.dhcpEnabled=VdmConfig.configFlash.netConfig.dhcpEnabled;
          networkInfo.ip=ETH.localIP();
          networkInfo.gateway=ETH.gatewayIP();
          networkInfo.dnsIp=ETH.dnsIP();
          networkInfo.mask=ETH.subnetMask();
          networkInfo.mac=ETH.macAddress();
          #ifdef netDebug
            UART_DBG.println("Eth ethIsStarting: set host name");
          #endif
          if (strlen(VdmConfig.configFlash.systemConfig.stationName)>0) ETH.setHostname(VdmConfig.configFlash.systemConfig.stationName);
          wifiState=wifiDisabled; 
          WiFi.disconnect(); 
          ethState=ethStarted;
        } else {
           #ifdef netDebug
              UART_DBG.print("Eth IsStarting : checkETHCounter = ");
              UART_DBG.println(String(checkETHCounter)+"ethResetCounter = "+String(ethResetCounter));
            #endif
          if (++checkETHCounter>30) {
            #ifdef netDebug
              UART_DBG.println("Eth IsStarting : go to begin ");
            #endif
            checkETHCounter=0;
            ethState=ethBegin; 
            if (VdmConfig.configFlash.netConfig.timeOutNetConnection>0) {
              if (++ethResetCounter>=VdmConfig.configFlash.netConfig.timeOutNetConnection) {
                Services.restartSystem(false);
              }
            }
          }  
        }
        break;
      }
      case ethStarted : 
      {
        #ifdef netDebug
          UART_DBG.println("Eth started : => InitServer");
        #endif
        ServerServices.initServer();
        serverIsStarted=true;
        ethState=ethIsRunning;
        break;
      }
      case ethIsRunning : break;
      case ethDisabled : break;
    }
  }  
}

void CVdmNet::scanWifiTask()
{
  #ifdef netDebugWIFI
    UART_DBG.println("scanWifiTask "+String(wifiScanState));
  #endif
  switch (wifiScanState)
  {
    case wifiScanIdle:
    {
      checkScanWifi=0;
      WiFi.scanDelete();
      netWorksSSID=ssidDefaultString;
      wifiScanState = wifiScanStarted;
      break;
    }
    case wifiScanStarted:
    {
      noOfNetworks = WiFi.scanNetworks(true);
      wifiScanState = wifiScanWaitForScanFinished;
      break;
    }
    case wifiScanWaitForScanFinished:
    {
      noOfNetworks=WiFi.scanComplete();
      #ifdef netDebugWIFI
        UART_DBG.print("get Wifi : no of SSID => ");
        UART_DBG.println(String(noOfNetworks));
      #endif
      if (noOfNetworks>=0) {
        getWifi(noOfNetworks);
        WiFi.scanDelete();
        wifiScanState = wifiScanFinished;
      }
      if (++checkScanWifi>=30) {
        wifiScanState = wifiScanIdle;
        if (++scanRepeatWifi>=5) {
          WiFi.scanDelete();
          wifiScanState = wifiScanFinished;
        }
      }
      break;
    }
    case wifiScanFinished:
    {
      break;
    }
  }
}

void CVdmNet::getWifi(int16_t thisNoOfNetworks)
{ 
  bool found;
  DynamicJsonDocument doc(1024);
  JsonArray ssid = doc.createNestedArray("scanSSID");

  for (uint16_t i = 0; i < thisNoOfNetworks; i++) {
    found = false;
    for (uint16_t n=0; n<ssid.size();n++) {
      if (ssid[n]==WiFi.SSID(i)) found = true;   
    }
    if (!found)
      ssid.add(WiFi.SSID(i));
  }
  netWorksSSID="";
  serializeJson(doc, netWorksSSID);
  doc.clear();
  #ifdef netDebugWIFI
    UART_DBG.println("get Wifi : SSID => "+netWorksSSID);
  #endif
} 


void CVdmNet::setupWifi() 
{
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
          UART_DBG.print("wifi : ssid = ");
          UART_DBG.print(String(VdmConfig.configFlash.netConfig.ssid));
          UART_DBG.print(" pw = ");
          UART_DBG.println(String(VdmConfig.configFlash.netConfig.pwd));
        #endif
        WiFi.mode(WIFI_MODE_STA); 
        WiFi.begin(VdmConfig.configFlash.netConfig.ssid, VdmConfig.configFlash.netConfig.pwd);
        checkIPCounter=0;
        wifiState=wifiConfig;
        break;
      }
      case wifiConfig :
      {
		    if (VdmConfig.configFlash.netConfig.dhcpEnabled==0) {
          WiFi.config(VdmConfig.configFlash.netConfig.staticIp, 
          VdmConfig.configFlash.netConfig.gateway, 
          VdmConfig.configFlash.netConfig.mask,VdmConfig.configFlash.netConfig.dnsIp);
        }
        if (strlen(VdmConfig.configFlash.systemConfig.stationName)>0) WiFi.setHostname(VdmConfig.configFlash.systemConfig.stationName);
        wifiState=wifiIsStarting;
        break;
      }
      case wifiIsStarting :
      {
        String wips = WiFi.localIP().toString();
        uint32_t wip = WiFi.localIP();
        uint32_t wifiStatus = WiFi.status();
        #ifdef netDebugWIFI
          UART_DBG.print("wifiIsStarting : Wifi Status=");
          UART_DBG.print(String(wifiStatus));
          UART_DBG.print(" WIP= ");
          UART_DBG.print(wips);
          UART_DBG.print(" : ");
          UART_DBG.println(String(wip));
          UART_DBG.print("CheckIpCounter : ");
          UART_DBG.print(String(checkIPCounter));
          UART_DBG.print(" , ResetCounter : ");
          UART_DBG.println(String(wifiResetCounter));
        #endif
        
        if ((WiFi.status() == WL_CONNECTED) && (wip!=0)) {
          wifiState=wifiStarted;
          networkInfo.interfaceType=currentInterfaceIsWifi;
          networkInfo.dhcpEnabled=VdmConfig.configFlash.netConfig.dhcpEnabled;
          networkInfo.ip=WiFi.localIP();
          networkInfo.gateway=WiFi.gatewayIP();
          networkInfo.dnsIp=WiFi.dnsIP();
          networkInfo.mask=WiFi.subnetMask();
          networkInfo.mac=WiFi.macAddress();
          checkIPCounter=0;
        } else {
          if (++checkIPCounter>30) {
            checkIPCounter=0;
          //  WiFi.disconnect();
           // WiFi.reconnect();
           WiFi.mode(WIFI_MODE_NULL);
           wifiState=wifiIdle;
            #ifdef netDebugWIFI
              UART_DBG.println("wifiIsStarting : Wifi reconnect");
            #endif
            if (VdmConfig.configFlash.netConfig.timeOutNetConnection>0) {
              if (++wifiResetCounter>=VdmConfig.configFlash.netConfig.timeOutNetConnection) {
                Services.restartSystem(false);
              }
            }
          }
         
        }
        break;
      }
      case wifiStarted : {
        #ifdef netDebugWIFI
          UART_DBG.println("Wifi started : => InitServer");
        #endif
        ServerServices.initServer();
        setupNtp();
        serverIsStarted=true;
        wifiState=wifiIsRunning;
        break;
      }
      case wifiIsRunning : {
        break;
      }
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
    UART_DBG.println("checkNet : serverIsStarted");
    VdmTask.deleteTask(&VdmTask.taskIdCheckNet);
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
    UART_DBG.println("checkNet : setup");
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

void CVdmNet::checkNetConnected()
{
  if (wifiState==wifiIsRunning) {
    uint8_t wifiStatus = WiFi.status();
    #ifdef netDebugWIFI
      UART_DBG.println("checkNetConnected : Wifi status "+ String(wifiStatus));
    #endif
    if ((wifiStatus!= WL_CONNECTED) || ((uint32_t) WiFi.localIP()==0)) {
      WiFi.disconnect();
      WiFi.reconnect();
      #ifdef netDebugWIFI
        UART_DBG.println("Wifi reconnect");
      #endif
      if (VdmConfig.configFlash.netConfig.timeOutNetConnection>0) {
        if (++wifiResetCounter>=VdmConfig.configFlash.netConfig.timeOutNetConnection) {
          Services.restartSystem(false);
        }
      }
    } else {
      wifiResetCounter=0;
      if (wifiScanState == wifiScanFinished) 
        VdmTask.deleteTask(&VdmTask.taskIdScanWiFi);
    }
  }
  if (ethState==ethIsRunning) {
    bool linkStatus=ETH.linkUp();
    #ifdef netDebug
      UART_DBG.println("checkNetConnected : eth link status "+ String(linkStatus));
    #endif
    if ((!linkStatus) || ((uint32_t) ETH.localIP()==0)) {
      ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER);
      #ifdef netDebug
        UART_DBG.println("ETH reconnect");
      #endif
      if (VdmConfig.configFlash.netConfig.timeOutNetConnection>0) {
        if (++ethResetCounter>=VdmConfig.configFlash.netConfig.timeOutNetConnection) {
          Services.restartSystem(false);
        }
      }
    } else {
      ethResetCounter=0;
      if (wifiScanState == wifiScanFinished) VdmTask.deleteTask(&VdmTask.taskIdScanWiFi);
    }
  }
}  


void CVdmNet::reconnect()
{
  #ifdef netDebug
    UART_DBG.println("Network reconnect");
  #endif
  switch (VdmConfig.configFlash.netConfig.eth_wifi) {
    case interfaceAuto :
      {
        ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER); 
        if (wifiState!=wifiDisabled) {
          //WiFi.disconnect();
          WiFi.reconnect();
        } 
        break;
      }
      case interfaceEth :
      {
        ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER); 
        break;
      }
      case interfaceWifi :
      {
        //WiFi.disconnect();
        WiFi.reconnect();
        break;
      }
  }
}