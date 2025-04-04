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
#include "VdmConfig.h"
#include "VdmTask.h"
#include "web.h"
#include "Services.h"
#include "stmApp.h"
#include "PIControl.h"

CVdmConfig VdmConfig;


CVdmConfig::CVdmConfig()
{
  
}

void CVdmConfig::init()
{
  setDefault();
  readConfig();
}

void CVdmConfig::resetConfig (bool reboot)
{  
  clearConfig(); 
  writeConfig();
  writeValvesControlConfig();
  if (reboot) Services.restartSystem(false);
}

void CVdmConfig::restoreConfig (bool reboot)
{  
  setDefault();
  writeConfig();
  writeValvesControlConfig();
  if (reboot) Services.restartSystem(false);
}

void CVdmConfig::setDefault()
{
  configFlash.netConfig.eth_wifi = 0;	      // 0 = auto (default)
  configFlash.netConfig.dhcpEnabled = 1;    // 0 = no DHCP, 1 = use DHCP
  configFlash.netConfig.staticIp = 0;
  configFlash.netConfig.mask = 0;
  configFlash.netConfig.gateway = 0;
  configFlash.netConfig.dnsIp = 0;
  memset (configFlash.netConfig.ssid,0,sizeof(configFlash.netConfig.ssid));
  memset (configFlash.netConfig.pwd,0,sizeof(configFlash.netConfig.pwd));
  memset (configFlash.netConfig.userName,0,sizeof(configFlash.netConfig.userName));
  memset (configFlash.netConfig.userPwd,0,sizeof(configFlash.netConfig.userPwd));
  memset (configFlash.timeZoneConfig.tz,0,sizeof(configFlash.timeZoneConfig.tz));
  memset (configFlash.timeZoneConfig.tzCode,0,sizeof(configFlash.timeZoneConfig.tzCode));
  clearConfig();
}

void CVdmConfig::clearConfig()
{ 
  configFlash.protConfig.dataProtocol = 0;
  configFlash.protConfig.brokerIp = 0;
  configFlash.protConfig.brokerPort = 0;
  configFlash.protConfig.brokerInterval = 1000;
  configFlash.protConfig.publishInterval = 10;
  configFlash.protConfig.minBrokerDelay = 5;
  memset (configFlash.protConfig.userName,0,sizeof(configFlash.protConfig.userName));
  memset (configFlash.protConfig.userPwd,0,sizeof(configFlash.protConfig.userPwd));
  configFlash.protConfig.protocolFlags.publishSeparate = false;
  configFlash.protConfig.protocolFlags.publishAllTemps = false;
  configFlash.protConfig.protocolFlags.publishPathAsRoot = false;
  configFlash.protConfig.protocolFlags.publishUpTime = false;
  configFlash.protConfig.protocolFlags.publishOnChange = false;
  configFlash.protConfig.protocolFlags.publishRetained = false;
  configFlash.protConfig.protocolFlags.publishPlainText = false;
  configFlash.protConfig.protocolFlags.publishDiag = false;
  configFlash.protConfig.keepAliveTime = 60;
  configFlash.protConfig.mqttConfig.flags.timeoutTSActive = false;
  configFlash.protConfig.mqttConfig.flags.timeoutDSActive = false;
  configFlash.protConfig.mqttConfig.timeOut = 120;
  configFlash.protConfig.mqttConfig.toPos = 10;
  
  for (uint8_t i=0; i<ACTUATOR_COUNT; i++) {
    configFlash.valvesConfig.valveConfig[i].active = false;
    memset (configFlash.valvesConfig.valveConfig[i].name,0,sizeof(configFlash.valvesConfig.valveConfig[i].name));
  }

  for (uint8_t i=0; i<ACTUATOR_COUNT; i++) {
    configFlash.valvesControlConfig.valveControlConfig[i].controlFlags.active = false;
    configFlash.valvesControlConfig.valveControlConfig[i].controlFlags.allow = allowHeatingCooling;
    configFlash.valvesControlConfig.valveControlConfig[i].controlFlags.windowInstalled = false;
    configFlash.valvesControlConfig.valveControlConfig[i].xp=20;
    configFlash.valvesControlConfig.valveControlConfig[i].link=0;
    configFlash.valvesControlConfig.valveControlConfig[i].offset=0;
    configFlash.valvesControlConfig.valveControlConfig[i].targetSource=0;
    configFlash.valvesControlConfig.valveControlConfig[i].valueSource=0;
    configFlash.valvesControlConfig.valveControlConfig[i].ti=3600;
    configFlash.valvesControlConfig.valveControlConfig[i].ts=900;
    configFlash.valvesControlConfig.valveControlConfig[i].scheme=0;
    configFlash.valvesControlConfig.valveControlConfig[i].ki=0.01;
    configFlash.valvesControlConfig.valveControlConfig[i].startActiveZone=0;
    configFlash.valvesControlConfig.valveControlConfig[i].endActiveZone=100;
    configFlash.valvesControlInit.valveControlInit[i].tTarget=20;
    configFlash.valvesControl1Config.valveControl1Config[i].deadband=0.0;
  }
  configFlash.valvesControlConfig.heatControl=0;
  configFlash.valvesControlConfig.parkingPosition=10;
 
  for (uint8_t i=0; i<TEMP_SENSORS_COUNT; i++) {
    configFlash.tempsConfig.tempConfig[i].active = false;
    configFlash.tempsConfig.tempConfig[i].offset = 0;
    memset (configFlash.tempsConfig.tempConfig[i].name,0,sizeof(configFlash.tempsConfig.tempConfig[i].name));
    memset (configFlash.tempsConfig.tempConfig[i].ID,0,sizeof(configFlash.tempsConfig.tempConfig[i].ID));
  }
  configFlash.systemConfig.celsiusFahrenheit=0;

  for (uint8_t i=0; i<VOLT_SENSORS_COUNT; i++) {
    configFlash.voltsConfig.voltConfig[i].active = false;
    configFlash.voltsConfig.voltConfig[i].offset = 0;
    configFlash.voltsConfig.voltConfig[i].factor = 1.0;
    memset (configFlash.voltsConfig.voltConfig[i].unit,0,sizeof(configFlash.voltsConfig.voltConfig[i].unit));
    memset (configFlash.voltsConfig.voltConfig[i].name,0,sizeof(configFlash.voltsConfig.voltConfig[i].name));
    memset (configFlash.voltsConfig.voltConfig[i].ID,0,sizeof(configFlash.voltsConfig.voltConfig[i].ID));
  }
  memset (configFlash.voltsConfig.voltAVConfig,0,sizeof(configFlash.voltsConfig.voltAVConfig));

  memset (configFlash.systemConfig.stationName,0,sizeof(configFlash.systemConfig.stationName));
  strncpy(configFlash.systemConfig.stationName,DEVICE_HOSTNAME,sizeof(configFlash.systemConfig.stationName));

  configFlash.valvesConfig.dayOfCalib=9;
  configFlash.valvesConfig.hourOfCalib=0;
  memset (configFlash.netConfig.pwd,0,sizeof(configFlash.netConfig.timeServer));
  strncpy(configFlash.netConfig.timeServer,"pool.ntp.org",sizeof(configFlash.netConfig.timeServer));
  strncpy(configFlash.timeZoneConfig.tz,"Europe/Berlin",sizeof(configFlash.timeZoneConfig.tz));
  strncpy(configFlash.timeZoneConfig.tzCode,"CET-1CEST,M3.5.0,M10.5.0/3",sizeof(configFlash.timeZoneConfig.tzCode));

  configFlash.netConfig.syslogLevel=0;
  configFlash.netConfig.syslogIp=0;
  configFlash.netConfig.syslogPort=0; 

  configFlash.messengerConfig.activeFlags.pushOver=0;
  configFlash.messengerConfig.reason.reasonFlags.mqttTimeOut=0;
  configFlash.messengerConfig.reason.reasonFlags.notDetect=0;
  configFlash.messengerConfig.reason.reasonFlags.reset=0;
  configFlash.messengerConfig.reason.reasonFlags.valveFailed=0;
  configFlash.messengerConfig.reason.reasonFlags.ds18Failed=0;
  configFlash.messengerConfig.reason.reasonFlags.tValueFailed=0;
  configFlash.messengerConfig.reason.reasonFlags.valveBlocked=0;
  configFlash.messengerConfig.reason.mqttTimeOutTime=10;
  memset(configFlash.messengerConfig.pushover.title,0,sizeof(configFlash.messengerConfig.pushover.title));
  memset(configFlash.messengerConfig.pushover.appToken,0,sizeof(configFlash.messengerConfig.pushover.appToken));
  memset(configFlash.messengerConfig.pushover.userToken,0,sizeof(configFlash.messengerConfig.pushover.userToken));

  miscValues.lastCalib=0;

}

void CVdmConfig::readConfig()
{
  if (prefs.begin(nvsSystemCfg,false)) {
    configFlash.systemConfig.celsiusFahrenheit=prefs.getUChar(nvsSystemCelsiusFahrenheit);
    if (prefs.isKey(nvsSystemStationName))
      prefs.getString(nvsSystemStationName,(char*) configFlash.systemConfig.stationName,sizeof(configFlash.systemConfig.stationName));
    prefs.end();
  }

  if (prefs.begin(nvsNetCfg,false)) {
    configFlash.netConfig.eth_wifi=prefs.getUChar(nvsNetEthwifi);
    configFlash.netConfig.dhcpEnabled=prefs.getUChar(nvsNetDhcp,1);
    configFlash.netConfig.staticIp=prefs.getULong(nvsNetStaticIp);
    configFlash.netConfig.mask=prefs.getULong(nvsNetMask);
    configFlash.netConfig.gateway=prefs.getULong(nvsNetGW);
    configFlash.netConfig.dnsIp=prefs.getULong(nvsNetDnsIp); 
    if (prefs.isKey(nvsNetSsid))
      prefs.getString(nvsNetSsid,(char*) configFlash.netConfig.ssid,sizeof(configFlash.netConfig.ssid));
    if (prefs.isKey(nvsNetPwd))
      prefs.getString(nvsNetPwd,(char*) configFlash.netConfig.pwd,sizeof(configFlash.netConfig.pwd));
    if (prefs.isKey(nvsNetUserName))
      prefs.getString(nvsNetUserName,(char*) configFlash.netConfig.userName,sizeof(configFlash.netConfig.userName));
    if (prefs.isKey(nvsNetUserPwd))
      prefs.getString(nvsNetUserPwd,(char*) configFlash.netConfig.userPwd,sizeof(configFlash.netConfig.userPwd));
    if (prefs.isKey(nvsNetTimeServer))
      prefs.getString(nvsNetTimeServer,(char*) configFlash.netConfig.timeServer,sizeof(configFlash.netConfig.timeServer));
   
    configFlash.netConfig.syslogLevel=prefs.getUChar(nvsNetSysLogEnable);
    configFlash.netConfig.syslogIp=prefs.getULong(nvsNetSysLogIp);
    configFlash.netConfig.syslogPort=prefs.getUShort(nvsNetSysLogPort);
    prefs.end();
  }
  if (prefs.begin(nvsProtCfg,false)) {
    configFlash.protConfig.dataProtocol = prefs.getUChar(nvsProtDataProt);
    configFlash.protConfig.brokerIp = prefs.getULong(nvsProtBrokerIp);
    configFlash.protConfig.brokerPort = prefs.getUShort(nvsProtBrokerPort);
    configFlash.protConfig.brokerInterval = prefs.getULong(nvsProtBrokerInterval,1000);
    configFlash.protConfig.publishInterval = prefs.getULong(nvsProtPublishInterval,10);
    if (prefs.isKey(nvsProtBrokerUser))
      prefs.getString(nvsProtBrokerUser,(char*) configFlash.protConfig.userName,sizeof(configFlash.protConfig.userName));
    if (prefs.isKey(nvsProtBrokerPwd))
      prefs.getString(nvsProtBrokerPwd,(char*) configFlash.protConfig.userPwd,sizeof(configFlash.protConfig.userPwd));
    uint8_t a=prefs.getUChar(nvsProtBrokerPublishFlags,7);
    configFlash.protConfig.protocolFlags =  *(VDM_PROTOCOL_CONFIG_FLAGS *)&a;
    configFlash.protConfig.keepAliveTime = prefs.getUShort(nvsProtBrokerKeepAliveTime,60);
    configFlash.protConfig.minBrokerDelay = prefs.getUShort(nvsProtBrokerMinBrokerDelay,5);
    a=prefs.getUChar(nvsProtBrokerMQTTFlags,0);
    configFlash.protConfig.mqttConfig.flags = *(VDM_PROTOCOL_MQTT_CONFIG_FLAGS *)&a;
    configFlash.protConfig.mqttConfig.timeOut =prefs.getULong(nvsProtBrokerMQTTTimeOut,120);
    configFlash.protConfig.mqttConfig.toPos=prefs.getUChar(nvsProtBrokerMQTTToPos,10);
    prefs.end();
  }

  if (prefs.begin(nvsValvesCfg,false)) {
    if (prefs.isKey(nvsValves))
      prefs.getBytes(nvsValves, (void *) configFlash.valvesConfig.valveConfig, sizeof(configFlash.valvesConfig.valveConfig));
    configFlash.valvesConfig.dayOfCalib=prefs.getUChar(nvsDayOfCalib,9);
    configFlash.valvesConfig.hourOfCalib=prefs.getUChar(nvsHourOfCalib); 
    prefs.end();
  }

  if (prefs.begin(nvsValvesControlCfg,false)) {
    if (prefs.isKey(nvsValvesControl)) {
      prefs.getBytes(nvsValvesControl, (void *) configFlash.valvesControlConfig.valveControlConfig, sizeof(configFlash.valvesControlConfig.valveControlConfig));
      configFlash.valvesControlConfig.heatControl=prefs.getUChar (nvsValvesControlHeatControl,0);
      configFlash.valvesControlConfig.parkingPosition=prefs.getUChar (nvsValvesControlParkPos,10);
      heatValues.heatControl=configFlash.valvesControlConfig.heatControl;
      heatValues.parkPosition=configFlash.valvesControlConfig.parkingPosition;
    }
    if (prefs.isKey(nvsValvesControl1)) {
      prefs.getBytes(nvsValvesControl1, (void *) configFlash.valvesControl1Config.valveControl1Config, sizeof(configFlash.valvesControl1Config.valveControl1Config));
    }
    if (prefs.isKey(nvsValvesControlInit)) {
      prefs.getBytes(nvsValvesControlInit, (void *) configFlash.valvesControlInit.valveControlInit, sizeof(configFlash.valvesControlInit.valveControlInit));
    }
    prefs.end();
  }
 


  if (prefs.begin(nvsTempsCfg,false)) {
    if (prefs.isKey(nvsTemps))
      prefs.getBytes(nvsTemps,(void *) configFlash.tempsConfig.tempConfig, sizeof(configFlash.tempsConfig.tempConfig));
    prefs.end();
  }

   if (prefs.begin(nvsVoltsCfg,false)) {
    if (prefs.isKey(nvsVolts))
      prefs.getBytes(nvsVolts,(void *) &configFlash.voltsConfig, sizeof(configFlash.voltsConfig));
    prefs.end();
  }

  if (prefs.begin(nvsTZCfg,false)) {
    if (prefs.isKey(nvsTZ))
      prefs.getString(nvsTZ,(char*) configFlash.timeZoneConfig.tz,sizeof(configFlash.timeZoneConfig.tz));
    if (prefs.isKey(nvsTZCode))
      prefs.getString(nvsTZCode,(char*) configFlash.timeZoneConfig.tzCode,sizeof(configFlash.timeZoneConfig.tzCode));
    prefs.end();
  }

  if (prefs.begin(nvsMsgCfg,false)) {
    uint8_t a=prefs.getUChar(nvsMsgCfgFlags,0);
    configFlash.messengerConfig.activeFlags =  *(VDM_MSG_ACTIVE_CONFIG_FLAGS *)&a;
    a=prefs.getUChar(nvsMsgCfgReason,0);
    configFlash.messengerConfig.reason.reasonFlags =  *(VDM_MSG_REASON_CONFIG_FLAGS *)&a;
    if (prefs.isKey(nvsMsgCfgMqttTimeout))
      configFlash.messengerConfig.reason.mqttTimeOutTime =prefs.getShort(nvsMsgCfgMqttTimeout,10);
    
    if (prefs.isKey(nvsMsgCfgPOAppToken))
      prefs.getString(nvsMsgCfgPOAppToken,(char*) configFlash.messengerConfig.pushover.appToken,sizeof(configFlash.messengerConfig.pushover.appToken));
    if (prefs.isKey(nvsMsgCfgPOUserToken))
      prefs.getString(nvsMsgCfgPOUserToken,(char*) configFlash.messengerConfig.pushover.userToken,sizeof(configFlash.messengerConfig.pushover.userToken));
    if (prefs.isKey(nvsMsgCfgPOTitle))
      prefs.getString(nvsMsgCfgPOTitle,(char*) configFlash.messengerConfig.pushover.title,sizeof(configFlash.messengerConfig.pushover.title));  
    
    if (prefs.isKey(nvsMsgCfgEmailUser))
      prefs.getString(nvsMsgCfgEmailUser,(char*) configFlash.messengerConfig.email.user,sizeof(configFlash.messengerConfig.email.user));
    if (prefs.isKey(nvsMsgCfgEmailPwd))
      prefs.getString(nvsMsgCfgEmailPwd,(char*) configFlash.messengerConfig.email.pwd,sizeof(configFlash.messengerConfig.email.pwd));
    if (prefs.isKey(nvsMsgCfgEMailHost))
      prefs.getString(nvsMsgCfgEMailHost,(char*) configFlash.messengerConfig.email.host,sizeof(configFlash.messengerConfig.email.host));
    configFlash.messengerConfig.email.port = prefs.getShort(nvsMsgCfgEmailPort,465);
    if (prefs.isKey(nvsMsgCfgEmailRecipient))
      prefs.getString(nvsMsgCfgEmailRecipient,(char*) configFlash.messengerConfig.email.recipient,sizeof(configFlash.messengerConfig.email.recipient));
    if (prefs.isKey(nvsMsgCfgEmailTitle))
      prefs.getString(nvsMsgCfgEmailTitle,(char*) configFlash.messengerConfig.email.title,sizeof(configFlash.messengerConfig.email.title));
    prefs.end();
  }

  if (prefs.begin(nvsMisc,false)) {
    if (prefs.isKey(nvsMiscLastCalib))
      miscValues.lastCalib=prefs.getLong(nvsMiscLastCalib,0);
    prefs.end();
  }
  
  for (uint8_t picIdx=0; picIdx<ACTUATOR_COUNT; picIdx++) { 
    PiControl[picIdx].target=VdmConfig.configFlash.valvesControlInit.valveControlInit[picIdx].tTarget;
  }

}

void CVdmConfig::writeConfig(bool reboot)
{
  prefs.begin(nvsSystemCfg,false);
  prefs.clear();
  prefs.putUChar(nvsSystemCelsiusFahrenheit,configFlash.systemConfig.celsiusFahrenheit);
  prefs.putString(nvsSystemStationName,configFlash.systemConfig.stationName);
  prefs.end();
 
  prefs.begin(nvsNetCfg,false);
  prefs.clear();
  prefs.putUChar(nvsNetEthwifi,configFlash.netConfig.eth_wifi);
  prefs.putUChar(nvsNetDhcp,configFlash.netConfig.dhcpEnabled);
  prefs.putULong(nvsNetStaticIp,configFlash.netConfig.staticIp);
  prefs.putULong(nvsNetMask,configFlash.netConfig.mask);
  prefs.putULong(nvsNetGW,configFlash.netConfig.gateway);
  prefs.putULong(nvsNetDnsIp,configFlash.netConfig.dnsIp); 
  prefs.putString(nvsNetSsid,configFlash.netConfig.ssid);
  prefs.putString(nvsNetPwd,configFlash.netConfig.pwd);
  prefs.putString(nvsNetUserName,configFlash.netConfig.userName);
  prefs.putString(nvsNetUserPwd,configFlash.netConfig.userPwd);
  prefs.putString(nvsNetTimeServer,configFlash.netConfig.timeServer);
  prefs.putUChar(nvsNetSysLogEnable,configFlash.netConfig.syslogLevel);
  prefs.putULong(nvsNetSysLogIp,configFlash.netConfig.syslogIp);
  prefs.putUShort(nvsNetSysLogPort,configFlash.netConfig.syslogPort);
  prefs.end();

  prefs.begin(nvsProtCfg,false);
  prefs.clear();
  prefs.putUChar(nvsProtDataProt,configFlash.protConfig.dataProtocol);
  if (configFlash.protConfig.dataProtocol>=protTypeMqtt) {
    prefs.putULong(nvsProtBrokerIp,configFlash.protConfig.brokerIp);
    prefs.putUShort(nvsProtBrokerPort,configFlash.protConfig.brokerPort);
    prefs.putULong(nvsProtBrokerInterval,configFlash.protConfig.brokerInterval);
    prefs.putULong(nvsProtPublishInterval,configFlash.protConfig.publishInterval);
    prefs.putString(nvsProtBrokerUser,configFlash.protConfig.userName);
    prefs.putString(nvsProtBrokerPwd,configFlash.protConfig.userPwd);
    uint8_t a = *(uint8_t *)&configFlash.protConfig.protocolFlags;
    prefs.putUChar(nvsProtBrokerPublishFlags,a);
    prefs.putUShort(nvsProtBrokerKeepAliveTime,configFlash.protConfig.keepAliveTime);
    prefs.putUShort(nvsProtBrokerMinBrokerDelay,configFlash.protConfig.minBrokerDelay);
    a = *(uint8_t *)&configFlash.protConfig.mqttConfig.flags;
    prefs.putUChar(nvsProtBrokerMQTTFlags,a);
    prefs.putULong(nvsProtBrokerMQTTTimeOut,configFlash.protConfig.mqttConfig.timeOut);
    prefs.putUChar(nvsProtBrokerMQTTToPos,configFlash.protConfig.mqttConfig.toPos);
  }
  prefs.end();
 
  prefs.begin(nvsValvesCfg,false);
  prefs.clear();
  prefs.putBytes(nvsValves, (void *) configFlash.valvesConfig.valveConfig, sizeof(configFlash.valvesConfig.valveConfig));
  prefs.putUChar(nvsDayOfCalib,configFlash.valvesConfig.dayOfCalib);
  prefs.putUChar(nvsHourOfCalib,configFlash.valvesConfig.hourOfCalib);
  prefs.end();

  prefs.begin(nvsTempsCfg,false);
  prefs.clear();
  prefs.putBytes(nvsTemps, (void *) configFlash.tempsConfig.tempConfig, sizeof(configFlash.tempsConfig.tempConfig));
  prefs.end();

  prefs.begin(nvsVoltsCfg,false);
  prefs.clear();
  prefs.putBytes(nvsVolts, (void *) &configFlash.voltsConfig, sizeof(configFlash.voltsConfig));
  prefs.end();
  
  prefs.begin(nvsTZCfg,false);
  prefs.clear();
  prefs.putString(nvsTZ,configFlash.timeZoneConfig.tz);
  prefs.putString(nvsTZCode,configFlash.timeZoneConfig.tzCode);
  prefs.end();

  prefs.begin(nvsMsgCfg,false);
  prefs.clear();
  uint8_t a = *(uint8_t *)&configFlash.messengerConfig.activeFlags;
  prefs.putUChar(nvsMsgCfgFlags,a);
  a = *(uint8_t *)&configFlash.messengerConfig.reason;
  prefs.putUChar(nvsMsgCfgReason,a);
  prefs.putShort(nvsMsgCfgMqttTimeout,configFlash.messengerConfig.reason.mqttTimeOutTime);
  prefs.putString(nvsMsgCfgPOAppToken,configFlash.messengerConfig.pushover.appToken);
  prefs.putString(nvsMsgCfgPOUserToken,configFlash.messengerConfig.pushover.userToken);
  prefs.putString(nvsMsgCfgPOTitle,configFlash.messengerConfig.pushover.title);

  prefs.putString(nvsMsgCfgEmailUser,configFlash.messengerConfig.email.user);
  prefs.putString(nvsMsgCfgEmailPwd,configFlash.messengerConfig.email.pwd);
  prefs.putString(nvsMsgCfgEMailHost,configFlash.messengerConfig.email.host);
  prefs.putShort(nvsMsgCfgEmailPort,configFlash.messengerConfig.email.port);
  prefs.putString(nvsMsgCfgEmailRecipient,configFlash.messengerConfig.email.recipient);
  prefs.putString(nvsMsgCfgEmailTitle,configFlash.messengerConfig.email.title);

  prefs.end();

  if (reboot) Services.restartSystem();
}

void CVdmConfig::writeMiscValues()
{
  prefs.begin(nvsMisc,false);
  prefs.putLong(nvsMiscLastCalib,miscValues.lastCalib);
  prefs.end();
}

void CVdmConfig::writeValvesControlConfig(bool reboot, bool restartTask)
{
  prefs.begin(nvsValvesControlCfg,false);
  prefs.clear();
  prefs.putBytes(nvsValvesControl, (void *) configFlash.valvesControlConfig.valveControlConfig, sizeof(configFlash.valvesControlConfig.valveControlConfig));
  prefs.putBytes(nvsValvesControl1, (void *) configFlash.valvesControl1Config.valveControl1Config, sizeof(configFlash.valvesControl1Config.valveControl1Config));
  prefs.putBytes(nvsValvesControlInit, (void *) configFlash.valvesControlInit.valveControlInit, sizeof(configFlash.valvesControlInit.valveControlInit));
  prefs.putUChar (nvsValvesControlHeatControl,configFlash.valvesControlConfig.heatControl);
  prefs.putUChar (nvsValvesControlParkPos,configFlash.valvesControlConfig.parkingPosition);
  prefs.end();
  if (VdmConfig.configFlash.netConfig.syslogLevel>=VISMODE_ATOMIC) {
    syslog.log(LOG_DEBUG, "pic: write control config, reboot = "+String(reboot)+" , restart = "+String(restartTask)+" , initiated = "+String(VdmTask.piTaskInitiated));
  }
  if (reboot) {
    Services.restartSystem();
  } else {
    if (VdmTask.piTaskInitiated) {
      if (restartTask) VdmTask.stopPIServices();
      VdmTask.startPIServices(restartTask);
    }
  }
}

void CVdmConfig::writeSysLogValues()
{
  prefs.begin(nvsNetCfg,false);
  prefs.putUChar(nvsNetSysLogEnable,configFlash.netConfig.syslogLevel);
  prefs.putULong(nvsNetSysLogIp,configFlash.netConfig.syslogIp);
  prefs.putUShort(nvsNetSysLogPort,configFlash.netConfig.syslogPort);
  prefs.end();
}

uint32_t CVdmConfig::doc2IPAddress(String id)
{
  IPAddress c1;
  bool b;
  b=c1.fromString(id) ; 
  if (b) return (uint32_t) c1; else return 0;
}

void CVdmConfig::postNetCfg (JsonObject doc)
{
  if (!doc["ethWifi"].isNull()) configFlash.netConfig.eth_wifi=doc["ethWifi"];
  if (!doc["dhcp"].isNull()) configFlash.netConfig.dhcpEnabled=doc["dhcp"];
  if (!doc["ip"].isNull()) configFlash.netConfig.staticIp=doc2IPAddress(doc["ip"]);
  if (!doc["mask"].isNull()) configFlash.netConfig.mask=doc2IPAddress(doc["mask"]);
  if (!doc["gw"].isNull()) configFlash.netConfig.gateway=doc2IPAddress(doc["gw"]);
  if (!doc["dns"].isNull()) configFlash.netConfig.dnsIp=doc2IPAddress(doc["dns"]);
  if (!doc["ssid"].isNull()) strncpy(configFlash.netConfig.ssid,doc["ssid"].as<const char*>(),sizeof(configFlash.netConfig.ssid));
  if (!doc["pwd"].isNull()) strncpy(configFlash.netConfig.pwd,doc["pwd"].as<const char*>(),sizeof(configFlash.netConfig.pwd));
  if (!doc["userName"].isNull()) strncpy(configFlash.netConfig.userName,doc["userName"].as<const char*>(),sizeof(configFlash.netConfig.userName));
  if (!doc["userPwd"].isNull()) strncpy(configFlash.netConfig.userPwd,doc["userPwd"].as<const char*>(),sizeof(configFlash.netConfig.userPwd));
  if (!doc["timeServer"].isNull()) strncpy(configFlash.netConfig.timeServer,doc["timeServer"].as<const char*>(),sizeof(configFlash.netConfig.timeServer));
  if (!doc["syslogLevel"].isNull()) configFlash.netConfig.syslogLevel=doc["syslogLevel"];
  if (!doc["syslogIp"].isNull()) configFlash.netConfig.syslogIp=doc2IPAddress(doc["syslogIp"]);
  if (!doc["syslogPort"].isNull()) configFlash.netConfig.syslogPort=doc["syslogPort"];
  if (!doc["tz"].isNull()) strncpy(configFlash.timeZoneConfig.tz,doc["tz"].as<const char*>(),sizeof(configFlash.timeZoneConfig.tz));
  if (!doc["tzCode"].isNull()) strncpy(configFlash.timeZoneConfig.tzCode,doc["tzCode"].as<const char*>(),sizeof(configFlash.timeZoneConfig.tzCode));
}

void CVdmConfig::postSysLogCfg (JsonObject doc)
{
  if (!doc["syslogLevel"].isNull()) configFlash.netConfig.syslogLevel=doc["syslogLevel"];
  if (!doc["syslogIp"].isNull()) configFlash.netConfig.syslogIp=doc2IPAddress(doc["syslogIp"]);
  if (!doc["syslogPort"].isNull()) configFlash.netConfig.syslogPort=doc["syslogPort"];
}

void CVdmConfig::postProtCfg (JsonObject doc)
{
  if (!doc["prot"].isNull()) configFlash.protConfig.dataProtocol = doc["prot"];
  if (!doc["ip"].isNull()) configFlash.protConfig.brokerIp = doc2IPAddress(doc["ip"]);
  if (!doc["port"].isNull()) configFlash.protConfig.brokerPort = doc["port"];
  if (!doc["interval"].isNull()) configFlash.protConfig.brokerInterval = doc["interval"];
  if (!doc["publish"].isNull()) configFlash.protConfig.publishInterval = doc["publish"];
  if (!doc["user"].isNull()) strncpy(configFlash.protConfig.userName,doc["user"].as<const char*>(),sizeof(configFlash.netConfig.userName));
  if (!doc["pwd"].isNull()) strncpy(configFlash.protConfig.userPwd,doc["pwd"].as<const char*>(),sizeof(configFlash.netConfig.userPwd));
  if (!doc["pubSeparate"].isNull()) configFlash.protConfig.protocolFlags.publishSeparate = doc["pubSeparate"];
  if (!doc["pubAllTemps"].isNull()) configFlash.protConfig.protocolFlags.publishAllTemps = doc["pubAllTemps"];
  if (!doc["pubPathAsRoot"].isNull()) configFlash.protConfig.protocolFlags.publishPathAsRoot = doc["pubPathAsRoot"];
  if (!doc["pubUpTime"].isNull()) configFlash.protConfig.protocolFlags.publishUpTime = doc["pubUpTime"];
  if (!doc["pubOnChange"].isNull()) configFlash.protConfig.protocolFlags.publishOnChange = doc["pubOnChange"];
  if (!doc["pubRetained"].isNull()) configFlash.protConfig.protocolFlags.publishRetained = doc["pubRetained"];
  if (!doc["pubPlainText"].isNull()) configFlash.protConfig.protocolFlags.publishPlainText = doc["pubPlainText"];
  if (!doc["pubDiag"].isNull()) configFlash.protConfig.protocolFlags.publishDiag = doc["pubDiag"];
  if (!doc["keepAliveTime"].isNull()) configFlash.protConfig.keepAliveTime = doc["keepAliveTime"];
  if (!doc["pubMinDelay"].isNull()) configFlash.protConfig.minBrokerDelay = doc["pubMinDelay"];
  if (!doc["mqttTOTSActive"].isNull()) configFlash.protConfig.mqttConfig.flags.timeoutTSActive = doc["mqttTOTSActive"];
  if (!doc["mqttTODSActive"].isNull()) configFlash.protConfig.mqttConfig.flags.timeoutDSActive = doc["mqttTODSActive"];
  if (!doc["mqttTO"].isNull()) configFlash.protConfig.mqttConfig.timeOut = doc["mqttTO"];
  if (!doc["mqttToPos"].isNull()) configFlash.protConfig.mqttConfig.toPos = doc["mqttToPos"];
  if (!doc["numFormat"].isNull()) configFlash.protConfig.mqttConfig.flags.numFormat = doc["numFormat"];
}

void CVdmConfig::postValvesCfg (JsonObject doc)
{
  uint8_t idx=0; 
  size_t size=doc["valves"].size();
  if (!doc["calib"]["dayOfCalib"].isNull()) configFlash.valvesConfig.dayOfCalib=doc["calib"]["dayOfCalib"];
  if (!doc["calib"]["hourOfCalib"].isNull()) configFlash.valvesConfig.hourOfCalib=doc["calib"]["hourOfCalib"];

  for (uint8_t i=0; i<size; i++) {
     if (!doc["valves"][i]["no"].isNull()) {
      idx=doc["valves"][i]["no"];
      idx--;
      if ((idx>=0) && (idx<12)) {
          if (!doc["valves"][i]["name"].isNull()) strncpy(configFlash.valvesConfig.valveConfig[idx].name,doc["valves"][i]["name"].as<const char*>(),sizeof(configFlash.valvesConfig.valveConfig[idx].name));
          if (!doc["valves"][i]["active"].isNull()) configFlash.valvesConfig.valveConfig[idx].active=doc["valves"][i]["active"];
          if (!doc["valves"][i]["tIdx1"].isNull()) {
            StmApp.actuators[idx].tIdx1=doc["valves"][i]["tIdx1"];
            StmApp.setTempIdxActive=true;
          }
          if (!doc["valves"][i]["tIdx2"].isNull()) {
            StmApp.actuators[idx].tIdx2=doc["valves"][i]["tIdx2"];
            StmApp.setTempIdxActive=true;
          }
      }
     }
  }    
  if (!doc["calib"]["cycles"].isNull()) {
    StmApp.learnAfterMovements=doc["calib"]["cycles"];
    StmApp.setLearnAfterMovements();
  }
  if (!doc["motor"]["lowC"].isNull()) {
    StmApp.motorChars.maxLowCurrent=doc["motor"]["lowC"];
    StmApp.setMotorCharsActive=true;
  }
  if (!doc["motor"]["highC"].isNull()) {
    StmApp.motorChars.maxHighCurrent=doc["motor"]["highC"];
    StmApp.setMotorCharsActive=true;
  }
  if (!doc["motor"]["startOnPower"].isNull()) {
    StmApp.motorChars.startOnPower=doc["motor"]["startOnPower"];
    StmApp.setMotorCharsActive=true;
  }
  if (!doc["motor"]["noOfMinCount"].isNull()) {
    StmApp.motorChars.noOfMinCount=doc["motor"]["noOfMinCount"];
    StmApp.setMotorCharsActive=true;
  }
  if (!doc["motor"]["maxCalReps"].isNull()) {
    StmApp.motorChars.maxCalReps=doc["motor"]["maxCalReps"];
    StmApp.setMotorCharsActive=true;
  }
  if (StmApp.setMotorCharsActive) {
    StmApp.setMotorChars();
    StmApp.setMotorCharsActive=false;
  }
  if (!doc["tempIdx"]["set"].isNull()) {
    uint8_t setTemp=doc["tempIdx"]["set"]; 
    if ((StmApp.setTempIdxActive) && (setTemp==1)) {
      StmApp.setTempIdx();
      //StmApp.matchSensors();
      //StmApp.matchSensorRequest = true;
      StmApp.setTempIdxActive=false;
    }
  }
}

void CVdmConfig::postValvesControlCfg (JsonObject doc)
{
  uint8_t idx=0;
  size_t size=doc["valves"].size();

  if (size==0) {
      for (uint8_t i =0;i<12;i++) {
        if (!doc["active"].isNull()) configFlash.valvesControlConfig.valveControlConfig[idx].controlFlags.active=doc["active"];
        if (!doc["allow"].isNull()) configFlash.valvesControlConfig.valveControlConfig[i].controlFlags.allow=doc["allow"]; 
        if (!doc["window"].isNull()) configFlash.valvesControlConfig.valveControlConfig[idx].controlFlags.windowInstalled=doc["window"];
        if (!doc["vSource"].isNull()) configFlash.valvesControlConfig.valveControlConfig[idx].valueSource=doc["vSource"];
        if (!doc["tSource"].isNull()) configFlash.valvesControlConfig.valveControlConfig[idx].targetSource=doc["tSource"];  
        if (!doc["scheme"].isNull()) configFlash.valvesControlConfig.valveControlConfig[idx].scheme=doc["scheme"];  
        if (!doc["startAZ"].isNull()) configFlash.valvesControlConfig.valveControlConfig[idx].startActiveZone=doc["startAZ"];  
        if (!doc["endAZ"].isNull()) configFlash.valvesControlConfig.valveControlConfig[idx].endActiveZone=doc["endAZ"]; 
        if (!doc["inittTarget"].isNull()) configFlash.valvesControlInit.valveControlInit[idx].tTarget=doc["inittTarget"];
        if (!doc["deadband"].isNull()) configFlash.valvesControl1Config.valveControl1Config[idx].deadband=doc["deadband"];  
      }
  } else {
    for (uint8_t i=0; i<size; i++) {
      if (!doc["valves"][i]["no"].isNull()) {
        idx=doc["valves"][i]["no"];
        idx--;
        if ((idx>=0) && (idx<12)) {
          if (!doc["valves"][i]["active"].isNull()) configFlash.valvesControlConfig.valveControlConfig[idx].controlFlags.active=doc["valves"][i]["active"];
          if (!doc["valves"][i]["allow"].isNull()) configFlash.valvesControlConfig.valveControlConfig[idx].controlFlags.allow=doc["valves"][i]["allow"];
          if (!doc["valves"][i]["window"].isNull()) configFlash.valvesControlConfig.valveControlConfig[idx].controlFlags.windowInstalled=doc["valves"][i]["window"];
          if (!doc["valves"][i]["link"].isNull()) configFlash.valvesControlConfig.valveControlConfig[idx].link=doc["valves"][i]["link"];
          if (!doc["valves"][i]["vSource"].isNull()) configFlash.valvesControlConfig.valveControlConfig[idx].valueSource=doc["valves"][i]["vSource"];
          if (!doc["valves"][i]["tSource"].isNull()) configFlash.valvesControlConfig.valveControlConfig[idx].targetSource=doc["valves"][i]["tSource"];
          if (!doc["valves"][i]["xp"].isNull()) configFlash.valvesControlConfig.valveControlConfig[idx].xp=doc["valves"][i]["xp"];
          if (!doc["valves"][i]["offset"].isNull()) configFlash.valvesControlConfig.valveControlConfig[idx].offset=doc["valves"][i]["offset"];
          if (!doc["valves"][i]["ti"].isNull()) configFlash.valvesControlConfig.valveControlConfig[idx].ti=doc["valves"][i]["ti"];
          if (!doc["valves"][i]["ts"].isNull()) {
            uint16_t ts = doc["valves"][i]["ts"];
            if (configFlash.valvesControlConfig.valveControlConfig[idx].ts!=ts) VdmTask.restartPiTask=true;
            configFlash.valvesControlConfig.valveControlConfig[idx].ts=ts;
          }
          if (!doc["valves"][i]["ki"].isNull()) configFlash.valvesControlConfig.valveControlConfig[idx].ki=doc["valves"][i]["ki"];  
          if (!doc["valves"][i]["scheme"].isNull()) configFlash.valvesControlConfig.valveControlConfig[idx].scheme=doc["valves"][i]["scheme"];  
          if (!doc["valves"][i]["startAZ"].isNull()) configFlash.valvesControlConfig.valveControlConfig[idx].startActiveZone=doc["valves"][i]["startAZ"];  
          if (!doc["valves"][i]["endAZ"].isNull()) configFlash.valvesControlConfig.valveControlConfig[idx].endActiveZone=doc["valves"][i]["endAZ"];  
          if (!doc["valves"][i]["inittTarget"].isNull()) configFlash.valvesControlInit.valveControlInit[idx].tTarget=doc["valves"][i]["inittTarget"];  
          if (!doc["valves"][i]["deadband"].isNull()) configFlash.valvesControl1Config.valveControl1Config[idx].deadband=doc["valves"][i]["deadband"];
        }
      }
    } 
  }
  if (!doc["common"]["heatControl"].isNull()) {
    configFlash.valvesControlConfig.heatControl=doc["common"]["heatControl"];
    heatValues.heatControl=configFlash.valvesControlConfig.heatControl;
  } 
  if (!doc["common"]["parkPosition"].isNull()) {
    configFlash.valvesControlConfig.parkingPosition=doc["common"]["parkPosition"];
    heatValues.parkPosition=configFlash.valvesControlConfig.parkingPosition;
  } 
}

void CVdmConfig::postTempsCfg (JsonObject doc)
{
  uint8_t chunkStart=doc["chunkStart"];
  uint8_t chunkEnd=doc["chunkEnd"];
  uint8_t idx=0;
 
  for (uint8_t i=chunkStart-1; i<chunkEnd; i++) {
    if (!doc["temps"][idx]["name"].isNull()) strncpy(configFlash.tempsConfig.tempConfig[i].name,doc["temps"][idx]["name"].as<const char*>(),sizeof(configFlash.tempsConfig.tempConfig[i].name));
    if (!doc["temps"][idx]["id"].isNull()) strncpy(configFlash.tempsConfig.tempConfig[i].ID,doc["temps"][idx]["id"].as<const char*>(),sizeof(configFlash.tempsConfig.tempConfig[i].ID));
    if (!doc["temps"][idx]["active"].isNull()) configFlash.tempsConfig.tempConfig[i].active=doc["temps"][idx]["active"];
    if (!doc["temps"][idx]["offset"].isNull()) configFlash.tempsConfig.tempConfig[i].offset=10*(doc["temps"][idx]["offset"].as<float>()) ;
    idx++;
  }
}

void CVdmConfig::postVoltsCfg (JsonObject doc)
{
  size_t size=doc["volts"].size(); 
  for (uint8_t i=0; i<size; i++) {
    if (!doc["volts"][i]["name"].isNull()) strncpy(configFlash.voltsConfig.voltConfig[i].name,doc["volts"][i]["name"].as<const char*>(),sizeof(configFlash.voltsConfig.voltConfig[i].name));
    if (!doc["volts"][i]["id"].isNull()) strncpy(configFlash.voltsConfig.voltConfig[i].ID,doc["volts"][i]["id"].as<const char*>(),sizeof(configFlash.voltsConfig.voltConfig[i].ID));
    if (!doc["volts"][i]["active"].isNull()) configFlash.voltsConfig.voltConfig[i].active=doc["volts"][i]["active"];
    if (!doc["volts"][i]["offset"].isNull()) configFlash.voltsConfig.voltConfig[i].offset=(doc["volts"][i]["offset"].as<float>()) ;
    if (!doc["volts"][i]["factor"].isNull()) configFlash.voltsConfig.voltConfig[i].factor=(doc["volts"][i]["factor"].as<float>()) ;
    if (!doc["volts"][i]["unit"].isNull()) strncpy(configFlash.voltsConfig.voltConfig[i].unit,doc["volts"][i]["unit"].as<const char*>(),sizeof(configFlash.voltsConfig.voltConfig[i].unit));
  }
}


void CVdmConfig::postSysCfg (JsonObject doc)
{
  if (!doc["CF"].isNull()) configFlash.systemConfig.celsiusFahrenheit = doc["CF"];
  if (!doc["station"].isNull()) strncpy(configFlash.systemConfig.stationName,doc["station"].as<const char*>(),sizeof(configFlash.systemConfig.stationName));
 
}

void CVdmConfig::postMessengerCfg (JsonObject doc)
{
  if (!doc["reason"]["valveFailed"].isNull()) configFlash.messengerConfig.reason.reasonFlags.valveFailed=doc["reason"]["valveFailed"];
  if (!doc["reason"]["notDetect"].isNull()) configFlash.messengerConfig.reason.reasonFlags.notDetect=doc["reason"]["notDetect"];
  if (!doc["reason"]["mqttTimeOut"].isNull()) configFlash.messengerConfig.reason.reasonFlags.mqttTimeOut=doc["reason"]["mqttTimeOut"];
  if (!doc["reason"]["mqttTimeOutTime"].isNull()) configFlash.messengerConfig.reason.mqttTimeOutTime=doc["reason"]["mqttTimeOutTime"];
  if (!doc["reason"]["reset"].isNull()) configFlash.messengerConfig.reason.reasonFlags.reset=doc["reason"]["reset"];
  if (!doc["reason"]["ds18Failed"].isNull()) configFlash.messengerConfig.reason.reasonFlags.ds18Failed=doc["reason"]["ds18Failed"];
  if (!doc["reason"]["tValueFailed"].isNull()) configFlash.messengerConfig.reason.reasonFlags.tValueFailed=doc["reason"]["tValueFailed"];
   if (!doc["reason"]["valveBlocked"].isNull()) configFlash.messengerConfig.reason.reasonFlags.valveBlocked=doc["reason"]["valveBlocked"];

  if (!doc["PO"]["active"].isNull()) configFlash.messengerConfig.activeFlags.pushOver=doc["PO"]["active"];
  if (!doc["PO"]["appToken"].isNull()) strncpy(configFlash.messengerConfig.pushover.appToken,doc["PO"]["appToken"].as<const char*>(),sizeof(configFlash.messengerConfig.pushover.appToken));
  if (!doc["PO"]["userToken"].isNull()) strncpy(configFlash.messengerConfig.pushover.userToken,doc["PO"]["userToken"].as<const char*>(),sizeof(configFlash.messengerConfig.pushover.userToken));
  if (!doc["PO"]["title"].isNull()) strncpy(configFlash.messengerConfig.pushover.title,doc["PO"]["title"].as<const char*>(),sizeof(configFlash.messengerConfig.pushover.title));
  
  if (!doc["Email"]["active"].isNull()) configFlash.messengerConfig.activeFlags.email=doc["Email"]["active"];
  if (!doc["Email"]["user"].isNull()) strncpy(configFlash.messengerConfig.email.user,doc["Email"]["user"].as<const char*>(),sizeof(configFlash.messengerConfig.email.user));
  if (!doc["Email"]["pwd"].isNull()) strncpy(configFlash.messengerConfig.email.pwd,doc["Email"]["pwd"].as<const char*>(),sizeof(configFlash.messengerConfig.email.pwd));
  if (!doc["Email"]["host"].isNull()) strncpy(configFlash.messengerConfig.email.host,doc["Email"]["host"].as<const char*>(),sizeof(configFlash.messengerConfig.email.host));
  if (!doc["Email"]["port"].isNull()) configFlash.messengerConfig.email.port=doc["Email"]["port"];
  if (!doc["Email"]["recipient"].isNull()) strncpy(configFlash.messengerConfig.email.recipient,doc["Email"]["recipient"].as<const char*>(),sizeof(configFlash.messengerConfig.email.recipient));
  if (!doc["Email"]["title"].isNull()) strncpy(configFlash.messengerConfig.email.title,doc["Email"]["title"].as<const char*>(),sizeof(configFlash.messengerConfig.email.title));
}

String CVdmConfig::handleAuth (JsonObject doc)
{
  bool userCheck=false;
  bool pwdCheck=false;
  uint8_t result=0;
  if ((strlen(configFlash.netConfig.userName)>0) && (strlen(configFlash.netConfig.userPwd)>0)) {
    if (!(doc["user"].isNull() || doc["pwd"].isNull())) {
      const char* du=doc["user"].as<const char*>();
      const char* dp=doc["pwd"].as<const char*>();
      userCheck=(strncmp (configFlash.netConfig.userName,du,sizeof(configFlash.netConfig.userName)))==0;
      pwdCheck=(strncmp (configFlash.netConfig.userPwd,dp,sizeof(configFlash.netConfig.userPwd)))==0;
    }
    if (userCheck) result+=1;  // userName compares
    if (pwdCheck) result+=2;  // pwdName compares
  } else result=3;
  return ("{\"auth\":"+String(result)+"}");
}



void CVdmConfig::checkToResetCfg() {
  pinMode(pinSetFactoryCfg, INPUT_PULLUP);
  if (digitalRead(pinSetFactoryCfg)==0) {  
    #ifdef EnvDevelop
      UART_DBG.println("Entry reset config");
    #endif
    delay(1000);
    if (digitalRead(pinSetFactoryCfg)==0) {
        #ifdef EnvDevelop
          UART_DBG.println("Reset config, remove shortcut");
        #endif
        while (digitalRead(pinSetFactoryCfg)==0) {
          delay(100);
        }
        #ifdef EnvDevelop
          UART_DBG.println("Reset config now");
        #endif
        VdmConfig.restoreConfig(true);
    }
  }
}