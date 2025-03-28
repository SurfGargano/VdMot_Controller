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


#include "PIControl.h"
#include "time.h"
#include "globals.h"
#include "stmApp.h"
#include "VdmConfig.h"
#include "VdmNet.h"
#include <Syslog.h>
#include "RtcRam.h"


CPiControl PiControl[ACTUATOR_COUNT];

void CPiControl::savePiControl() {
  piControlRtcs.piControlRtc[valveIndex].target=target;
  piControlRtcs.piControlRtc[valveIndex].value=value;
  piControlRtcs.piControlRtc[valveIndex].iProp=iProp;
  piControlRtcs.piControlRtc[valveIndex].esum=esum;
  strncpy(piControlRtcs.piControlRtc[valveIndex].pattern,RTC_RAM_PATTERN,sizeof(piControlRtcs.piControlRtc[valveIndex].pattern));
  piControlRtcs.piControlRtc[valveIndex].lastControlPosition=lastControlPosition;
}

void CPiControl::reloadPiControl() {
  if (strncmp(piControlRtcs.piControlRtc[valveIndex].pattern,RTC_RAM_PATTERN,sizeof(RTC_RAM_PATTERN))==0) {
    target=piControlRtcs.piControlRtc[valveIndex].target;
    value=piControlRtcs.piControlRtc[valveIndex].value;
    iProp=piControlRtcs.piControlRtc[valveIndex].iProp;
    esum=piControlRtcs.piControlRtc[valveIndex].esum;
    lastControlPosition=piControlRtcs.piControlRtc[valveIndex].lastControlPosition;
    start=true;
    time(&ts);
    setValveAction(lastControlPosition);
  //  UART_DBG.println("Reload pi valve # "+String(valveIndex+1));
    if (VdmConfig.configFlash.netConfig.syslogLevel>=VISMODE_ATOMIC) {
      syslog.log(LOG_DEBUG, "pic: reload pi valve # "+String(valveIndex+1)+" ("+String(VdmConfig.configFlash.valvesConfig.valveConfig[valveIndex].name)+") tTarget = "+String(target)+" tValue = "+String(value)+" iProp = "+String(iProp)+" esum = "+String(esum)+" position = "+String(lastControlPosition));
    }
  } else  {
    if (VdmConfig.configFlash.netConfig.syslogLevel>=VISMODE_ATOMIC) {
      syslog.log(LOG_DEBUG, "pic: no reload pi valve # "+String(valveIndex+1)+" ("+String(VdmConfig.configFlash.valvesConfig.valveConfig[valveIndex].name)+")");
    }
   // UART_DBG.println("No reload pi valve # "+String(valveIndex+1));
    setValveAction(VdmConfig.configFlash.valvesControlInit.valveControlInit[valveIndex].tTarget);
  }
}

double CPiControl::getpProp(float e) {
   // p - proportion
   double p = kp * e + 50 + offset + dynOffset;
   if (p > 100.0) p = 100.0;
   if (p < 0.0) p = 0.0;  
   return p;
}

float CPiControl::piCtrl(float target,float value) {
  double p;
  double y;
  double e;
  double dt;
  time_t now;

  if ((xp==0) || (ti==0)) return (startValvePos);
  
  if (!start) {
    time(&ts);
    iProp=startValvePos-50;
    start=true;
    esum=0;
  }

  old_esum=esum;

  time(&now);
  ta=difftime(now,ts);      // in sec
  time(&ts);
 
  dt =  ((double)ta / (double)ti);

  if (windowControlState == windowCloseRestore) {
    ta = VdmConfig.configFlash.valvesControlConfig.valveControlConfig[valveIndex].ts;
    windowControlState = windowIdle;
  }
  
  if (VdmConfig.heatValues.heatControl==piControlOnHeating)
  {
    e=target-value;
  } else {
    e=value-target;
  } 

  switch (scheme) {
    default:
    case piControlDynamicKi : 
    {
      // p - proportion
      p = getpProp(e);
      // i - proportion
      iProp += dt * kp * e;
      // pi
      y = p + iProp;
      break;
    } 
    case piControlStaticKi : 
    {
      // p - proportion
      //p = getpProp(e);
      p = kp * e + 50 + offset + dynOffset;
      esum += e*dt;
      iProp = ki * esum;
      // pi
      y = p + iProp;
    break;
    }
  }

  if (VdmConfig.configFlash.netConfig.syslogLevel>=VISMODE_ATOMIC) {
    syslog.log(LOG_DEBUG, "pic: calc valve position : #"+String(valveIndex+1)+" ("+String(VdmConfig.configFlash.valvesConfig.valveConfig[valveIndex].name)+") tTarget = "+String(target)+" tValue = "+String(value)+" diff = "+String(e)+" iProp = "+String(iProp)+" kp = "+String(kp)+" esum = "+String(esum)+" p = "+String(p)+" y = "+String(y));
  }

  if (y > 100.0) {
    y = 100.0;
    iProp = 100.0 - p;
    if (esum>old_esum) esum=old_esum;
  }
  if (y < 0.0) {
    y = 0.0;
    iProp = 0.0 - p;
    if (esum<old_esum) esum=old_esum;
  }

  if (VdmConfig.configFlash.netConfig.syslogLevel>=VISMODE_ATOMIC) {
    syslog.log(LOG_DEBUG, "pic: calc valve position corrected : #"+String(valveIndex+1)+" ("+String(VdmConfig.configFlash.valvesConfig.valveConfig[valveIndex].name)+") tTarget = "+String(target)+" tValue = "+String(value)+" diff = "+String(e)+" iProp = "+String(iProp)+" kp = "+String(kp)+" esum = "+String(esum)+" p = "+String(p)+" y = "+String(y));
  }

  return y;
}

void CPiControl::setWindowAction(uint8_t windowControl)
{
 if (VdmConfig.configFlash.valvesControlConfig.valveControlConfig[valveIndex].controlFlags.windowInstalled && controlActive && 
    (VdmConfig.heatValues.heatControl==1 || VdmConfig.heatValues.heatControl==2))
 {
  windowState = windowControl;
  switch (windowControl) {
    case windowActionCloseRestore: {
      windowControlState = windowCloseRestore;
      setValveAction(lastControlPosition);
      break;
    }
    case windowActionOpenLock: {
      windowControlState = windowOpenLock;
      setValveAction(windowOpenTarget);
      break;
    }
  }
 } else {
  if (windowControlState == windowOpenLock) {
    windowControlState = windowCloseRestore;
    setValveAction(lastControlPosition);
  }
 }
}

void CPiControl::setValveAction(uint8_t valvePosition) {
  if (!(failed) && (VdmConfig.configFlash.valvesControlConfig.valveControlConfig[valveIndex].controlFlags.active && controlActive)) {
    StmApp.actuators[valveIndex].target_position=valvePosition;
    if (VdmConfig.configFlash.netConfig.syslogLevel>=VISMODE_ATOMIC) {
      syslog.log(LOG_DEBUG, "pic: setValveAction valve position has changed : "+String(VdmConfig.configFlash.valvesConfig.valveConfig[valveIndex].name)+"(#"+String(valveIndex+1)+") = "+String(valvePosition)+" Ttarget "+String(target)+" Tvalue "+String(value));
    }
    // check if there are links
    for (uint8_t i=0; i< ACTUATOR_COUNT; i++) {
      if (VdmConfig.configFlash.valvesControlConfig.valveControlConfig[i].controlFlags.active) {
        if (VdmConfig.configFlash.valvesControlConfig.valveControlConfig[i].link==(valveIndex+1)) {
          StmApp.actuators[i].target_position=valvePosition;
          if (VdmConfig.configFlash.netConfig.syslogLevel>=VISMODE_ATOMIC) {
            syslog.log(LOG_DEBUG, "pic:setValveAction link valve position has changed : "+String(VdmConfig.configFlash.valvesConfig.valveConfig[i].name)+"(#"+String(i+1)+") = "+String(valvePosition));
          }
        }
      }
    }
  }
}


CHECKACTION CPiControl::checkAction(uint8_t idx)
{
  if (!controlActive) return(nothing);
  if (!VdmConfig.configFlash.valvesControlConfig.valveControlConfig[valveIndex].controlFlags.active)  return(nothing);
  if (failed) return(nothing);

  if (!VdmConfig.configFlash.valvesControlConfig.valveControlConfig[valveIndex].controlFlags.windowInstalled) {
    windowControlState = windowIdle;  
  }
  if (windowControlState == windowOpenLock) {
    setPosition(windowOpenTarget);
    return(nothing);
  }
  
  switch (VdmConfig.heatValues.heatControl)
  {
    case piControlManual:
      return(nothing);
    case piControlOnHeating:
      if ((VdmConfig.configFlash.valvesControlConfig.valveControlConfig[idx].controlFlags.allow==piControlAllowedHeatingCooling) || (VdmConfig.configFlash.valvesControlConfig.valveControlConfig[idx].controlFlags.allow==piControlAllowedHeating))
      {
        return(control); 
      }
      else {
        return(gotoMin);  
      }
    case piControlOnCooling:
      if ((VdmConfig.configFlash.valvesControlConfig.valveControlConfig[idx].controlFlags.allow==piControlAllowedHeatingCooling) || (VdmConfig.configFlash.valvesControlConfig.valveControlConfig[idx].controlFlags.allow==piControlAllowedCooling))
      {
        return(control); 
      }
      else {
        return(gotoMin);  
      }
    case piControlOff:
      return (gotoPark);
  } 
  return (nothing);
}


uint8_t CPiControl::calcValve()
{
  float piValue=0;
  piValue=piCtrl(target,value);
  return (round(piValue));
}

void CPiControl::controlValve() 
{
  CHECKACTION check=checkAction(valveIndex);
 
  switch (check) {
    case nothing:
      break;
    case gotoMin:
      setPosition(startActiveZone);
      break;
    case gotoPark:
      setPosition(VdmConfig.heatValues.parkPosition);
      break;
    case control:
      doControlValve();
      break;
  }
}

void CPiControl::setPosition(uint8_t thisPosition)
{
  StmApp.actuators[valveIndex].target_position=thisPosition;
     // check if there are links
     for (uint8_t i=0; i< ACTUATOR_COUNT; i++) {
      if (VdmConfig.configFlash.valvesControlConfig.valveControlConfig[i].controlFlags.active) {
        if (VdmConfig.configFlash.valvesControlConfig.valveControlConfig[i].link==(valveIndex+1)) {
          StmApp.actuators[i].target_position=thisPosition;
          if (VdmConfig.configFlash.netConfig.syslogLevel>=VISMODE_DETAIL) {
            syslog.log(LOG_DEBUG, "pic:link valve position has changed to : "+String(VdmConfig.configFlash.valvesConfig.valveConfig[i].name)+"(#"+String(i+1)+") = "+String(thisPosition));
          }
        }
      }
    }
}
  
bool CPiControl::getValueFromSource()
{
  if (VdmConfig.configFlash.valvesControlConfig.valveControlConfig[valveIndex].valueSource==1) {
      if (StmApp.actuators[valveIndex].temp1>-500) 
        value=((float)StmApp.actuators[valveIndex].temp1)/10;
      else return (false);
    }
    if (VdmConfig.configFlash.valvesControlConfig.valveControlConfig[valveIndex].valueSource==2) {
      if (StmApp.actuators[valveIndex].temp2>-500) 
        value=((float)StmApp.actuators[valveIndex].temp2)/10;
      else return (false);
    }
    return (true);
}

void CPiControl::doControlValve() 
{
    uint8_t valvePosition;
    if (!getValueFromSource()) return;
    // deadband
    if (VdmConfig.configFlash.valvesControl1Config.valveControl1Config[valveIndex].deadband>0) {
      if (abs(target-value)<VdmConfig.configFlash.valvesControl1Config.valveControl1Config[valveIndex].deadband) return;
    }
    float newValveValue=calcValve();
    if (endActiveZone>startActiveZone) {
      valvePosition=round((((float)(endActiveZone-startActiveZone))*newValveValue/100.0)+startActiveZone); 
    } else valvePosition=round(newValveValue);
    if (valvePosition>100) {
      if (VdmConfig.configFlash.netConfig.syslogLevel>=VISMODE_DETAIL) {
        syslog.log(LOG_DEBUG, "pic:valve position exceeds 100 : (#"+String(valveIndex+1)+") = "+String(valvePosition));
      }
      valvePosition=100;
    }
    if (controlActive==0) valvePosition=0; 
    setPosition(valvePosition);
    lastControlPosition=valvePosition;
    savePiControl();
}
  
void CPiControl::setControlActive(uint8_t thisControl)
{
  controlActive=thisControl;
  // check if there are links
   for (uint8_t i=0; i< ACTUATOR_COUNT; i++) {
    if (VdmConfig.configFlash.valvesControlConfig.valveControlConfig[i].controlFlags.active) {
      if (VdmConfig.configFlash.valvesControlConfig.valveControlConfig[i].link==(valveIndex+1)) {
        PiControl[i].controlActive=thisControl;
      }
    }
  }
}

void CPiControl::setFailed(uint8_t thisPosition)
{
  failed=true;
  StmApp.actuators[valveIndex].target_position=thisPosition;
  // check if there are links
  for (uint8_t i=0; i< ACTUATOR_COUNT; i++) {
    if (VdmConfig.configFlash.valvesControlConfig.valveControlConfig[i].link==(valveIndex+1)) {
      StmApp.actuators[i].target_position=thisPosition;
    }
  }
}