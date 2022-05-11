# Software installation

This picture shows essential connectors and jumpers of the VdMot Controller Board:
![-](./hardware/tht_assembly_C2-sample.png "tht assembly drawing C2 revision")

    B1 - BlackPill Board
    B2 - WT32-ETH01 Board


# Initial ESP32 / WT32 Flash via direct upload in Platformio / Visual Studio Code
You will need a USB2UART converter for that step. For example a converter with FTDI FT232 or CH340.

Following configuration of platformio.ini is needed:

    upload_port = COM Port of FTDI Board
    upload_speed = 115200 (or faster)
    
    
Please proceed according following steps:

    1. Connect FT232 to VdMot Controller Board X23 - male header for UART connection
        - X23:Pin1 (GND) -> FT232 GND 
        - X23:Pin2 (RX)  -> FT232 TX 
        - X23:Pin3 (TX)  -> FT232 RX 
    2. Short jumper X22 - this brings ESP32 into flash mode
    3. Power up VdMot Controller
    4. Start upload directly from Platformio in Visual Studio Code. 



# Initial ESP32 / WT32 Flash via Flash Download Tool
You will need a USB2UART converter for that step. For example a converter with FTDI FT232 or CH340.

Following configuration of the flash download tool is needed:

    TBD
    
    
Please proceed according following steps:

    1. Connect FT232 to VdMot Controller Board X23 - male header for UART connection
        - X23:Pin1 (GND) -> FT232 GND 
        - X23:Pin2 (RX)  -> FT232 TX 
        - X23:Pin3 (TX)  -> FT232 RX 
    2. Short jumper X22 - this brings ESP32 into flash mode
    3. Power up VdMot Controller
    4. Click start
    

# Initial STM32 Flash via VdMot Controller
The STM32 can be flashed directly by the ESP32 VdMot Controller.

Please proceed according following steps:

    1. Short jumper X20 - this allows the ESP32 to Reset STM32 and start the Bootload sequence
    2. Power up VdMot Controller
    3. Open VdMot Controller website and goto "Update"
    4. Click on "Update STM32"
    5. Choose the STM32 firmware file to upload
    6. Click "Upload STM32 file"
    7. Choose Option "boot partition" for initial flashing
    8. After Upload choose correct file in filelist
    9. Press BOOT0 Button at BlackPill Board until flashing progress is >15%
    10. Click "Upload STM32 now"

![-](./stm32_initial_flash.png "STM32 initial flash")