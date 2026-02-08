

### ESP32 characteristics and limitations : 

Datasheet : https://documentation.espressif.com/esp32_datasheet_en.pdf

1. onboard 3.3 V regulator (LDO) can usually supply ~250 mA max safely.

```c++
=== ESP32 Chip Info ===
Model: ESP32
Cores: 2
Revision: 3
Features: WiFi BLE BT
Flash: 4MB external
Free heap: 337064 bytes
CPU frequency: 240 MHz
SDK version: v4.4.7-dirty
Sketch size: 288896 bytes
Free sketch space: 1310720 bytes
Chip ID: C1D8CBB0
```

#### ??? : 
Datasheet :
- 8 KB of SRAM in RTC, which is called RTC FAST Memory and can be used for data storage; it is accessed
by the main CPU during RTC Boot from the Deep-sleep mode.
- 8 KB of SRAM in RTC, which is called RTC SLOW Memory and can be accessed by the ULP coprocessor
during the Deep-sleep mode



### FreeRtos Tasks : 





### Ultrasonic Sensors: HC-SR04 & ESP32

**ESP32 max GPIO voltage:** 3.6V  
**HC-SR04 Echo pin output:** 5V

#### Voltage Divider Calculation :

To safely connect the Echo pin to ESP32, use a voltage divider:

$$
V_{out} = V_{in} \times \frac{R_2}{R_1 + R_2}
$$
- $V_{in}$ = 5V
- $V_{out}$ = desired output (≤3.3V)
- $R_1$ = resistor from Echo to ESP32 GPIO
- $R_2$ = resistor from ESP32 GPIO to GND

**Example:**  
$R_1 = 2k\Omega$, 

$R_2 = 4k\Omega$

$$
V_{out} = 5 \times \frac{4}{2+4} = 3.33\,V
$$

#### Pinout : 
```
HC-SR04 ECHO ---[R1]---+--- ESP32 GPIO
                        |
                      [R2]
                        |
                       GND
```






