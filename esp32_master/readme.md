






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






