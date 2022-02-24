EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Personal Weather Station on Arduino Nano-33-IoT"
Date "2021-07-03"
Rev "1.0"
Comp "Radek Kaczorek"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L power:GND #PWR02
U 1 1 60E35DCE
P 2000 5000
F 0 "#PWR02" H 2000 4750 50  0001 C CNN
F 1 "GND" H 2005 4827 50  0000 C CNN
F 2 "" H 2000 5000 50  0001 C CNN
F 3 "" H 2000 5000 50  0001 C CNN
	1    2000 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 4750 2000 4750
Wire Wire Line
	2000 4750 2000 5000
Text GLabel 1900 4550 2    50   Input ~ 0
SDA
Text GLabel 1900 4650 2    50   Input ~ 0
SCL
Text GLabel 1900 4450 2    50   Input ~ 0
3V3
$Comp
L Sensor:BME280 U1
U 1 1 60E40D52
P 2050 2400
F 0 "U1" H 1621 2446 50  0000 R CNN
F 1 "BME280" H 1621 2355 50  0000 R CNN
F 2 "Housings_LGA:Bosch_LGA-8_2.5x2.5mm_Pitch0.65mm_ClockwisePinNumbering" H 3550 1950 50  0001 C CNN
F 3 "https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BME280-DS002.pdf" H 2050 2200 50  0001 C CNN
	1    2050 2400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 60E05656
P 1950 3000
F 0 "#PWR05" H 1950 2750 50  0001 C CNN
F 1 "GND" H 1955 2827 50  0000 C CNN
F 2 "" H 1950 3000 50  0001 C CNN
F 3 "" H 1950 3000 50  0001 C CNN
	1    1950 3000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 60E05A69
P 2150 3000
F 0 "#PWR08" H 2150 2750 50  0001 C CNN
F 1 "GND" H 2155 2827 50  0000 C CNN
F 2 "" H 2150 3000 50  0001 C CNN
F 3 "" H 2150 3000 50  0001 C CNN
	1    2150 3000
	1    0    0    -1  
$EndComp
Text GLabel 2650 2500 2    50   Input ~ 0
SDA
Text GLabel 2650 2300 2    50   Input ~ 0
SCL
$Comp
L power:GND #PWR09
U 1 1 60E0CB2C
P 3200 2250
F 0 "#PWR09" H 3200 2000 50  0001 C CNN
F 1 "GND" H 3205 2077 50  0000 C CNN
F 2 "" H 3200 2250 50  0001 C CNN
F 3 "" H 3200 2250 50  0001 C CNN
	1    3200 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 1800 1950 1700
Wire Wire Line
	1950 1700 2150 1700
Wire Wire Line
	3600 1700 3600 2700
Wire Wire Line
	3600 2700 2650 2700
Wire Wire Line
	2150 1800 2150 1700
Connection ~ 2150 1700
Wire Wire Line
	2150 1700 3600 1700
Text GLabel 1850 1700 0    50   Input ~ 0
3V3
Wire Wire Line
	1850 1700 1950 1700
Connection ~ 1950 1700
$Comp
L Sensor_Optical:APDS-9301 U3
U 1 1 60E1CEA4
P 8900 2450
F 0 "U3" H 9200 2950 50  0000 C CNN
F 1 "APDS-9301" H 9200 2850 50  0000 C CNN
F 2 "Opto-Devices:Broadcom_APDS-9301" H 8900 2900 50  0001 C CNN
F 3 "https://docs.broadcom.com/docs/AV02-2315EN" H 8600 2800 50  0001 C CNN
	1    8900 2450
	1    0    0    -1  
$EndComp
Text GLabel 8800 2000 0    50   Input ~ 0
3V3
$Comp
L power:GND #PWR07
U 1 1 60E1F634
P 8900 2850
F 0 "#PWR07" H 8900 2600 50  0001 C CNN
F 1 "GND" H 8905 2677 50  0000 C CNN
F 2 "" H 8900 2850 50  0001 C CNN
F 3 "" H 8900 2850 50  0001 C CNN
	1    8900 2850
	1    0    0    -1  
$EndComp
NoConn ~ 9300 2550
Text GLabel 9300 2350 2    50   Input ~ 0
SDA
Text GLabel 9300 2450 2    50   Input ~ 0
SCL
Wire Wire Line
	8900 2050 8900 2000
Wire Wire Line
	8900 2000 8800 2000
$Comp
L Connector:Screw_Terminal_01x04 J1
U 1 1 60E89144
P 1700 4550
F 0 "J1" H 1618 4867 50  0000 C CNN
F 1 "MCU Connection" H 1618 4776 50  0000 C CNN
F 2 "TerminalBlocks_Phoenix:TerminalBlock_Phoenix_MPT-2.54mm_4pol" H 1700 4550 50  0001 C CNN
F 3 "~" H 1700 4550 50  0001 C CNN
	1    1700 4550
	-1   0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Female J2
U 1 1 60E9548C
P 4850 1950
F 0 "J2" H 4878 1926 50  0000 L CNN
F 1 "Conn_01x04_Female" H 4878 1835 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x04_Pitch2.54mm" H 4850 1950 50  0001 C CNN
F 3 "~" H 4850 1950 50  0001 C CNN
	1    4850 1950
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male J3
U 1 1 60E95CD5
P 6950 1950
F 0 "J3" H 7058 2231 50  0000 C CNN
F 1 "Conn_01x04_Male" H 7058 2140 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 6950 1950 50  0001 C CNN
F 3 "~" H 6950 1950 50  0001 C CNN
	1    6950 1950
	1    0    0    -1  
$EndComp
Text GLabel 4650 1850 0    50   Input ~ 0
3V3
$Comp
L power:GND #PWR03
U 1 1 60EA548B
P 4600 2200
F 0 "#PWR03" H 4600 1950 50  0001 C CNN
F 1 "GND" H 4605 2027 50  0000 C CNN
F 2 "" H 4600 2200 50  0001 C CNN
F 3 "" H 4600 2200 50  0001 C CNN
	1    4600 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 2150 4600 2150
Wire Wire Line
	4600 2150 4600 2200
Text GLabel 4650 1950 0    50   Input ~ 0
SDA
Text GLabel 4650 2050 0    50   Input ~ 0
SCL
Text GLabel 7150 1950 2    50   Input ~ 0
SDA
Text GLabel 7150 2050 2    50   Input ~ 0
SCL
Text GLabel 7150 1850 2    50   Input ~ 0
3V3
$Comp
L power:GND #PWR04
U 1 1 60EAB2C1
P 7200 2200
F 0 "#PWR04" H 7200 1950 50  0001 C CNN
F 1 "GND" H 7205 2027 50  0000 C CNN
F 2 "" H 7200 2200 50  0001 C CNN
F 3 "" H 7200 2200 50  0001 C CNN
	1    7200 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 2150 7200 2150
Wire Wire Line
	7200 2150 7200 2200
Wire Notes Line
	5850 1350 5850 5550
Wire Notes Line
	5850 5550 1000 5550
Wire Notes Line
	1000 5550 1000 1350
Wire Notes Line
	1000 1350 5850 1350
Text Notes 3000 1200 0    100  ~ 20
Mainboard
Text Notes 7750 1200 0    100  ~ 20
Extension Board
Wire Notes Line
	6250 1350 6250 5550
Wire Notes Line
	6250 5550 10650 5550
Wire Notes Line
	10650 5550 10650 1350
Wire Notes Line
	10650 1350 6250 1350
$Comp
L Connector:TestPoint TP5
U 1 1 60E84641
P 8250 2550
F 0 "TP5" V 8350 2600 50  0000 C CNN
F 1 "TestPoint" V 8354 2622 50  0001 C CNN
F 2 "Measurement_Points:Measurement_Point_Square-SMD-Pad_Small" H 8450 2550 50  0001 C CNN
F 3 "~" H 8450 2550 50  0001 C CNN
	1    8250 2550
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP4
U 1 1 60E858B9
P 8250 2350
F 0 "TP4" V 8150 2400 50  0000 C CNN
F 1 "TestPoint" V 8354 2422 50  0001 C CNN
F 2 "Measurement_Points:Measurement_Point_Square-SMD-Pad_Small" H 8450 2350 50  0001 C CNN
F 3 "~" H 8450 2350 50  0001 C CNN
	1    8250 2350
	0    1    1    0   
$EndComp
Text GLabel 8250 2350 0    50   Input ~ 0
3V3
$Comp
L Connector:TestPoint TP6
U 1 1 60E82B8A
P 8500 2450
F 0 "TP6" V 8500 2700 50  0000 C CNN
F 1 "TestPoint" V 8604 2522 50  0001 C CNN
F 2 "Measurement_Points:Measurement_Point_Square-SMD-Pad_Small" H 8700 2450 50  0001 C CNN
F 3 "~" H 8700 2450 50  0001 C CNN
	1    8500 2450
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR010
U 1 1 60E86EA9
P 8100 2700
F 0 "#PWR010" H 8100 2450 50  0001 C CNN
F 1 "GND" H 8105 2527 50  0000 C CNN
F 2 "" H 8100 2700 50  0001 C CNN
F 3 "" H 8100 2700 50  0001 C CNN
	1    8100 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 2550 8100 2550
Wire Wire Line
	8100 2550 8100 2700
$Comp
L Connector:TestPoint TP1
U 1 1 60E8DEC8
P 2900 2100
F 0 "TP1" V 2900 2350 50  0000 C CNN
F 1 "TestPoint" V 3004 2172 50  0001 C CNN
F 2 "Measurement_Points:Measurement_Point_Square-SMD-Pad_Small" H 3100 2100 50  0001 C CNN
F 3 "~" H 3100 2100 50  0001 C CNN
	1    2900 2100
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP3
U 1 1 60E904D1
P 3150 2200
F 0 "TP3" V 3150 2450 50  0000 C CNN
F 1 "TestPoint" V 3254 2272 50  0001 C CNN
F 2 "Measurement_Points:Measurement_Point_Square-SMD-Pad_Small" H 3350 2200 50  0001 C CNN
F 3 "~" H 3350 2200 50  0001 C CNN
	1    3150 2200
	0    -1   -1   0   
$EndComp
$Comp
L Connector:TestPoint TP2
U 1 1 60E9093E
P 3150 2000
F 0 "TP2" V 3150 2250 50  0000 C CNN
F 1 "TestPoint" V 3254 2072 50  0001 C CNN
F 2 "Measurement_Points:Measurement_Point_Square-SMD-Pad_Small" H 3350 2000 50  0001 C CNN
F 3 "~" H 3350 2000 50  0001 C CNN
	1    3150 2000
	0    -1   -1   0   
$EndComp
Text GLabel 3150 2000 2    50   Input ~ 0
3V3
Wire Wire Line
	3150 2200 3200 2200
Wire Wire Line
	3200 2200 3200 2250
Wire Wire Line
	2900 2100 2650 2100
Text Notes 2450 1600 0    50   ~ 0
SDO to GND results in I2C slave address 0x76\nSDO to 3V3 results in I2C slave address 0x77
Wire Wire Line
	8900 3800 8900 3850
Wire Wire Line
	8700 3800 8900 3800
Text GLabel 8700 3800 0    50   Input ~ 0
3V3
Text GLabel 9500 4650 2    50   Input ~ 0
SDA
Text GLabel 9500 4250 2    50   Input ~ 0
SCL
$Comp
L power:GND #PWR06
U 1 1 60E050A4
P 8900 5050
F 0 "#PWR06" H 8900 4800 50  0001 C CNN
F 1 "GND" H 8905 4877 50  0000 C CNN
F 2 "" H 8900 5050 50  0001 C CNN
F 3 "" H 8900 5050 50  0001 C CNN
	1    8900 5050
	1    0    0    -1  
$EndComp
$Comp
L local:MLX90614 U2
U 1 1 60E4AD0A
P 8900 4450
F 0 "U2" H 8473 4496 50  0000 R CNN
F 1 "MLX90614" H 8473 4405 50  0000 R CNN
F 2 "local:MLX90614" H 8400 4850 50  0001 C CNN
F 3 "" H 8400 4850 50  0001 C CNN
	1    8900 4450
	1    0    0    -1  
$EndComp
Text Notes 8200 1700 0    50   ~ 0
ADR_SEL to GND results in I2C slave address 0x29\nADR_SEL floating results in I2C slave address 0x39\nADR_SEL to 3V3 results in I2C slave address 0x49
Text Notes 8450 3550 0    50   ~ 0
Fixed I2C slave address 0x5B
$EndSCHEMATC
