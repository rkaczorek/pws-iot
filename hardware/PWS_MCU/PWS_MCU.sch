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
L Connector:RJ12 J2
U 1 1 60DE5817
P 1650 2000
F 0 "J2" H 1700 2450 50  0000 R CNN
F 1 "RJ12 (RAIN SENSOR)" H 2050 1600 50  0000 R CNN
F 2 "local:molex_6p6c" V 1650 2025 50  0001 C CNN
F 3 "~" V 1650 2025 50  0001 C CNN
	1    1650 2000
	1    0    0    1   
$EndComp
$Comp
L Connector:RJ12 J3
U 1 1 60DE619A
P 1650 4600
F 0 "J3" H 1700 5050 50  0000 R CNN
F 1 "RJ12 (WIND SENSOR)" H 2050 4200 50  0000 R CNN
F 2 "local:molex_6p6c" V 1650 4625 50  0001 C CNN
F 3 "~" V 1650 4625 50  0001 C CNN
	1    1650 4600
	1    0    0    1   
$EndComp
$Comp
L Device:R_Small R5
U 1 1 60DEC2C3
P 2300 2000
F 0 "R5" V 2400 2000 39  0000 C CNN
F 1 "1K" V 2300 2000 39  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 2300 2000 50  0001 C CNN
F 3 "~" H 2300 2000 50  0001 C CNN
	1    2300 2000
	0    -1   -1   0   
$EndComp
NoConn ~ 2050 1800
NoConn ~ 2050 1900
NoConn ~ 2050 2200
NoConn ~ 2050 2300
$Comp
L power:GND #PWR0101
U 1 1 60DF20A8
P 2200 2500
F 0 "#PWR0101" H 2200 2250 50  0001 C CNN
F 1 "GND" H 2205 2327 50  0000 C CNN
F 2 "" H 2200 2500 50  0001 C CNN
F 3 "" H 2200 2500 50  0001 C CNN
	1    2200 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 2100 2200 2100
Wire Wire Line
	2200 2100 2200 2150
Wire Wire Line
	2200 2350 2200 2500
Wire Wire Line
	2050 2000 2200 2000
Wire Wire Line
	2400 2000 2500 2000
$Comp
L Device:D_Small D1
U 1 1 60DF9F63
P 2500 1800
F 0 "D1" H 2350 1750 39  0000 L CNN
F 1 "BAS16" H 2250 1850 39  0000 L CNN
F 2 "Diodes_SMD:D_SOD-123" V 2500 1800 50  0001 C CNN
F 3 "~" V 2500 1800 50  0001 C CNN
	1    2500 1800
	0    1    1    0   
$EndComp
$Comp
L Device:D_Small D2
U 1 1 60DFC486
P 2500 2250
F 0 "D2" H 2600 2200 39  0000 L CNN
F 1 "BAS16" H 2550 2300 39  0000 L CNN
F 2 "Diodes_SMD:D_SOD-123" V 2500 2250 50  0001 C CNN
F 3 "~" V 2500 2250 50  0001 C CNN
	1    2500 2250
	0    1    1    0   
$EndComp
Wire Wire Line
	2500 1900 2500 2000
Connection ~ 2500 2000
Wire Wire Line
	2500 2000 2650 2000
Wire Wire Line
	2500 2150 2500 2000
$Comp
L power:GND #PWR0102
U 1 1 60DFCE73
P 2500 2500
F 0 "#PWR0102" H 2500 2250 50  0001 C CNN
F 1 "GND" H 2505 2327 50  0000 C CNN
F 2 "" H 2500 2500 50  0001 C CNN
F 3 "" H 2500 2500 50  0001 C CNN
	1    2500 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 2350 2500 2500
Text GLabel 2650 2000 2    50   Input ~ 0
RAIN
Wire Wire Line
	2500 1550 2500 1700
NoConn ~ 2050 4400
NoConn ~ 2050 4900
Wire Wire Line
	2050 4800 2150 4800
Wire Wire Line
	2150 4800 2150 4900
Wire Wire Line
	2050 4700 2350 4700
Wire Wire Line
	2350 4700 2350 4900
$Comp
L power:GND #PWR0104
U 1 1 60E07878
P 2150 5200
F 0 "#PWR0104" H 2150 4950 50  0001 C CNN
F 1 "GND" H 2155 5027 50  0000 C CNN
F 2 "" H 2150 5200 50  0001 C CNN
F 3 "" H 2150 5200 50  0001 C CNN
	1    2150 5200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 60E07CDF
P 2350 5200
F 0 "#PWR0105" H 2350 4950 50  0001 C CNN
F 1 "GND" H 2355 5027 50  0000 C CNN
F 2 "" H 2350 5200 50  0001 C CNN
F 3 "" H 2350 5200 50  0001 C CNN
	1    2350 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 5100 2150 5200
Wire Wire Line
	2350 5100 2350 5200
$Comp
L Device:R_Small R6
U 1 1 60E06DCB
P 2350 5000
F 0 "R6" V 2250 4950 39  0000 L CNN
F 1 "1K" V 2350 4950 39  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 2350 5000 50  0001 C CNN
F 3 "~" H 2350 5000 50  0001 C CNN
	1    2350 5000
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R1
U 1 1 60E0692A
P 2150 5000
F 0 "R1" V 2050 4950 39  0000 L CNN
F 1 "1K" V 2150 4950 39  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 2150 5000 50  0001 C CNN
F 3 "~" H 2150 5000 50  0001 C CNN
	1    2150 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 4500 2150 4500
Wire Wire Line
	2050 4600 2150 4600
Wire Wire Line
	3150 4500 3050 4500
Wire Wire Line
	3150 4600 2550 4600
$Comp
L Device:R_Small R7
U 1 1 60E160F1
P 3050 4350
F 0 "R7" H 3100 4350 39  0000 L CNN
F 1 "4K7" V 3050 4300 39  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 3050 4350 50  0001 C CNN
F 3 "~" H 3050 4350 50  0001 C CNN
	1    3050 4350
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Small D3
U 1 1 60E205AD
P 2550 4350
F 0 "D3" V 2450 4250 39  0000 L CNN
F 1 "BAS16" H 2300 4400 39  0000 L CNN
F 2 "Diodes_SMD:D_SOD-123" V 2550 4350 50  0001 C CNN
F 3 "~" V 2550 4350 50  0001 C CNN
	1    2550 4350
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 60E24867
P 2550 5200
F 0 "#PWR0106" H 2550 4950 50  0001 C CNN
F 1 "GND" H 2555 5027 50  0000 C CNN
F 2 "" H 2550 5200 50  0001 C CNN
F 3 "" H 2550 5200 50  0001 C CNN
	1    2550 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 5200 2550 4900
Wire Wire Line
	2550 4700 2550 4600
Connection ~ 2550 4600
Wire Wire Line
	2550 4600 2350 4600
Wire Wire Line
	2550 4600 2550 4450
$Comp
L Device:D_Small D5
U 1 1 60E3DABB
P 2800 4350
F 0 "D5" V 2700 4250 39  0000 L CNN
F 1 "BAS16" H 2550 4400 39  0000 L CNN
F 2 "Diodes_SMD:D_SOD-123" V 2800 4350 50  0001 C CNN
F 3 "~" V 2800 4350 50  0001 C CNN
	1    2800 4350
	0    1    1    0   
$EndComp
$Comp
L Device:D_Small D6
U 1 1 60E3E058
P 2800 4800
F 0 "D6" H 2900 4750 39  0000 L CNN
F 1 "BAS16" H 2850 4850 39  0000 L CNN
F 2 "Diodes_SMD:D_SOD-123" V 2800 4800 50  0001 C CNN
F 3 "~" V 2800 4800 50  0001 C CNN
	1    2800 4800
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 60E3E4F8
P 2800 5200
F 0 "#PWR0108" H 2800 4950 50  0001 C CNN
F 1 "GND" H 2805 5027 50  0000 C CNN
F 2 "" H 2800 5200 50  0001 C CNN
F 3 "" H 2800 5200 50  0001 C CNN
	1    2800 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 5200 2800 4900
Wire Wire Line
	2800 4700 2800 4500
Connection ~ 2800 4500
Wire Wire Line
	2800 4500 2350 4500
Wire Wire Line
	2800 4450 2800 4500
Wire Wire Line
	3050 4450 3050 4500
Connection ~ 3050 4500
Wire Wire Line
	3050 4500 2800 4500
Text GLabel 3150 4500 2    50   Input ~ 0
WDIR
Text GLabel 3150 4600 2    50   Input ~ 0
WSPEED
$Comp
L Connector:RJ12 J1
U 1 1 60DE5BC0
P 8800 2050
F 0 "J1" H 8850 2500 50  0000 R CNN
F 1 "RJ12 (I2C SENSORS)" H 9200 1650 50  0000 R CNN
F 2 "local:molex_6p6c" V 8800 2075 50  0001 C CNN
F 3 "~" V 8800 2075 50  0001 C CNN
	1    8800 2050
	1    0    0    1   
$EndComp
NoConn ~ 9200 1850
NoConn ~ 9200 2350
$Comp
L power:GND #PWR0111
U 1 1 60DF3557
P 9300 2500
F 0 "#PWR0111" H 9300 2250 50  0001 C CNN
F 1 "GND" H 9305 2327 50  0000 C CNN
F 2 "" H 9300 2500 50  0001 C CNN
F 3 "" H 9300 2500 50  0001 C CNN
	1    9300 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9200 2250 9300 2250
Wire Wire Line
	9300 2250 9300 2500
Text GLabel 9200 2050 2    50   Input ~ 0
SDA
Text GLabel 9200 2150 2    50   Input ~ 0
SCL
$Comp
L local:Arduino-NANO-33 J5
U 1 1 60E00E33
P 5700 3300
F 0 "J5" H 5700 4365 50  0000 C CNN
F 1 "Arduino-NANO-33-IoT" H 5700 4274 50  0000 C CNN
F 2 "local:NANO_33_Footprint_SMD_Castell" H 5700 4273 50  0001 C CNN
F 3 "~" H 5375 3200 50  0001 C CNN
	1    5700 3300
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Small D4
U 1 1 60E1DC73
P 2550 4800
F 0 "D4" H 2650 4750 39  0000 L CNN
F 1 "BAS16" H 2600 4850 39  0000 L CNN
F 2 "Diodes_SMD:D_SOD-123" V 2550 4800 50  0001 C CNN
F 3 "~" V 2550 4800 50  0001 C CNN
	1    2550 4800
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R4
U 1 1 60E0C0DE
P 2250 4600
F 0 "R4" V 2200 4450 39  0000 C CNN
F 1 "1K" V 2250 4600 39  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 2250 4600 50  0001 C CNN
F 3 "~" H 2250 4600 50  0001 C CNN
	1    2250 4600
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R3
U 1 1 60E089F3
P 2250 4500
F 0 "R3" V 2300 4350 39  0000 C CNN
F 1 "1K" V 2250 4500 39  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 2250 4500 50  0001 C CNN
F 3 "~" H 2250 4500 50  0001 C CNN
	1    2250 4500
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R2
U 1 1 60DF1520
P 2200 2250
F 0 "R2" V 2300 2200 39  0000 L CNN
F 1 "1K" V 2200 2200 39  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 2200 2250 50  0001 C CNN
F 3 "~" H 2200 2250 50  0001 C CNN
	1    2200 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 4150 4900 3900
Wire Wire Line
	4900 3900 5200 3900
Wire Wire Line
	6500 4150 6500 3700
Wire Wire Line
	6500 3700 6200 3700
$Comp
L power:GND #PWR01
U 1 1 60E28DA5
P 4900 4150
F 0 "#PWR01" H 4900 3900 50  0001 C CNN
F 1 "GND" H 4905 3977 50  0000 C CNN
F 2 "" H 4900 4150 50  0001 C CNN
F 3 "" H 4900 4150 50  0001 C CNN
	1    4900 4150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 60E29222
P 6500 4150
F 0 "#PWR03" H 6500 3900 50  0001 C CNN
F 1 "GND" H 6505 3977 50  0000 C CNN
F 2 "" H 6500 4150 50  0001 C CNN
F 3 "" H 6500 4150 50  0001 C CNN
	1    6500 4150
	1    0    0    -1  
$EndComp
NoConn ~ 5200 3700
NoConn ~ 5200 3800
NoConn ~ 6200 3800
NoConn ~ 5200 2600
Text GLabel 5200 2700 0    50   Input ~ 0
3V3
Text GLabel 5200 3300 0    50   Input ~ 0
SDA
Text GLabel 5200 3400 0    50   Input ~ 0
SCL
Text GLabel 9200 1950 2    50   Input ~ 0
3V3
Text GLabel 2500 1550 1    50   Input ~ 0
3V3
Wire Wire Line
	2550 4250 2550 4050
Wire Wire Line
	2550 4050 2800 4050
Wire Wire Line
	3050 4050 3050 4250
Wire Wire Line
	2800 4250 2800 4050
Connection ~ 2800 4050
Wire Wire Line
	2800 4050 3050 4050
Wire Wire Line
	2800 3950 2800 4050
Text GLabel 2800 3950 1    50   Input ~ 0
3V3
Text GLabel 5200 4000 0    50   Input ~ 0
VIN
Text GLabel 6200 3900 2    50   Input ~ 0
RX
Text GLabel 6200 4000 2    50   Input ~ 0
TX
NoConn ~ 5200 2800
NoConn ~ 5200 3600
NoConn ~ 5200 3500
NoConn ~ 5200 3200
NoConn ~ 5200 3100
NoConn ~ 5200 3000
Text GLabel 5200 2900 0    50   Input ~ 0
WDIR
Text GLabel 5650 5400 2    50   Input ~ 0
SDA
Text GLabel 5650 5600 2    50   Input ~ 0
SCL
$Comp
L Device:R_Small R8
U 1 1 60E0BEDA
P 5550 5400
F 0 "R8" V 5450 5350 39  0000 L CNN
F 1 "4K7" V 5550 5350 39  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 5550 5400 50  0001 C CNN
F 3 "~" H 5550 5400 50  0001 C CNN
	1    5550 5400
	0    1    1    0   
$EndComp
Text GLabel 5450 5400 0    50   Input ~ 0
3V3
$Comp
L Device:R_Small R9
U 1 1 60E18BEA
P 5550 5600
F 0 "R9" V 5450 5550 39  0000 L CNN
F 1 "4K7" V 5550 5550 39  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 5550 5600 50  0001 C CNN
F 3 "~" H 5550 5600 50  0001 C CNN
	1    5550 5600
	0    1    1    0   
$EndComp
Text GLabel 5450 5600 0    50   Input ~ 0
3V3
Text GLabel 6200 2800 2    50   Input ~ 0
WSPEED
Text GLabel 6200 2700 2    50   Input ~ 0
RAIN
NoConn ~ 6200 2900
NoConn ~ 6200 3000
NoConn ~ 6200 3100
NoConn ~ 6200 3200
NoConn ~ 6200 3300
NoConn ~ 6200 3400
NoConn ~ 6200 3500
NoConn ~ 6200 3600
Text GLabel 8900 4650 2    50   Input ~ 0
TX
Text GLabel 8900 4750 2    50   Input ~ 0
RX
$Comp
L power:GND #PWR02
U 1 1 60E53879
P 8950 4900
F 0 "#PWR02" H 8950 4650 50  0001 C CNN
F 1 "GND" H 8955 4727 50  0000 C CNN
F 2 "" H 8950 4900 50  0001 C CNN
F 3 "" H 8950 4900 50  0001 C CNN
	1    8950 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 4850 8950 4850
Wire Wire Line
	8950 4850 8950 4900
Text Notes 6050 2050 2    50   ~ 0
Main Computing Unit
Text Notes 2050 1150 2    50   ~ 0
Rain Sensor Connector
Text Notes 2050 3650 2    50   ~ 0
Wind Sensor Connector
Text Notes 5950 5150 2    50   ~ 0
I2C Pull-Up Resistors
Text Notes 9200 1350 2    50   ~ 0
I2C Sensors Connector
Text Notes 9300 4450 2    50   ~ 0
Serial Debug Connector
Text Notes 6600 1050 2    100  ~ 20
Personal Weather Station
Text Notes 1850 6300 2    50   ~ 0
Power Connector
Text GLabel 1550 6750 0    50   Input ~ 0
VIN
Wire Wire Line
	1500 6850 1500 6900
Wire Wire Line
	1550 6850 1500 6850
$Comp
L power:GND #PWR011
U 1 1 60E41377
P 1500 6900
F 0 "#PWR011" H 1500 6650 50  0001 C CNN
F 1 "GND" H 1505 6727 50  0000 C CNN
F 2 "" H 1500 6900 50  0001 C CNN
F 3 "" H 1500 6900 50  0001 C CNN
	1    1500 6900
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J7
U 1 1 60E3FE23
P 1750 6750
F 0 "J7" H 1830 6742 50  0000 L CNN
F 1 "Screw_Terminal_01x02" H 1830 6651 50  0000 L CNN
F 2 "TerminalBlocks_Phoenix:TerminalBlock_Phoenix_MPT-2.54mm_2pol" H 1750 6750 50  0001 C CNN
F 3 "~" H 1750 6750 50  0001 C CNN
	1    1750 6750
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP1
U 1 1 60E629F8
P 8900 4650
F 0 "TP1" V 8900 4900 50  0000 C CNN
F 1 "TestPoint" V 9000 5000 50  0001 C CNN
F 2 "Measurement_Points:Measurement_Point_Square-SMD-Pad_Small" H 9100 4650 50  0001 C CNN
F 3 "~" H 9100 4650 50  0001 C CNN
	1    8900 4650
	0    -1   -1   0   
$EndComp
$Comp
L Connector:TestPoint TP2
U 1 1 60E63688
P 8900 4750
F 0 "TP2" V 8900 5000 50  0000 C CNN
F 1 "TestPoint" V 8900 5100 50  0001 C CNN
F 2 "Measurement_Points:Measurement_Point_Square-SMD-Pad_Small" H 9100 4750 50  0001 C CNN
F 3 "~" H 9100 4750 50  0001 C CNN
	1    8900 4750
	0    -1   -1   0   
$EndComp
$Comp
L Connector:TestPoint TP3
U 1 1 60E63890
P 8900 4850
F 0 "TP3" V 8900 5100 50  0000 C CNN
F 1 "TestPoint" V 8800 5200 50  0001 C CNN
F 2 "Measurement_Points:Measurement_Point_Square-SMD-Pad_Small" H 9100 4850 50  0001 C CNN
F 3 "~" H 9100 4850 50  0001 C CNN
	1    8900 4850
	0    -1   -1   0   
$EndComp
NoConn ~ 6200 2600
$EndSCHEMATC
