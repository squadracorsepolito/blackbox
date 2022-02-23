EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Interface_CAN_LIN:MCP2562-E-P U?
U 1 1 620A4E01
P 4450 2050
F 0 "U?" H 4000 3000 50  0001 C CNN
F 1 "MCP2562-E-P" H 4450 2050 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 4450 1550 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/25167A.pdf" H 4450 2050 50  0001 C CNN
	1    4450 2050
	-1   0    0    -1  
$EndComp
$Comp
L Interface_CAN_LIN:MCP2562-E-P U?
U 1 1 620A59FB
P 4450 3350
F 0 "U?" H 4450 3931 50  0001 C CNN
F 1 "MCP2562-E-P" H 4450 3350 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 4450 2850 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/25167A.pdf" H 4450 3350 50  0001 C CNN
	1    4450 3350
	-1   0    0    -1  
$EndComp
$Comp
L Device:Buzzer BZ?
U 1 1 620A5C31
P 2000 4600
F 0 "BZ?" H 2152 4629 50  0000 L CNN
F 1 "Buzzer" H 2152 4538 50  0000 L CNN
F 2 "" V 1975 4700 50  0001 C CNN
F 3 "~" V 1975 4700 50  0001 C CNN
	1    2000 4600
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_SPDT SW?
U 1 1 620A7C3B
P 3750 1000
F 0 "SW?" H 3750 1285 50  0001 C CNN
F 1 "SW_SPDT" H 3750 1194 50  0000 C CNN
F 2 "" H 3750 1000 50  0001 C CNN
F 3 "~" H 3750 1000 50  0001 C CNN
	1    3750 1000
	-1   0    0    -1  
$EndComp
$Comp
L Telemetry:Longan_Nano U?
U 1 1 620AC438
P 6850 2300
F 0 "U?" H 6850 3365 50  0000 C CNN
F 1 "Longan_Nano" H 6850 3274 50  0000 C CNN
F 2 "" H 6750 2700 50  0001 C CNN
F 3 "" H 6750 2700 50  0001 C CNN
	1    6850 2300
	1    0    0    -1  
$EndComp
$Comp
L Device:Battery_Cell BT?
U 1 1 620AE22F
P 900 1050
F 0 "BT?" H 1018 1146 50  0000 L CNN
F 1 "Battery_Cell" H 1018 1055 50  0000 L CNN
F 2 "" V 900 1110 50  0001 C CNN
F 3 "~" V 900 1110 50  0001 C CNN
	1    900  1050
	1    0    0    -1  
$EndComp
$Comp
L Telemetry:DC-DC U?
U 1 1 620AFB6B
P 1850 1000
F 0 "U?" H 1908 1325 50  0000 C CNN
F 1 "DC-DC" H 1908 1234 50  0000 C CNN
F 2 "" H 2000 850 50  0001 C CNN
F 3 "" H 2000 850 50  0001 C CNN
	1    1850 1000
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR?
U 1 1 620D073C
P 900 850
F 0 "#PWR?" H 900 700 50  0001 C CNN
F 1 "+BATT" H 915 1023 50  0000 C CNN
F 2 "" H 900 850 50  0001 C CNN
F 3 "" H 900 850 50  0001 C CNN
	1    900  850 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 620D0E27
P 900 1150
F 0 "#PWR?" H 900 900 50  0001 C CNN
F 1 "GND" H 905 977 50  0000 C CNN
F 2 "" H 900 1150 50  0001 C CNN
F 3 "" H 900 1150 50  0001 C CNN
	1    900  1150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 620D28E0
P 6300 3100
F 0 "#PWR?" H 6300 2850 50  0001 C CNN
F 1 "GND" H 6305 2927 50  0000 C CNN
F 2 "" H 6300 3100 50  0001 C CNN
F 3 "" H 6300 3100 50  0001 C CNN
	1    6300 3100
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR?
U 1 1 620D334F
P 7750 3100
F 0 "#PWR?" H 7750 2950 50  0001 C CNN
F 1 "+BATT" H 7765 3273 50  0000 C CNN
F 2 "" H 7750 3100 50  0001 C CNN
F 3 "" H 7750 3100 50  0001 C CNN
	1    7750 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 3100 7750 3100
$Comp
L Transistor_BJT:2N2219 Q?
U 1 1 620D4CCF
P 1800 5100
F 0 "Q?" H 1990 5146 50  0000 L CNN
F 1 "2N2219" H 1990 5055 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-39-3" H 2000 5025 50  0001 L CIN
F 3 "http://www.onsemi.com/pub_link/Collateral/2N2219-D.PDF" H 1800 5100 50  0001 L CNN
	1    1800 5100
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR?
U 1 1 620D590C
P 2300 900
F 0 "#PWR?" H 2300 750 50  0001 C CNN
F 1 "+12V" H 2315 1073 50  0000 C CNN
F 2 "" H 2300 900 50  0001 C CNN
F 3 "" H 2300 900 50  0001 C CNN
	1    2300 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 900  2300 900 
$Comp
L power:+5V #PWR?
U 1 1 620D61BA
P 2300 1100
F 0 "#PWR?" H 2300 950 50  0001 C CNN
F 1 "+5V" H 2315 1273 50  0000 C CNN
F 2 "" H 2300 1100 50  0001 C CNN
F 3 "" H 2300 1100 50  0001 C CNN
	1    2300 1100
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_02x04_Top_Bottom J?
U 1 1 620D6E31
P 1400 2650
F 0 "J?" H 1450 2967 50  0001 C CNN
F 1 "CAN" H 1450 2875 50  0000 C CNN
F 2 "Connector_Molex:Molex_Nano-Fit_105310-xx08_2x04_P2.50mm_Vertical" H 1400 2650 50  0001 C CNN
F 3 "~" H 1400 2650 50  0001 C CNN
	1    1400 2650
	-1   0    0    -1  
$EndComp
$Comp
L power:+12V #PWR?
U 1 1 620D7868
P 1750 2500
F 0 "#PWR?" H 1750 2350 50  0001 C CNN
F 1 "+12V" H 1765 2673 50  0000 C CNN
F 2 "" H 1750 2500 50  0001 C CNN
F 3 "" H 1750 2500 50  0001 C CNN
	1    1750 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 2550 1750 2550
Wire Wire Line
	1750 2550 1750 2500
Wire Wire Line
	2100 1100 2300 1100
$Comp
L power:GND #PWR?
U 1 1 620D7DE5
P 2300 1000
F 0 "#PWR?" H 2300 750 50  0001 C CNN
F 1 "GND" V 2305 872 50  0000 R CNN
F 2 "" H 2300 1000 50  0001 C CNN
F 3 "" H 2300 1000 50  0001 C CNN
	1    2300 1000
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 620D8AAB
P 1750 2900
F 0 "#PWR?" H 1750 2650 50  0001 C CNN
F 1 "GND" H 1755 2727 50  0000 C CNN
F 2 "" H 1750 2900 50  0001 C CNN
F 3 "" H 1750 2900 50  0001 C CNN
	1    1750 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 2850 1750 2850
Wire Wire Line
	1750 2850 1750 2900
$Comp
L power:+5V #PWR?
U 1 1 620DA590
P 4450 1650
F 0 "#PWR?" H 4450 1500 50  0001 C CNN
F 1 "+5V" H 4450 1800 50  0000 C CNN
F 2 "" H 4450 1650 50  0001 C CNN
F 3 "" H 4450 1650 50  0001 C CNN
	1    4450 1650
	-1   0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 620DB174
P 4950 2150
F 0 "#PWR?" H 4950 2000 50  0001 C CNN
F 1 "+3.3V" V 4965 2278 50  0000 L CNN
F 2 "" H 4950 2150 50  0001 C CNN
F 3 "" H 4950 2150 50  0001 C CNN
	1    4950 2150
	0    1    -1   0   
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 620DC035
P 4950 3450
F 0 "#PWR?" H 4950 3300 50  0001 C CNN
F 1 "+3.3V" V 4965 3578 50  0000 L CNN
F 2 "" H 4950 3450 50  0001 C CNN
F 3 "" H 4950 3450 50  0001 C CNN
	1    4950 3450
	0    1    -1   0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 620DC916
P 4450 2950
F 0 "#PWR?" H 4450 2800 50  0001 C CNN
F 1 "+5V" H 4450 3100 50  0000 C CNN
F 2 "" H 4450 2950 50  0001 C CNN
F 3 "" H 4450 2950 50  0001 C CNN
	1    4450 2950
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 620DCC56
P 4450 2450
F 0 "#PWR?" H 4450 2200 50  0001 C CNN
F 1 "GND" H 4455 2277 50  0000 C CNN
F 2 "" H 4450 2450 50  0001 C CNN
F 3 "" H 4450 2450 50  0001 C CNN
	1    4450 2450
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 620DD9D8
P 4450 3750
F 0 "#PWR?" H 4450 3500 50  0001 C CNN
F 1 "GND" H 4455 3577 50  0000 C CNN
F 2 "" H 4450 3750 50  0001 C CNN
F 3 "" H 4450 3750 50  0001 C CNN
	1    4450 3750
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 620DDB98
P 4950 3550
F 0 "#PWR?" H 4950 3300 50  0001 C CNN
F 1 "GND" H 4955 3377 50  0000 C CNN
F 2 "" H 4950 3550 50  0001 C CNN
F 3 "" H 4950 3550 50  0001 C CNN
	1    4950 3550
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 620DDE02
P 4950 2250
F 0 "#PWR?" H 4950 2000 50  0001 C CNN
F 1 "GND" H 4955 2077 50  0000 C CNN
F 2 "" H 4950 2250 50  0001 C CNN
F 3 "" H 4950 2250 50  0001 C CNN
	1    4950 2250
	-1   0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 620E1CAC
P 3700 2300
F 0 "R?" H 3770 2346 50  0001 L CNN
F 1 "120" H 3630 2300 50  0000 R CNN
F 2 "" V 3630 2300 50  0001 C CNN
F 3 "~" H 3700 2300 50  0001 C CNN
	1    3700 2300
	1    0    0    1   
$EndComp
Wire Wire Line
	3100 2550 3850 2550
Wire Wire Line
	3850 2550 3850 1950
Wire Wire Line
	3850 1950 3950 1950
Wire Wire Line
	3100 2850 3850 2850
$Comp
L Connector_Generic:Conn_01x06 J?
U 1 1 620E8102
P 2900 2650
F 0 "J?" H 2980 2642 50  0000 L CNN
F 1 "Conn_01x06" H 2980 2551 50  0000 L CNN
F 2 "" H 2900 2650 50  0001 C CNN
F 3 "~" H 2900 2650 50  0001 C CNN
	1    2900 2650
	-1   0    0    -1  
$EndComp
Connection ~ 3850 1950
Wire Wire Line
	3950 2150 3700 2150
Wire Wire Line
	3700 2450 3100 2450
Connection ~ 3700 2150
Wire Wire Line
	3100 2950 3700 2950
$Comp
L Device:R R?
U 1 1 620E2408
P 3700 3100
F 0 "R?" H 3770 3146 50  0001 L CNN
F 1 "120" H 3630 3100 50  0000 R CNN
F 2 "" V 3630 3100 50  0001 C CNN
F 3 "~" H 3700 3100 50  0001 C CNN
	1    3700 3100
	1    0    0    1   
$EndComp
Wire Wire Line
	3950 3250 3700 3250
Wire Wire Line
	3950 3450 3850 3450
Connection ~ 3700 3250
Connection ~ 3850 3450
Wire Wire Line
	3850 3450 1950 3450
Wire Wire Line
	3850 2850 3850 3450
$Comp
L power:GND #PWR?
U 1 1 620F9950
P 950 2900
F 0 "#PWR?" H 950 2650 50  0001 C CNN
F 1 "GND" H 955 2727 50  0000 C CNN
F 2 "" H 950 2900 50  0001 C CNN
F 3 "" H 950 2900 50  0001 C CNN
	1    950  2900
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1100 2850 950  2850
Wire Wire Line
	950  2850 950  2900
$Comp
L power:+12V #PWR?
U 1 1 620FA2BE
P 950 2500
F 0 "#PWR?" H 950 2350 50  0001 C CNN
F 1 "+12V" H 965 2673 50  0000 C CNN
F 2 "" H 950 2500 50  0001 C CNN
F 3 "" H 950 2500 50  0001 C CNN
	1    950  2500
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1100 2550 950  2550
Wire Wire Line
	950  2550 950  2500
Wire Wire Line
	800  1950 800  2650
Wire Wire Line
	800  2650 1100 2650
Wire Wire Line
	800  1950 3850 1950
Wire Wire Line
	700  2150 700  2750
Wire Wire Line
	700  2750 1100 2750
Wire Wire Line
	700  2150 3700 2150
Wire Wire Line
	1600 2750 1950 2750
Wire Wire Line
	1950 2750 1950 3450
Wire Wire Line
	3700 3250 2150 3250
Wire Wire Line
	1600 2650 2150 2650
Wire Wire Line
	2150 2650 2150 3250
Wire Wire Line
	2300 1000 2100 1000
$Comp
L Device:R R?
U 1 1 62106731
P 1450 5100
F 0 "R?" H 1520 5146 50  0001 L CNN
F 1 "1k" V 1335 5100 50  0000 C CNN
F 2 "" V 1380 5100 50  0001 C CNN
F 3 "~" H 1450 5100 50  0001 C CNN
	1    1450 5100
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 6210B4E3
P 1900 4350
F 0 "#PWR?" H 1900 4200 50  0001 C CNN
F 1 "+5V" H 1915 4523 50  0000 C CNN
F 2 "" H 1900 4350 50  0001 C CNN
F 3 "" H 1900 4350 50  0001 C CNN
	1    1900 4350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6210C4A3
P 1900 5300
F 0 "#PWR?" H 1900 5050 50  0001 C CNN
F 1 "GND" H 1905 5127 50  0000 C CNN
F 2 "" H 1900 5300 50  0001 C CNN
F 3 "" H 1900 5300 50  0001 C CNN
	1    1900 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 4350 1900 4500
Wire Wire Line
	1900 4700 1900 4900
Text Label 950  5100 0    50   ~ 0
Buzzer
Wire Wire Line
	1300 5100 850  5100
Wire Wire Line
	6300 2900 5900 2900
Text Label 5900 2900 0    50   ~ 0
Buzzer
Wire Wire Line
	6300 2200 5900 2200
Wire Wire Line
	6300 2300 5900 2300
Text Label 5900 2200 0    50   ~ 0
CAN0_RX
Text Label 5900 2300 0    50   ~ 0
CAN0_TX
Wire Wire Line
	6300 2500 5900 2500
Wire Wire Line
	6300 2600 5900 2600
Text Label 5900 2500 0    50   ~ 0
CAN1_RX
Text Label 5900 2600 0    50   ~ 0
CAN1_TX
Wire Wire Line
	6300 2800 5900 2800
Text Label 5900 2800 0    50   ~ 0
SW
Wire Wire Line
	3950 1000 4350 1000
Text Label 4350 1000 2    50   ~ 0
SW
$Comp
L power:GND #PWR?
U 1 1 62118A38
P 3450 1100
F 0 "#PWR?" H 3450 850 50  0001 C CNN
F 1 "GND" H 3455 927 50  0000 C CNN
F 2 "" H 3450 1100 50  0001 C CNN
F 3 "" H 3450 1100 50  0001 C CNN
	1    3450 1100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3450 1100 3550 1100
$Comp
L power:+3.3V #PWR?
U 1 1 6211AB1F
P 3450 900
F 0 "#PWR?" H 3450 750 50  0001 C CNN
F 1 "+3.3V" H 3465 1073 50  0000 C CNN
F 2 "" H 3450 900 50  0001 C CNN
F 3 "" H 3450 900 50  0001 C CNN
	1    3450 900 
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3450 900  3550 900 
$Comp
L power:+5V #PWR?
U 1 1 6210053A
P 6300 1500
F 0 "#PWR?" H 6300 1350 50  0001 C CNN
F 1 "+5V" H 6300 1650 50  0000 C CNN
F 2 "" H 6300 1500 50  0001 C CNN
F 3 "" H 6300 1500 50  0001 C CNN
	1    6300 1500
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4950 1850 5350 1850
Wire Wire Line
	4950 1950 5350 1950
Text Label 5350 1950 2    50   ~ 0
CAN0_RX
Text Label 5350 1850 2    50   ~ 0
CAN0_TX
Wire Wire Line
	4950 3150 5350 3150
Wire Wire Line
	4950 3250 5350 3250
Text Label 5350 3250 2    50   ~ 0
CAN1_RX
Text Label 5350 3150 2    50   ~ 0
CAN1_TX
NoConn ~ 6300 2700
NoConn ~ 6300 2400
NoConn ~ 6300 2100
NoConn ~ 6300 2000
NoConn ~ 6300 1900
NoConn ~ 6300 1800
NoConn ~ 6300 1700
NoConn ~ 6300 1600
NoConn ~ 7400 1500
NoConn ~ 7400 1600
NoConn ~ 7400 1700
NoConn ~ 7400 1800
NoConn ~ 7400 1900
NoConn ~ 7400 2000
NoConn ~ 7400 2100
NoConn ~ 7400 2200
NoConn ~ 7400 2300
NoConn ~ 7400 2400
NoConn ~ 7400 2500
NoConn ~ 7400 2600
NoConn ~ 7400 2700
NoConn ~ 7400 2800
NoConn ~ 7400 2900
NoConn ~ 7400 3000
$EndSCHEMATC
