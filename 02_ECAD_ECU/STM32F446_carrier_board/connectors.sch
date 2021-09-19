EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 3
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
L max_KiCAD_lib:FX10A-80P_8-SV_71_ J2
U 1 1 6145B3E2
P 3100 3050
F 0 "J2" H 3600 3315 50  0000 C CNN
F 1 "FX10A-80P_8-SV_71_" H 3600 3224 50  0000 C CNN
F 2 "max_KiCAD_lib:FX10A80P8SV71" H 3950 3150 50  0001 L CNN
F 3 "https://www.hirose.com/en/product/document?clcode=CL0570-0001-5-71&productname=FX10A-80P%2f8-SV(71)&series=FX10&documenttype=2DDrawing&lang=en&documentid=0000909743" H 3950 3050 50  0001 L CNN
F 4 "Board to Board & Mezzanine Connectors HDR 80POS W/POST SMT" H 3950 2950 50  0001 L CNN "Description"
F 5 "3.8" H 3950 2850 50  0001 L CNN "Height"
F 6 "798-FX10A80P8SV71" H 3950 2750 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Hirose-Connector/FX10A-80P-8-SV71?qs=zOgoZG1CgI%252B6a4LOiFBFhA%3D%3D" H 3950 2650 50  0001 L CNN "Mouser Price/Stock"
F 8 "Hirose" H 3950 2550 50  0001 L CNN "Manufacturer_Name"
F 9 "FX10A-80P/8-SV(71)" H 3950 2450 50  0001 L CNN "Manufacturer_Part_Number"
F 10 "-" H 3100 3050 50  0001 C CNN "LCSC part #"
	1    3100 3050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 615FD4FD
P 2250 1850
AR Path="/615FD4FD" Ref="#PWR?"  Part="1" 
AR Path="/62731E6D/615FD4FD" Ref="#PWR0117"  Part="1" 
F 0 "#PWR0117" H 2250 1600 50  0001 C CNN
F 1 "GND" H 2255 1677 50  0000 C CNN
F 2 "" H 2250 1850 50  0001 C CNN
F 3 "" H 2250 1850 50  0001 C CNN
	1    2250 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 1750 2250 1850
Wire Wire Line
	2150 1750 2250 1750
$Comp
L Connector:Screw_Terminal_01x02 J?
U 1 1 615FD505
P 1950 1650
AR Path="/615FD505" Ref="J?"  Part="1" 
AR Path="/62731E6D/615FD505" Ref="J1"  Part="1" 
F 0 "J1" H 1868 1325 50  0000 C CNN
F 1 "POWER" H 1950 1400 50  0000 C CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_PT-1,5-2-3.5-H_1x02_P3.50mm_Horizontal" H 1950 1650 50  0001 C CNN
F 3 "~" H 1950 1650 50  0001 C CNN
F 4 "C474892" H 1950 1650 50  0001 C CNN "LCSC part #"
	1    1950 1650
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2150 1650 2250 1650
Wire Wire Line
	2250 1650 2250 1600
$Comp
L power:VCC #PWR?
U 1 1 615FD50D
P 2250 1600
AR Path="/615FD50D" Ref="#PWR?"  Part="1" 
AR Path="/62731E6D/615FD50D" Ref="#PWR0118"  Part="1" 
F 0 "#PWR0118" H 2250 1450 50  0001 C CNN
F 1 "VCC" H 2265 1773 50  0000 C CNN
F 2 "" H 2250 1600 50  0001 C CNN
F 3 "" H 2250 1600 50  0001 C CNN
	1    2250 1600
	1    0    0    -1  
$EndComp
$Comp
L Power_Protection:USBLC6-2SC6 U?
U 1 1 615FD513
P 5900 1650
AR Path="/615FD513" Ref="U?"  Part="1" 
AR Path="/62731E6D/615FD513" Ref="U2"  Part="1" 
F 0 "U2" H 6200 2000 50  0000 C CNN
F 1 "USBLC6-2SC6" H 6250 1300 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6" H 5900 1150 50  0001 C CNN
F 3 "https://www.st.com/resource/en/datasheet/usblc6-2.pdf" H 6100 2000 50  0001 C CNN
F 4 "C2827654" H 5900 1650 50  0001 C CNN "LCSC part #"
	1    5900 1650
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 615FD519
P 5900 1200
AR Path="/615FD519" Ref="#PWR?"  Part="1" 
AR Path="/62731E6D/615FD519" Ref="#PWR0119"  Part="1" 
F 0 "#PWR0119" H 5900 1050 50  0001 C CNN
F 1 "+5V" H 5915 1373 50  0000 C CNN
F 2 "" H 5900 1200 50  0001 C CNN
F 3 "" H 5900 1200 50  0001 C CNN
	1    5900 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 1200 5900 1250
$Comp
L power:GND #PWR?
U 1 1 615FD520
P 5900 2100
AR Path="/615FD520" Ref="#PWR?"  Part="1" 
AR Path="/62731E6D/615FD520" Ref="#PWR0120"  Part="1" 
F 0 "#PWR0120" H 5900 1850 50  0001 C CNN
F 1 "GND" H 5905 1927 50  0000 C CNN
F 2 "" H 5900 2100 50  0001 C CNN
F 3 "" H 5900 2100 50  0001 C CNN
	1    5900 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 2050 5900 2100
Text GLabel 5450 1550 0    50   Input ~ 0
USB_D-
Text GLabel 6350 1550 2    50   Input ~ 0
USB_D+
Wire Wire Line
	6300 1550 6350 1550
Wire Wire Line
	5450 1550 5500 1550
Text GLabel 5450 1750 0    50   Input ~ 0
USB_CONN_D-
Text GLabel 6350 1750 2    50   Input ~ 0
USB_CONN_D+
Wire Wire Line
	5450 1750 5500 1750
Wire Wire Line
	6300 1750 6350 1750
$Comp
L max_KiCAD_lib:C10418 J?
U 1 1 61607D65
P 3100 1100
AR Path="/61607D65" Ref="J?"  Part="1" 
AR Path="/62731E6D/61607D65" Ref="J3"  Part="1" 
F 0 "J3" V 3746 1228 50  0000 L CNN
F 1 "C10418" V 3655 1228 50  0000 L CNN
F 2 "max_KiCAD_lib:C10418" H 4150 1200 50  0001 L CNN
F 3 "https://datasheet.lcsc.com/szlcsc/Jing-Extension-of-the-Electronic-Co-LCSC-MICRO-USB-5S-B-Type-horns-High-temperature_C10418.pdf" H 4150 1100 50  0001 L CNN
F 4 "USB Connectors SMD RoHS" H 4150 1000 50  0001 L CNN "Description"
F 5 "3" H 4150 900 50  0001 L CNN "Height"
F 6 "Jing Extension of the Electronic Co." H 4150 800 50  0001 L CNN "Manufacturer_Name"
F 7 "C10418" H 4150 700 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "C10418" H 3100 1100 50  0001 C CNN "LCSC part #"
	1    3100 1100
	0    -1   1    0   
$EndComp
NoConn ~ 3700 1800
$Comp
L power:GND #PWR?
U 1 1 61607D6C
P 3850 2000
AR Path="/61607D6C" Ref="#PWR?"  Part="1" 
AR Path="/62731E6D/61607D6C" Ref="#PWR0121"  Part="1" 
F 0 "#PWR0121" H 3850 1750 50  0001 C CNN
F 1 "GND" H 3855 1827 50  0000 C CNN
F 2 "" H 3850 2000 50  0001 C CNN
F 3 "" H 3850 2000 50  0001 C CNN
	1    3850 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 1900 3850 1900
Wire Wire Line
	3850 1900 3850 2000
NoConn ~ 3100 2300
NoConn ~ 3700 2000
NoConn ~ 3700 1400
NoConn ~ 3100 1100
Wire Wire Line
	3700 1500 3850 1500
Wire Wire Line
	3850 1500 3850 1400
$Comp
L power:+5V #PWR?
U 1 1 61607D7A
P 3850 1400
AR Path="/61607D7A" Ref="#PWR?"  Part="1" 
AR Path="/62731E6D/61607D7A" Ref="#PWR0122"  Part="1" 
F 0 "#PWR0122" H 3850 1250 50  0001 C CNN
F 1 "+5V" H 3865 1573 50  0000 C CNN
F 2 "" H 3850 1400 50  0001 C CNN
F 3 "" H 3850 1400 50  0001 C CNN
	1    3850 1400
	1    0    0    -1  
$EndComp
Text GLabel 3850 1600 2    50   Input ~ 0
USB_CONN_D-
Text GLabel 3850 1700 2    50   Input ~ 0
USB_CONN_D+
Wire Wire Line
	3700 1600 3850 1600
Wire Wire Line
	3700 1700 3850 1700
$Comp
L Connector_Generic:Conn_02x05_Odd_Even J?
U 1 1 6168371F
P 7950 1600
AR Path="/6168371F" Ref="J?"  Part="1" 
AR Path="/62731E6D/6168371F" Ref="J4"  Part="1" 
F 0 "J4" H 8000 2017 50  0000 C CNN
F 1 "SWD" H 8000 1250 50  0000 C CNN
F 2 "Connector_PinHeader_1.27mm:PinHeader_2x05_P1.27mm_Vertical" H 7950 1600 50  0001 C CNN
F 3 "~" H 7950 1600 50  0001 C CNN
F 4 "-" H 7950 1600 50  0001 C CNN "LCSC part #"
	1    7950 1600
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 61683725
P 7600 1350
AR Path="/61683725" Ref="#PWR?"  Part="1" 
AR Path="/62731E6D/61683725" Ref="#PWR0123"  Part="1" 
F 0 "#PWR0123" H 7600 1200 50  0001 C CNN
F 1 "+3.3V" H 7615 1523 50  0000 C CNN
F 2 "" H 7600 1350 50  0001 C CNN
F 3 "" H 7600 1350 50  0001 C CNN
	1    7600 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 1350 7600 1400
Wire Wire Line
	7600 1400 7750 1400
$Comp
L power:GND #PWR?
U 1 1 6168372D
P 7600 1900
AR Path="/6168372D" Ref="#PWR?"  Part="1" 
AR Path="/62731E6D/6168372D" Ref="#PWR0124"  Part="1" 
F 0 "#PWR0124" H 7600 1650 50  0001 C CNN
F 1 "GND" H 7605 1727 50  0000 C CNN
F 2 "" H 7600 1900 50  0001 C CNN
F 3 "" H 7600 1900 50  0001 C CNN
	1    7600 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 1500 7600 1500
Wire Wire Line
	7600 1500 7600 1600
Wire Wire Line
	7750 1800 7600 1800
Connection ~ 7600 1800
Wire Wire Line
	7600 1800 7600 1900
NoConn ~ 7750 1700
Wire Wire Line
	8350 1400 8250 1400
$Comp
L Device:R_Small R?
U 1 1 6168373A
P 8450 1400
AR Path="/6168373A" Ref="R?"  Part="1" 
AR Path="/62731E6D/6168373A" Ref="R7"  Part="1" 
F 0 "R7" V 8400 1550 50  0000 C CNN
F 1 "22" V 8400 1250 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 8450 1400 50  0001 C CNN
F 3 "~" H 8450 1400 50  0001 C CNN
F 4 "C17958" H 8450 1400 50  0001 C CNN "LCSC part #"
	1    8450 1400
	0    -1   1    0   
$EndComp
Wire Wire Line
	8250 1500 8350 1500
$Comp
L Device:R_Small R?
U 1 1 61683741
P 8450 1500
AR Path="/61683741" Ref="R?"  Part="1" 
AR Path="/62731E6D/61683741" Ref="R8"  Part="1" 
F 0 "R8" V 8400 1650 50  0000 C CNN
F 1 "22" V 8400 1350 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 8450 1500 50  0001 C CNN
F 3 "~" H 8450 1500 50  0001 C CNN
F 4 "C17958" H 8450 1500 50  0001 C CNN "LCSC part #"
	1    8450 1500
	0    -1   1    0   
$EndComp
Wire Wire Line
	8550 1400 8900 1400
Wire Wire Line
	8550 1500 8900 1500
Wire Wire Line
	7600 1600 7750 1600
Connection ~ 7600 1600
Wire Wire Line
	7600 1600 7600 1800
$Comp
L Device:R_Small R?
U 1 1 6168374C
P 8450 1800
AR Path="/6168374C" Ref="R?"  Part="1" 
AR Path="/62731E6D/6168374C" Ref="R10"  Part="1" 
F 0 "R10" V 8400 1950 50  0000 C CNN
F 1 "22" V 8400 1650 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 8450 1800 50  0001 C CNN
F 3 "~" H 8450 1800 50  0001 C CNN
F 4 "C17958" H 8450 1800 50  0001 C CNN "LCSC part #"
	1    8450 1800
	0    -1   1    0   
$EndComp
$Comp
L Device:C_Small C?
U 1 1 61683752
P 8650 1950
AR Path="/61683752" Ref="C?"  Part="1" 
AR Path="/62731E6D/61683752" Ref="C11"  Part="1" 
F 0 "C11" H 8750 2000 50  0000 L CNN
F 1 "100n" H 8750 1900 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8650 1950 50  0001 C CNN
F 3 "~" H 8650 1950 50  0001 C CNN
F 4 "C14663" H 8650 1950 50  0001 C CNN "LCSC part #"
	1    8650 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 1800 8350 1800
Wire Wire Line
	8550 1800 8650 1800
Wire Wire Line
	8650 1800 8650 1850
Wire Wire Line
	8650 2050 8650 2100
$Comp
L power:GND #PWR?
U 1 1 6168375C
P 8650 2100
AR Path="/6168375C" Ref="#PWR?"  Part="1" 
AR Path="/62731E6D/6168375C" Ref="#PWR0125"  Part="1" 
F 0 "#PWR0125" H 8650 1850 50  0001 C CNN
F 1 "GND" H 8655 1927 50  0000 C CNN
F 2 "" H 8650 2100 50  0001 C CNN
F 3 "" H 8650 2100 50  0001 C CNN
	1    8650 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 1800 8900 1800
Connection ~ 8650 1800
$Comp
L Device:R_Small R?
U 1 1 61683764
P 8450 1600
AR Path="/61683764" Ref="R?"  Part="1" 
AR Path="/62731E6D/61683764" Ref="R9"  Part="1" 
F 0 "R9" V 8400 1750 50  0000 C CNN
F 1 "22" V 8400 1450 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 8450 1600 50  0001 C CNN
F 3 "~" H 8450 1600 50  0001 C CNN
F 4 "C17958" H 8450 1600 50  0001 C CNN "LCSC part #"
	1    8450 1600
	0    -1   1    0   
$EndComp
Wire Wire Line
	8250 1600 8350 1600
Wire Wire Line
	8550 1600 8900 1600
NoConn ~ 8250 1700
Text GLabel 8900 1400 2    50   Input ~ 0
SWDIO
Text GLabel 8900 1500 2    50   Input ~ 0
SWCLK
Text GLabel 8900 1600 2    50   Input ~ 0
SWO
Text GLabel 8900 1800 2    50   Input ~ 0
NRST
Wire Wire Line
	2600 5450 3100 5450
Wire Wire Line
	2600 5650 3100 5650
Wire Wire Line
	2600 6050 3100 6050
Wire Wire Line
	2600 5850 3100 5850
Wire Wire Line
	2600 6250 3100 6250
Wire Wire Line
	2600 6650 3100 6650
Wire Wire Line
	2600 6450 3100 6450
Wire Wire Line
	4600 5950 4100 5950
Wire Wire Line
	4600 6150 4100 6150
Wire Wire Line
	4600 6350 4100 6350
Wire Wire Line
	2600 6950 3100 6950
Wire Wire Line
	2600 7450 3100 7450
Wire Wire Line
	2600 7250 3100 7250
Wire Wire Line
	2600 5950 3100 5950
Wire Wire Line
	2600 4150 3100 4150
Wire Wire Line
	2600 3950 3100 3950
Wire Wire Line
	2600 3750 3100 3750
Wire Wire Line
	2600 3550 3100 3550
Wire Wire Line
	2600 3350 3100 3350
Wire Wire Line
	4600 3550 4100 3550
Text GLabel 2600 3550 0    50   Input ~ 0
I2C1_SCL
Text GLabel 2600 3350 0    50   Input ~ 0
I2C1_SDA
Text GLabel 2600 5450 0    50   Input ~ 0
PA0
Text GLabel 2600 5650 0    50   Input ~ 0
PA1
Text GLabel 2600 5850 0    50   Input ~ 0
PA3
Text GLabel 2600 6050 0    50   Input ~ 0
PA4
Text GLabel 2600 6250 0    50   Input ~ 0
PA5
Text GLabel 2600 6450 0    50   Input ~ 0
PA6
Text GLabel 2600 6650 0    50   Input ~ 0
PA7
Text GLabel 4600 5950 2    50   Input ~ 0
PA8
Text GLabel 4600 6150 2    50   Input ~ 0
PA9
Text GLabel 4600 6350 2    50   Input ~ 0
PA10
Text GLabel 2600 6950 0    50   Input ~ 0
PA15
Text GLabel 2600 7250 0    50   Input ~ 0
PB0
Text GLabel 2600 7450 0    50   Input ~ 0
PB1
Text GLabel 2600 5950 0    50   Input ~ 0
PB4
Text GLabel 2600 4150 0    50   Input ~ 0
PB5
Text GLabel 2600 3950 0    50   Input ~ 0
PB6
Text GLabel 2600 3750 0    50   Input ~ 0
PB7
Text GLabel 4600 3550 2    50   Input ~ 0
PB10
Text GLabel 2600 4650 0    50   Input ~ 0
PC2
Text GLabel 2600 5050 0    50   Input ~ 0
PC3
Text GLabel 2600 6150 0    50   Input ~ 0
PD2
Text GLabel 2600 3650 0    50   Input ~ 0
PC0
Text GLabel 2600 4450 0    50   Input ~ 0
PC1
Text GLabel 2600 6850 0    50   Input ~ 0
PC4
Text GLabel 2600 7050 0    50   Input ~ 0
PC5
Text GLabel 4600 5550 2    50   Input ~ 0
PC8
Text GLabel 4600 5750 2    50   Input ~ 0
PC9
Text GLabel 2600 6750 0    50   Input ~ 0
PC10
Text GLabel 2600 6550 0    50   Input ~ 0
PC11
Text GLabel 2600 6350 0    50   Input ~ 0
PC12
Text GLabel 2600 3050 0    50   Input ~ 0
PC13
Wire Wire Line
	3100 3050 2600 3050
Wire Wire Line
	3100 6350 2600 6350
Wire Wire Line
	3100 6550 2600 6550
Wire Wire Line
	3100 6750 2600 6750
Wire Wire Line
	4100 5750 4600 5750
Wire Wire Line
	4100 5550 4600 5550
Wire Wire Line
	3100 7050 2600 7050
Wire Wire Line
	3100 6850 2600 6850
Wire Wire Line
	3100 5050 2600 5050
Wire Wire Line
	3100 4650 2600 4650
Wire Wire Line
	3100 4450 2600 4450
Wire Wire Line
	3100 3650 2600 3650
Wire Wire Line
	3100 6150 2600 6150
$EndSCHEMATC
