EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 3
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
L MCU_ST_STM32F4:STM32F446RETx U1
U 1 1 6138BFE8
P 4150 3900
F 0 "U1" H 4700 5550 50  0000 C CNN
F 1 "STM32F446RETx" H 4700 2150 50  0000 C CNN
F 2 "Package_QFP:LQFP-64_10x10mm_P0.5mm" H 3550 2200 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00141306.pdf" H 4150 3900 50  0001 C CNN
F 4 "-" H 4150 3900 50  0001 C CNN "LCSC part #"
	1    4150 3900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 6144AE55
P 3950 5800
F 0 "#PWR0101" H 3950 5550 50  0001 C CNN
F 1 "GND" H 3955 5627 50  0000 C CNN
F 2 "" H 3950 5800 50  0001 C CNN
F 3 "" H 3950 5800 50  0001 C CNN
	1    3950 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 5700 3950 5750
Wire Wire Line
	4350 5700 4350 5750
Wire Wire Line
	4350 5750 4250 5750
Connection ~ 3950 5750
Wire Wire Line
	3950 5750 3950 5800
Wire Wire Line
	4050 5700 4050 5750
Connection ~ 4050 5750
Wire Wire Line
	4050 5750 3950 5750
Wire Wire Line
	4150 5700 4150 5750
Connection ~ 4150 5750
Wire Wire Line
	4150 5750 4050 5750
Wire Wire Line
	4250 5700 4250 5750
Connection ~ 4250 5750
Wire Wire Line
	4250 5750 4150 5750
Wire Wire Line
	3950 2100 3950 2150
Wire Wire Line
	3950 2150 4050 2150
Connection ~ 3950 2150
Wire Wire Line
	3950 2150 3950 2200
Wire Wire Line
	4350 2200 4350 2150
Wire Wire Line
	4250 2200 4250 2150
Connection ~ 4250 2150
Wire Wire Line
	4250 2150 4350 2150
Wire Wire Line
	4150 2200 4150 2150
Connection ~ 4150 2150
Wire Wire Line
	4150 2150 4250 2150
Wire Wire Line
	4050 2200 4050 2150
Connection ~ 4050 2150
Wire Wire Line
	4050 2150 4150 2150
$Comp
L power:+3.3VA #PWR0102
U 1 1 6144E9B0
P 4450 2100
F 0 "#PWR0102" H 4450 1950 50  0001 C CNN
F 1 "+3.3VA" H 4465 2273 50  0000 C CNN
F 2 "" H 4450 2100 50  0001 C CNN
F 3 "" H 4450 2100 50  0001 C CNN
	1    4450 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 2100 4450 2200
$Comp
L power:+3.3V #PWR0103
U 1 1 6144F92A
P 3950 2100
F 0 "#PWR0103" H 3950 1950 50  0001 C CNN
F 1 "+3.3V" H 3965 2273 50  0000 C CNN
F 2 "" H 3950 2100 50  0001 C CNN
F 3 "" H 3950 2100 50  0001 C CNN
	1    3950 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 2600 3000 2600
Text Label 3000 2600 0    50   ~ 0
BOOT0
$Comp
L Device:C_Small C4
U 1 1 61450E2B
P 3250 2950
F 0 "C4" H 3342 2996 50  0000 L CNN
F 1 "4.7u" H 3342 2905 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 3250 2950 50  0001 C CNN
F 3 "~" H 3250 2950 50  0001 C CNN
F 4 "C29823" H 3250 2950 50  0001 C CNN "LCSC part #"
	1    3250 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 2800 3250 2800
Wire Wire Line
	3250 2800 3250 2850
$Comp
L power:GND #PWR0104
U 1 1 61451D54
P 3250 3100
F 0 "#PWR0104" H 3250 2850 50  0001 C CNN
F 1 "GND" H 3255 2927 50  0000 C CNN
F 2 "" H 3250 3100 50  0001 C CNN
F 3 "" H 3250 3100 50  0001 C CNN
	1    3250 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 3050 3250 3100
Text Notes 2900 3150 0    50   ~ 0
ESR 1 Ω \nor below
Wire Wire Line
	3450 3500 3000 3500
Wire Wire Line
	3450 3600 3000 3600
Text Label 3000 3500 0    50   ~ 0
HSE_IN
Text Label 3000 3600 0    50   ~ 0
HSE_OUT
Wire Wire Line
	4850 2600 5300 2600
Text Label 5300 2600 2    50   ~ 0
LED_STATUS
Connection ~ 5350 1450
Wire Wire Line
	5350 1450 5350 1500
$Comp
L power:GND #PWR0105
U 1 1 61502F67
P 5350 1500
F 0 "#PWR0105" H 5350 1250 50  0001 C CNN
F 1 "GND" H 5355 1327 50  0000 C CNN
F 2 "" H 5350 1500 50  0001 C CNN
F 3 "" H 5350 1500 50  0001 C CNN
	1    5350 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 1450 5750 1400
Wire Wire Line
	5350 1450 5750 1450
Wire Wire Line
	5350 1400 5350 1450
Connection ~ 5750 1150
Wire Wire Line
	5750 1150 5750 1200
Connection ~ 5350 1150
Wire Wire Line
	5750 1150 5750 1100
Wire Wire Line
	5350 1150 5750 1150
$Comp
L power:+3.3VA #PWR0106
U 1 1 614F684A
P 5750 1100
F 0 "#PWR0106" H 5750 950 50  0001 C CNN
F 1 "+3.3VA" H 5765 1273 50  0000 C CNN
F 2 "" H 5750 1100 50  0001 C CNN
F 3 "" H 5750 1100 50  0001 C CNN
	1    5750 1100
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C9
U 1 1 614F20D2
P 5750 1300
F 0 "C9" H 5842 1346 50  0000 L CNN
F 1 "10n" H 5842 1255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5750 1300 50  0001 C CNN
F 3 "~" H 5750 1300 50  0001 C CNN
F 4 "C1710" H 5750 1300 50  0001 C CNN "LCSC part #"
	1    5750 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 1150 5350 1200
Wire Wire Line
	5150 1150 5350 1150
$Comp
L Device:C_Small C8
U 1 1 614EDF09
P 5350 1300
F 0 "C8" H 5442 1346 50  0000 L CNN
F 1 "1u" H 5442 1255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5350 1300 50  0001 C CNN
F 3 "~" H 5350 1300 50  0001 C CNN
F 4 "C28323" H 5350 1300 50  0001 C CNN "LCSC part #"
	1    5350 1300
	1    0    0    -1  
$EndComp
Connection ~ 2300 1450
Wire Wire Line
	2300 1450 2300 1500
$Comp
L power:GND #PWR0108
U 1 1 614D41C4
P 2300 1500
F 0 "#PWR0108" H 2300 1250 50  0001 C CNN
F 1 "GND" H 2305 1327 50  0000 C CNN
F 2 "" H 2300 1500 50  0001 C CNN
F 3 "" H 2300 1500 50  0001 C CNN
	1    2300 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 1450 3100 1450
Connection ~ 2700 1450
Wire Wire Line
	2700 1400 2700 1450
Wire Wire Line
	3100 1450 3500 1450
Connection ~ 3100 1450
Wire Wire Line
	3100 1400 3100 1450
Wire Wire Line
	3500 1450 3900 1450
Connection ~ 3500 1450
Wire Wire Line
	3500 1400 3500 1450
Wire Wire Line
	3900 1450 4300 1450
Connection ~ 3900 1450
Wire Wire Line
	3900 1400 3900 1450
Wire Wire Line
	4300 1450 4300 1400
Wire Wire Line
	2300 1450 2700 1450
Wire Wire Line
	2300 1400 2300 1450
Wire Wire Line
	3100 1150 3500 1150
Connection ~ 3100 1150
Wire Wire Line
	3100 1200 3100 1150
Wire Wire Line
	2700 1150 3100 1150
Connection ~ 2700 1150
Wire Wire Line
	2700 1200 2700 1150
Wire Wire Line
	3500 1150 3900 1150
Connection ~ 3500 1150
Wire Wire Line
	3500 1200 3500 1150
Wire Wire Line
	3900 1150 4300 1150
Connection ~ 3900 1150
Wire Wire Line
	3900 1200 3900 1150
Wire Wire Line
	2300 1150 2300 1100
Connection ~ 2300 1150
Wire Wire Line
	4300 1150 4300 1200
Wire Wire Line
	2300 1150 2700 1150
$Comp
L power:+3.3V #PWR0109
U 1 1 614B43E9
P 2300 1100
F 0 "#PWR0109" H 2300 950 50  0001 C CNN
F 1 "+3.3V" H 2315 1273 50  0000 C CNN
F 2 "" H 2300 1100 50  0001 C CNN
F 3 "" H 2300 1100 50  0001 C CNN
	1    2300 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 1200 2300 1150
$Comp
L Device:C_Small C7
U 1 1 614B157B
P 4300 1300
F 0 "C7" H 4392 1346 50  0000 L CNN
F 1 "100n" H 4392 1255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4300 1300 50  0001 C CNN
F 3 "~" H 4300 1300 50  0001 C CNN
F 4 "C14663" H 4300 1300 50  0001 C CNN "LCSC part #"
	1    4300 1300
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C6
U 1 1 614B106B
P 3900 1300
F 0 "C6" H 3992 1346 50  0000 L CNN
F 1 "100n" H 3992 1255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3900 1300 50  0001 C CNN
F 3 "~" H 3900 1300 50  0001 C CNN
F 4 "C14663" H 3900 1300 50  0001 C CNN "LCSC part #"
	1    3900 1300
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C5
U 1 1 614B09E1
P 3500 1300
F 0 "C5" H 3592 1346 50  0000 L CNN
F 1 "100n" H 3592 1255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3500 1300 50  0001 C CNN
F 3 "~" H 3500 1300 50  0001 C CNN
F 4 "C14663" H 3500 1300 50  0001 C CNN "LCSC part #"
	1    3500 1300
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 614B062B
P 3100 1300
F 0 "C3" H 3192 1346 50  0000 L CNN
F 1 "100n" H 3192 1255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3100 1300 50  0001 C CNN
F 3 "~" H 3100 1300 50  0001 C CNN
F 4 "C14663" H 3100 1300 50  0001 C CNN "LCSC part #"
	1    3100 1300
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C2
U 1 1 614AFDDF
P 2700 1300
F 0 "C2" H 2792 1346 50  0000 L CNN
F 1 "100n" H 2792 1255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2700 1300 50  0001 C CNN
F 3 "~" H 2700 1300 50  0001 C CNN
F 4 "C14663" H 2700 1300 50  0001 C CNN "LCSC part #"
	1    2700 1300
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C1
U 1 1 614AF416
P 2300 1300
F 0 "C1" H 2392 1346 50  0000 L CNN
F 1 "4.7u" H 2392 1255 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 2300 1300 50  0001 C CNN
F 3 "~" H 2300 1300 50  0001 C CNN
F 4 "C29823" H 2300 1300 50  0001 C CNN "LCSC part #"
	1    2300 1300
	1    0    0    -1  
$EndComp
$Comp
L Device:Crystal_GND24_Small Y1
U 1 1 61531C1C
P 6900 2700
F 0 "Y1" H 6850 3050 50  0000 L CNN
F 1 "16MHz" H 6750 2950 50  0000 L CNN
F 2 "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm" H 6900 2700 50  0001 C CNN
F 3 "~" H 6900 2700 50  0001 C CNN
F 4 "C13738" H 6900 2700 50  0001 C CNN "LCSC part #"
	1    6900 2700
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C10
U 1 1 61536AEC
P 6650 2950
F 0 "C10" H 6742 2996 50  0000 L CNN
F 1 "12p" H 6742 2905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6650 2950 50  0001 C CNN
F 3 "~" H 6650 2950 50  0001 C CNN
F 4 "C38523" H 6650 2950 50  0001 C CNN "LCSC part #"
	1    6650 2950
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C12
U 1 1 61537D59
P 7150 2950
F 0 "C12" H 7242 2996 50  0000 L CNN
F 1 "12p" H 7242 2905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7150 2950 50  0001 C CNN
F 3 "~" H 7150 2950 50  0001 C CNN
F 4 "C38523" H 7150 2950 50  0001 C CNN "LCSC part #"
	1    7150 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 2700 6650 2700
Wire Wire Line
	6650 2700 6650 2850
Wire Wire Line
	7000 2700 7150 2700
Wire Wire Line
	7150 2700 7150 2850
$Comp
L power:GND #PWR0110
U 1 1 61541A66
P 6900 3150
F 0 "#PWR0110" H 6900 2900 50  0001 C CNN
F 1 "GND" H 6905 2977 50  0000 C CNN
F 2 "" H 6900 3150 50  0001 C CNN
F 3 "" H 6900 3150 50  0001 C CNN
	1    6900 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 3050 6650 3100
Wire Wire Line
	6650 3100 6900 3100
Wire Wire Line
	6900 3100 6900 3150
Wire Wire Line
	6900 3100 7150 3100
Wire Wire Line
	7150 3100 7150 3050
Connection ~ 6900 3100
Wire Wire Line
	6900 2800 6900 2850
Wire Wire Line
	6900 2600 6900 2550
Wire Wire Line
	6900 2550 7050 2550
Wire Wire Line
	7050 2550 7050 2850
Wire Wire Line
	7050 2850 6900 2850
Connection ~ 6900 2850
Wire Wire Line
	6900 2850 6900 3100
$Comp
L Device:R_Small R3
U 1 1 615567E4
P 7150 2400
F 0 "R3" H 7300 2500 50  0000 C CNN
F 1 "47" H 7300 2400 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 7150 2400 50  0001 C CNN
F 3 "~" H 7150 2400 50  0001 C CNN
F 4 "C23182" H 7150 2400 50  0001 C CNN "LCSC part #"
	1    7150 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 2500 7150 2700
Connection ~ 7150 2700
Wire Wire Line
	6650 2700 6650 2100
Wire Wire Line
	6650 2100 7500 2100
Connection ~ 6650 2700
Wire Wire Line
	7150 2300 7150 2200
Wire Wire Line
	7150 2200 7500 2200
Text Label 7500 2100 2    50   ~ 0
HSE_IN
Text Label 7500 2200 2    50   ~ 0
HSE_OUT
Text Notes 6500 3500 0    50   ~ 0
C_load = 2 * (CL -Cstray)\n
Wire Wire Line
	2200 2700 2200 2750
Wire Wire Line
	2300 2700 2200 2700
$Comp
L power:GND #PWR0111
U 1 1 61458042
P 2200 2750
F 0 "#PWR0111" H 2200 2500 50  0001 C CNN
F 1 "GND" H 2205 2577 50  0000 C CNN
F 2 "" H 2200 2750 50  0001 C CNN
F 3 "" H 2200 2750 50  0001 C CNN
	1    2200 2750
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2200 2500 2200 2450
Wire Wire Line
	2300 2500 2200 2500
$Comp
L power:+3.3V #PWR0112
U 1 1 61456743
P 2200 2450
F 0 "#PWR0112" H 2200 2300 50  0001 C CNN
F 1 "+3.3V" H 2215 2623 50  0000 C CNN
F 2 "" H 2200 2450 50  0001 C CNN
F 3 "" H 2200 2450 50  0001 C CNN
	1    2200 2450
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2800 2600 2700 2600
$Comp
L Device:R_Small R1
U 1 1 61455161
P 2900 2600
F 0 "R1" V 2800 2600 50  0000 C CNN
F 1 "10k" V 3000 2600 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 2900 2600 50  0001 C CNN
F 3 "~" H 2900 2600 50  0001 C CNN
F 4 "C17902" H 2900 2600 50  0001 C CNN "LCSC part #"
	1    2900 2600
	0    -1   1    0   
$EndComp
$Comp
L Switch:SW_SPDT SW1
U 1 1 61452EA4
P 2500 2600
F 0 "SW1" H 2500 2885 50  0000 C CNN
F 1 "SW_SPDT" H 2500 2794 50  0000 C CNN
F 2 "max_KiCAD_lib:SPDT_Slide_Switches" H 2500 2600 50  0001 C CNN
F 3 "~" H 2500 2600 50  0001 C CNN
F 4 "C145856" H 2500 2600 50  0001 C CNN "LCSC part #"
	1    2500 2600
	-1   0    0    -1  
$EndComp
$Comp
L Device:LED_Small D1
U 1 1 614DD3BA
P 1050 2600
F 0 "D1" V 1096 2530 50  0000 R CNN
F 1 "BLUE" V 1005 2530 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" V 1050 2600 50  0001 C CNN
F 3 "~" V 1050 2600 50  0001 C CNN
F 4 "C72041" H 1050 2600 50  0001 C CNN "LCSC part #"
	1    1050 2600
	0    -1   -1   0   
$EndComp
Text Label 950  2350 2    50   ~ 0
LED_STATUS
Wire Wire Line
	1050 2350 950  2350
Wire Wire Line
	1050 2350 1050 2500
$Comp
L Device:R_Small R2
U 1 1 614E96EF
P 1050 2850
F 0 "R2" H 950 2900 50  0000 C CNN
F 1 "1.5k" H 900 2800 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 1050 2850 50  0001 C CNN
F 3 "~" H 1050 2850 50  0001 C CNN
F 4 "C4310" H 1050 2850 50  0001 C CNN "LCSC part #"
	1    1050 2850
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1050 2700 1050 2750
$Comp
L Device:Ferrite_Bead_Small FB2
U 1 1 61AB3042
P 5050 1150
F 0 "FB2" V 4800 1100 50  0000 L CNN
F 1 "600 @ 600MHz" V 4900 900 50  0000 L CNN
F 2 "Inductor_SMD:L_0805_2012Metric" V 4980 1150 50  0001 C CNN
F 3 "~" H 5050 1150 50  0001 C CNN
F 4 "C1017" H 5050 1150 50  0001 C CNN "LCSC part #"
	1    5050 1150
	0    1    1    0   
$EndComp
Wire Wire Line
	4650 1150 4650 1100
$Comp
L power:+3.3V #PWR0107
U 1 1 614E9A5F
P 4650 1100
F 0 "#PWR0107" H 4650 950 50  0001 C CNN
F 1 "+3.3V" H 4665 1273 50  0000 C CNN
F 2 "" H 4650 1100 50  0001 C CNN
F 3 "" H 4650 1100 50  0001 C CNN
	1    4650 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 1150 4950 1150
$Comp
L power:GND #PWR0113
U 1 1 614F4F24
P 1050 3000
F 0 "#PWR0113" H 1050 2750 50  0001 C CNN
F 1 "GND" H 1055 2827 50  0000 C CNN
F 2 "" H 1050 3000 50  0001 C CNN
F 3 "" H 1050 3000 50  0001 C CNN
	1    1050 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1050 2950 1050 3000
Wire Wire Line
	5350 2400 4850 2400
Wire Wire Line
	5350 2500 4850 2500
Wire Wire Line
	5350 2800 4850 2800
Wire Wire Line
	5350 2700 4850 2700
Wire Wire Line
	5350 2900 4850 2900
Wire Wire Line
	5350 3000 4850 3000
Wire Wire Line
	5350 3100 4850 3100
Wire Wire Line
	5350 3200 4850 3200
Wire Wire Line
	5350 3300 4850 3300
Wire Wire Line
	5350 3400 4850 3400
Wire Wire Line
	5350 3900 4850 3900
Wire Wire Line
	5350 4100 4850 4100
Wire Wire Line
	5350 4200 4850 4200
Wire Wire Line
	5350 4500 4850 4500
Wire Wire Line
	5350 4600 4850 4600
Wire Wire Line
	5350 4700 4850 4700
Wire Wire Line
	5350 4800 4850 4800
Wire Wire Line
	5350 4900 4850 4900
Wire Wire Line
	5350 5000 4850 5000
Wire Wire Line
	5350 5100 4850 5100
Wire Wire Line
	5350 5200 4850 5200
Wire Wire Line
	5350 5300 4850 5300
Wire Wire Line
	5350 5400 4850 5400
Wire Wire Line
	5350 5500 4850 5500
Text Label 3000 5500 0    50   ~ 0
OSC32_OUT
Text Label 3000 5400 0    50   ~ 0
OSC32_IN
Wire Wire Line
	3000 5400 3450 5400
Wire Wire Line
	3000 5500 3450 5500
$Comp
L Device:Crystal_Small Y2
U 1 1 623DA3C9
P 7000 4250
F 0 "Y2" H 7000 4475 50  0000 C CNN
F 1 "32.768kHz" H 7000 4384 50  0000 C CNN
F 2 "Crystal:Crystal_SMD_2012-2Pin_2.0x1.2mm" H 7000 4250 50  0001 C CNN
F 3 "~" H 7000 4250 50  0001 C CNN
F 4 "C97602" H 7000 4250 50  0001 C CNN "LCSC part #"
	1    7000 4250
	1    0    0    -1  
$EndComp
Text Label 6350 4250 0    50   ~ 0
OSC32_IN
Text Label 7650 4250 2    50   ~ 0
OSC32_OUT
$Comp
L Device:C_Small C18
U 1 1 624237D5
P 6850 4450
F 0 "C18" H 6942 4496 50  0000 L CNN
F 1 "10p" H 6942 4405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6850 4450 50  0001 C CNN
F 3 "~" H 6850 4450 50  0001 C CNN
F 4 "C32949" H 6850 4450 50  0001 C CNN "LCSC part #"
	1    6850 4450
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C19
U 1 1 6242442F
P 7150 4450
F 0 "C19" H 7242 4496 50  0000 L CNN
F 1 "10p" H 7242 4405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 7150 4450 50  0001 C CNN
F 3 "~" H 7150 4450 50  0001 C CNN
F 4 "C32949" H 7150 4450 50  0001 C CNN "LCSC part #"
	1    7150 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 4250 7150 4250
Wire Wire Line
	6850 4250 6850 4350
Connection ~ 6850 4250
Wire Wire Line
	6850 4250 6900 4250
Wire Wire Line
	7150 4350 7150 4250
Connection ~ 7150 4250
Wire Wire Line
	7150 4250 7650 4250
Wire Wire Line
	6350 4250 6850 4250
$Comp
L power:GND #PWR0134
U 1 1 6249696C
P 6850 4650
F 0 "#PWR0134" H 6850 4400 50  0001 C CNN
F 1 "GND" H 6855 4477 50  0000 C CNN
F 2 "" H 6850 4650 50  0001 C CNN
F 3 "" H 6850 4650 50  0001 C CNN
	1    6850 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 4550 6850 4600
Wire Wire Line
	7150 4550 7150 4600
Wire Wire Line
	7150 4600 6850 4600
Connection ~ 6850 4600
Wire Wire Line
	6850 4600 6850 4650
Wire Wire Line
	3450 3800 2950 3800
Wire Wire Line
	3450 4000 2950 4000
Wire Wire Line
	3450 4100 2950 4100
Wire Wire Line
	3450 4200 2950 4200
Wire Wire Line
	3450 4300 2950 4300
Wire Wire Line
	3450 4400 2950 4400
Wire Wire Line
	3450 4500 2950 4500
Wire Wire Line
	3450 4600 2950 4600
Wire Wire Line
	3450 4700 2950 4700
Wire Wire Line
	3450 4800 2950 4800
Wire Wire Line
	3450 4900 2950 4900
Wire Wire Line
	3450 5000 2950 5000
Wire Wire Line
	3450 5100 2950 5100
Wire Wire Line
	3450 5200 2950 5200
Wire Wire Line
	3450 5300 2950 5300
$Sheet
S 9700 4800 1200 550 
U 62731E6D
F0 "connectors" 50
F1 "connectors.sch" 50
$EndSheet
$Sheet
S 9700 5650 1200 550 
U 6146CE41
F0 "power" 50
F1 "power.sch" 50
$EndSheet
Text GLabel 5350 3500 2    50   Input ~ 0
USB_D-
Text GLabel 5350 3600 2    50   Input ~ 0
USB_D+
Wire Wire Line
	4850 3500 5350 3500
Wire Wire Line
	4850 3600 5350 3600
Text GLabel 5350 3700 2    50   Input ~ 0
SWDIO
Text GLabel 5350 3800 2    50   Input ~ 0
SWCLK
Wire Wire Line
	4850 3700 5350 3700
Wire Wire Line
	4850 3800 5350 3800
Text GLabel 5350 4400 2    50   Input ~ 0
SWO
Wire Wire Line
	4850 4400 5350 4400
Text GLabel 2950 2400 0    50   Input ~ 0
NRST
Wire Wire Line
	2950 2400 3450 2400
$Comp
L Mechanical:MountingHole_Pad H1
U 1 1 61725F60
P 900 6100
F 0 "H1" V 1137 6103 50  0000 C CNN
F 1 "MountingHole_Pad" V 1046 6103 50  0000 C CNN
F 2 "MountingHole:MountingHole_2.2mm_M2_Pad_Via" H 900 6100 50  0001 C CNN
F 3 "~" H 900 6100 50  0001 C CNN
F 4 "-" H 900 6100 50  0001 C CNN "LCSC part #"
	1    900  6100
	0    -1   -1   0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 61726FEF
P 900 6550
F 0 "H2" V 1137 6553 50  0000 C CNN
F 1 "MountingHole_Pad" V 1046 6553 50  0000 C CNN
F 2 "MountingHole:MountingHole_2.2mm_M2_Pad_Via" H 900 6550 50  0001 C CNN
F 3 "~" H 900 6550 50  0001 C CNN
F 4 "-" H 900 6550 50  0001 C CNN "LCSC part #"
	1    900  6550
	0    -1   -1   0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H3
U 1 1 6172D7BE
P 900 7000
F 0 "H3" V 1137 7003 50  0000 C CNN
F 1 "MountingHole_Pad" V 1046 7003 50  0000 C CNN
F 2 "MountingHole:MountingHole_2.2mm_M2_Pad_Via" H 900 7000 50  0001 C CNN
F 3 "~" H 900 7000 50  0001 C CNN
F 4 "-" H 900 7000 50  0001 C CNN "LCSC part #"
	1    900  7000
	0    -1   -1   0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H4
U 1 1 61733DA5
P 900 7450
F 0 "H4" V 1137 7453 50  0000 C CNN
F 1 "MountingHole_Pad" V 1046 7453 50  0000 C CNN
F 2 "MountingHole:MountingHole_2.2mm_M2_Pad_Via" H 900 7450 50  0001 C CNN
F 3 "~" H 900 7450 50  0001 C CNN
F 4 "-" H 900 7450 50  0001 C CNN "LCSC part #"
	1    900  7450
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0114
U 1 1 61740C27
P 1250 7500
F 0 "#PWR0114" H 1250 7250 50  0001 C CNN
F 1 "GND" H 1255 7327 50  0000 C CNN
F 2 "" H 1250 7500 50  0001 C CNN
F 3 "" H 1250 7500 50  0001 C CNN
	1    1250 7500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 7450 1250 7450
Wire Wire Line
	1250 7450 1250 7500
Wire Wire Line
	1000 6100 1250 6100
Wire Wire Line
	1250 6100 1250 6550
Connection ~ 1250 7450
Wire Wire Line
	1000 7000 1250 7000
Connection ~ 1250 7000
Wire Wire Line
	1250 7000 1250 7450
Wire Wire Line
	1000 6550 1250 6550
Connection ~ 1250 6550
Wire Wire Line
	1250 6550 1250 7000
Text GLabel 2950 4200 0    50   Input ~ 0
PC2
Text GLabel 2950 4300 0    50   Input ~ 0
PC3
Text GLabel 2950 3800 0    50   Input ~ 0
PD2
Text GLabel 2950 4000 0    50   Input ~ 0
PC0
Text GLabel 2950 4100 0    50   Input ~ 0
PC1
Text GLabel 2950 4400 0    50   Input ~ 0
PC4
Text GLabel 2950 4500 0    50   Input ~ 0
PC5
Text GLabel 2950 4800 0    50   Input ~ 0
PC8
Text GLabel 2950 4900 0    50   Input ~ 0
PC9
Text GLabel 2950 5000 0    50   Input ~ 0
PC10
Text GLabel 2950 5100 0    50   Input ~ 0
PC11
Text GLabel 2950 5200 0    50   Input ~ 0
PC12
Text GLabel 2950 5300 0    50   Input ~ 0
PC13
Text GLabel 2950 4600 0    50   Input ~ 0
FLASH_!HOLD
Text GLabel 2950 4700 0    50   Input ~ 0
FLASH_!WP
Text GLabel 5350 5200 2    50   Input ~ 0
SPI2_!CS
Text GLabel 5350 5300 2    50   Input ~ 0
SPI2_SCK
Text GLabel 5350 5400 2    50   Input ~ 0
SPI2_MISO
Text GLabel 5350 5500 2    50   Input ~ 0
SPI2_MOSI
$Comp
L Device:R_Small R4
U 1 1 617D9DCA
P 5850 4500
F 0 "R4" H 5750 4550 50  0000 C CNN
F 1 "10k" H 5700 4450 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 5850 4500 50  0001 C CNN
F 3 "~" H 5850 4500 50  0001 C CNN
F 4 "C17902" H 5850 4500 50  0001 C CNN "LCSC part #"
	1    5850 4500
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 617DA6B8
P 5850 4700
F 0 "#PWR0115" H 5850 4450 50  0001 C CNN
F 1 "GND" H 5855 4527 50  0000 C CNN
F 2 "" H 5850 4700 50  0001 C CNN
F 3 "" H 5850 4700 50  0001 C CNN
	1    5850 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 4300 5850 4400
Wire Wire Line
	5850 4600 5850 4700
Wire Wire Line
	4850 4300 5850 4300
Text GLabel 5350 4900 2    50   Input ~ 0
I2C1_SCL
Text GLabel 5350 5000 2    50   Input ~ 0
I2C1_SDA
Text GLabel 6750 5200 0    50   Input ~ 0
I2C1_SCL
Text GLabel 6750 5100 0    50   Input ~ 0
I2C1_SDA
$Comp
L Device:R_Small R5
U 1 1 61831547
P 6850 5350
F 0 "R5" H 6750 5400 50  0000 C CNN
F 1 "2.2k" H 6700 5300 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 6850 5350 50  0001 C CNN
F 3 "~" H 6850 5350 50  0001 C CNN
F 4 "C17520" H 6850 5350 50  0001 C CNN "LCSC part #"
	1    6850 5350
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_Small R6
U 1 1 61831D2A
P 7150 5350
F 0 "R6" H 7050 5400 50  0000 C CNN
F 1 "2.2k" H 7000 5300 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 7150 5350 50  0001 C CNN
F 3 "~" H 7150 5350 50  0001 C CNN
F 4 "C17520" H 7150 5350 50  0001 C CNN "LCSC part #"
	1    7150 5350
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6750 5200 6850 5200
Wire Wire Line
	6850 5200 6850 5250
Wire Wire Line
	6750 5100 7150 5100
Wire Wire Line
	7150 5100 7150 5250
$Comp
L power:GND #PWR0116
U 1 1 6184137F
P 6850 5550
F 0 "#PWR0116" H 6850 5300 50  0001 C CNN
F 1 "GND" H 6855 5377 50  0000 C CNN
F 2 "" H 6850 5550 50  0001 C CNN
F 3 "" H 6850 5550 50  0001 C CNN
	1    6850 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 5450 6850 5500
Wire Wire Line
	7150 5450 7150 5500
Wire Wire Line
	7150 5500 6850 5500
Connection ~ 6850 5500
Wire Wire Line
	6850 5550 6850 5500
Text GLabel 5350 2400 2    50   Input ~ 0
PA0
Text GLabel 5350 2500 2    50   Input ~ 0
PA1
Text GLabel 5350 2700 2    50   Input ~ 0
PA3
Text GLabel 5350 2800 2    50   Input ~ 0
PA4
Text GLabel 5350 2900 2    50   Input ~ 0
PA5
Text GLabel 5350 3000 2    50   Input ~ 0
PA6
Text GLabel 5350 3100 2    50   Input ~ 0
PA7
Text GLabel 5350 3200 2    50   Input ~ 0
PA8
Text GLabel 5350 3300 2    50   Input ~ 0
PA9
Text GLabel 5350 3400 2    50   Input ~ 0
PA10
Text GLabel 5350 3900 2    50   Input ~ 0
PA15
Text GLabel 5350 4100 2    50   Input ~ 0
PB0
Text GLabel 5350 4200 2    50   Input ~ 0
PB1
Text GLabel 5350 4500 2    50   Input ~ 0
PB4
Text GLabel 5350 4600 2    50   Input ~ 0
PB5
Text GLabel 5350 4700 2    50   Input ~ 0
PB6
Text GLabel 5350 4800 2    50   Input ~ 0
PB7
Text GLabel 5350 5100 2    50   Input ~ 0
PB10
$Comp
L max_KiCAD_lib:W25N01GVZEIG_TR U3
U 1 1 61480725
P 8450 1450
F 0 "U3" H 8450 2120 50  0000 C CNN
F 1 "W25N01GVZEIG_TR" H 8450 2029 50  0000 C CNN
F 2 "max_KiCAD_lib:SON127P800X600X80-9N" H 8450 1450 50  0001 L BNN
F 3 "" H 8450 1450 50  0001 L BNN
F 4 "None" H 8450 1450 50  0001 L BNN "PRICE"
F 5 "IPC7351B" H 8450 1450 50  0001 L BNN "STANDARD"
F 6 "WSON-8 Winbond" H 8450 1450 50  0001 L BNN "PACKAGE"
F 7 "Winbond Electronics" H 8450 1450 50  0001 L BNN "MANUFACTURER"
F 8 "Ic Flash 1gbit 104mhz 8wson" H 8450 1450 50  0001 L BNN "DESCRIPTION"
F 9 "W25N01GVZEIG TR" H 8450 1450 50  0001 L BNN "MP"
F 10 "C88868" H 8450 1450 50  0001 C CNN "LCSC part #"
	1    8450 1450
	1    0    0    -1  
$EndComp
Text GLabel 7450 1250 0    50   Input ~ 0
SPI2_!CS
Text GLabel 7450 1350 0    50   Input ~ 0
SPI2_SCK
Text GLabel 7450 1650 0    50   Input ~ 0
SPI2_MISO
Text GLabel 7450 1550 0    50   Input ~ 0
SPI2_MOSI
Wire Wire Line
	7450 1650 7550 1650
Wire Wire Line
	7450 1250 7550 1250
Wire Wire Line
	7450 1350 7550 1350
Wire Wire Line
	7450 1550 7550 1550
$Comp
L power:GND #PWR0138
U 1 1 614C487E
P 9400 1900
F 0 "#PWR0138" H 9400 1650 50  0001 C CNN
F 1 "GND" H 9405 1727 50  0000 C CNN
F 2 "" H 9400 1900 50  0001 C CNN
F 3 "" H 9400 1900 50  0001 C CNN
	1    9400 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9350 1850 9400 1850
Wire Wire Line
	9400 1850 9400 1900
$Comp
L power:+3.3V #PWR0139
U 1 1 614CDE88
P 9400 1000
F 0 "#PWR0139" H 9400 850 50  0001 C CNN
F 1 "+3.3V" H 9415 1173 50  0000 C CNN
F 2 "" H 9400 1000 50  0001 C CNN
F 3 "" H 9400 1000 50  0001 C CNN
	1    9400 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9350 1050 9400 1050
Wire Wire Line
	9400 1050 9400 1000
$Comp
L Device:C_Small C20
U 1 1 614D74C0
P 9400 1150
F 0 "C20" H 9492 1196 50  0000 L CNN
F 1 "100n" H 9492 1105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 9400 1150 50  0001 C CNN
F 3 "~" H 9400 1150 50  0001 C CNN
F 4 "C14663" H 9400 1150 50  0001 C CNN "LCSC part #"
	1    9400 1150
	1    0    0    -1  
$EndComp
Connection ~ 9400 1050
$Comp
L power:GND #PWR0140
U 1 1 614D7DB2
P 9400 1300
F 0 "#PWR0140" H 9400 1050 50  0001 C CNN
F 1 "GND" H 9405 1127 50  0000 C CNN
F 2 "" H 9400 1300 50  0001 C CNN
F 3 "" H 9400 1300 50  0001 C CNN
	1    9400 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 1250 9400 1300
Text GLabel 10400 1550 2    50   Input ~ 0
FLASH_!HOLD
Text GLabel 10400 1650 2    50   Input ~ 0
FLASH_!WP
Wire Wire Line
	9350 1550 9850 1550
Wire Wire Line
	10400 1650 10150 1650
$Comp
L power:+3.3V #PWR0141
U 1 1 61530514
P 9850 1150
F 0 "#PWR0141" H 9850 1000 50  0001 C CNN
F 1 "+3.3V" H 9865 1323 50  0000 C CNN
F 2 "" H 9850 1150 50  0001 C CNN
F 3 "" H 9850 1150 50  0001 C CNN
	1    9850 1150
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0142
U 1 1 615309F9
P 10150 1150
F 0 "#PWR0142" H 10150 1000 50  0001 C CNN
F 1 "+3.3V" H 10165 1323 50  0000 C CNN
F 2 "" H 10150 1150 50  0001 C CNN
F 3 "" H 10150 1150 50  0001 C CNN
	1    10150 1150
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R18
U 1 1 61530C68
P 9850 1300
F 0 "R18" H 9950 1200 50  0000 C CNN
F 1 "10k" H 9950 1300 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 9850 1300 50  0001 C CNN
F 3 "~" H 9850 1300 50  0001 C CNN
F 4 "C17902" H 9850 1300 50  0001 C CNN "LCSC part #"
	1    9850 1300
	1    0    0    1   
$EndComp
Wire Wire Line
	9850 1150 9850 1200
Wire Wire Line
	9850 1400 9850 1550
Connection ~ 9850 1550
Wire Wire Line
	9850 1550 10400 1550
$Comp
L Device:R_Small R19
U 1 1 6154C7CE
P 10150 1300
F 0 "R19" H 10250 1200 50  0000 C CNN
F 1 "10k" H 10250 1300 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 10150 1300 50  0001 C CNN
F 3 "~" H 10150 1300 50  0001 C CNN
F 4 "C17902" H 10150 1300 50  0001 C CNN "LCSC part #"
	1    10150 1300
	1    0    0    1   
$EndComp
Wire Wire Line
	10150 1150 10150 1200
Wire Wire Line
	10150 1400 10150 1650
Connection ~ 10150 1650
Wire Wire Line
	10150 1650 9350 1650
$EndSCHEMATC
