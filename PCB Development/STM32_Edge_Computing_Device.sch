EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 4
Title "STM32 Edge Computing Device"
Date "2021-09-03"
Rev "1.1"
Comp "University of Patras"
Comment1 "Designer: Alysandratos Spyridon"
Comment2 ""
Comment3 "maintenance of stepper motor and realization of digital twin."
Comment4 "Module based PCB design around an STM32L4 μC for predictive"
$EndDescr
$Sheet
S 2000 4300 2400 1750
U 60F621F9
F0 "IO Connectors" 50
F1 "IO_Connectors.sch" 50
$EndSheet
$Sheet
S 5700 1850 2750 1450
U 60F620E5
F0 "MCU" 50
F1 "STM32L4.sch" 50
$EndSheet
Text Notes 1950 1700 0    197  ~ 39
Power
Text Notes 5650 1700 0    197  ~ 39
MCU
Text Notes 1950 4150 0    197  ~ 39
IO Connectors
Text Notes 3000 2700 0    197  ~ 39
1
Text Notes 6950 2700 0    197  ~ 39
2
Text Notes 3050 5300 0    197  ~ 39
3
Text Notes 6450 4450 0    79   ~ 16
Mounting Holes
$Comp
L Mechanical:MountingHole_Pad H1
U 1 1 611BA16A
P 6800 4650
F 0 "H1" V 6754 4800 50  0000 L CNN
F 1 "MountingHole_Pad" V 6845 4800 50  0000 L CNN
F 2 "MountingHole:MountingHole_3mm_Pad_Via" H 6800 4650 50  0001 C CNN
F 3 "~" H 6800 4650 50  0001 C CNN
	1    6800 4650
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 611BBA45
P 6800 4900
F 0 "H2" V 6754 5050 50  0000 L CNN
F 1 "MountingHole_Pad" V 6845 5050 50  0000 L CNN
F 2 "MountingHole:MountingHole_3mm_Pad_Via" H 6800 4900 50  0001 C CNN
F 3 "~" H 6800 4900 50  0001 C CNN
	1    6800 4900
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H3
U 1 1 611BBE58
P 6800 5150
F 0 "H3" V 6754 5300 50  0000 L CNN
F 1 "MountingHole_Pad" V 6845 5300 50  0000 L CNN
F 2 "MountingHole:MountingHole_3mm_Pad_Via" H 6800 5150 50  0001 C CNN
F 3 "~" H 6800 5150 50  0001 C CNN
	1    6800 5150
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H4
U 1 1 611BC19A
P 6800 5400
F 0 "H4" V 6754 5550 50  0000 L CNN
F 1 "MountingHole_Pad" V 6845 5550 50  0000 L CNN
F 2 "MountingHole:MountingHole_3mm_Pad_Via" H 6800 5400 50  0001 C CNN
F 3 "~" H 6800 5400 50  0001 C CNN
	1    6800 5400
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0163
U 1 1 611BCC45
P 6600 5500
F 0 "#PWR0163" H 6600 5250 50  0001 C CNN
F 1 "GND" H 6605 5327 50  0000 C CNN
F 2 "" H 6600 5500 50  0001 C CNN
F 3 "" H 6600 5500 50  0001 C CNN
	1    6600 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 5500 6600 5400
Wire Wire Line
	6600 4650 6700 4650
Wire Wire Line
	6700 4900 6600 4900
Connection ~ 6600 4900
Wire Wire Line
	6600 4900 6600 4650
Wire Wire Line
	6700 5150 6600 5150
Connection ~ 6600 5150
Wire Wire Line
	6600 5150 6600 4900
Wire Wire Line
	6700 5400 6600 5400
Connection ~ 6600 5400
Wire Wire Line
	6600 5400 6600 5150
Wire Notes Line
	6450 4500 6450 5750
Wire Notes Line
	6450 5750 7750 5750
Wire Notes Line
	7750 5750 7750 4500
Wire Notes Line
	7750 4500 6450 4500
$Sheet
S 2000 1850 2400 1450
U 60F6203D
F0 "Power" 50
F1 "Power.sch" 50
$EndSheet
$EndSCHEMATC
