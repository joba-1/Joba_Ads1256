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
L Device:Thermistor_NTC TH1
U 1 1 62C2DA65
P 8700 5600
F 0 "TH1" V 8410 5600 50  0000 C CNN
F 1 "NTC" V 8501 5600 50  0000 C CNN
F 2 "" H 8700 5650 50  0001 C CNN
F 3 "~" H 8700 5650 50  0001 C CNN
	1    8700 5600
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 62C2E35B
P 9950 5000
F 0 "#PWR?" H 9950 4850 50  0001 C CNN
F 1 "+5V" H 9965 5173 50  0000 C CNN
F 2 "" H 9950 5000 50  0001 C CNN
F 3 "" H 9950 5000 50  0001 C CNN
	1    9950 5000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 62C2E9AD
P 9500 5150
F 0 "R2" V 9707 5150 50  0000 C CNN
F 1 "10000" V 9616 5150 50  0000 C CNN
F 2 "" V 9430 5150 50  0001 C CNN
F 3 "~" H 9500 5150 50  0001 C CNN
	1    9500 5150
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R1
U 1 1 62C2EE3F
P 8700 5150
F 0 "R1" V 8907 5150 50  0000 C CNN
F 1 "10000" V 8816 5150 50  0000 C CNN
F 2 "" V 8630 5150 50  0001 C CNN
F 3 "~" H 8700 5150 50  0001 C CNN
	1    8700 5150
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 62C2F305
P 8000 5300
F 0 "#PWR?" H 8000 5050 50  0001 C CNN
F 1 "GND" H 8005 5127 50  0000 C CNN
F 2 "" H 8000 5300 50  0001 C CNN
F 3 "" H 8000 5300 50  0001 C CNN
	1    8000 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 5150 8300 5150
Wire Wire Line
	8000 5150 8000 5300
Text GLabel 9100 4550 1    50   Input ~ 0
Ain
$Comp
L Device:R R3
U 1 1 62C33839
P 9100 4800
F 0 "R3" H 9030 4754 50  0000 R CNN
F 1 "100" H 9030 4845 50  0000 R CNN
F 2 "" V 9030 4800 50  0001 C CNN
F 3 "~" H 9100 4800 50  0001 C CNN
	1    9100 4800
	-1   0    0    1   
$EndComp
Wire Wire Line
	9100 4550 9100 4650
Wire Wire Line
	9100 4950 9100 5150
Wire Wire Line
	9100 5150 8850 5150
Wire Wire Line
	9350 5150 9100 5150
Connection ~ 9100 5150
Wire Wire Line
	9650 5150 9950 5150
Wire Wire Line
	9950 5150 9950 5000
Wire Wire Line
	9100 5150 9100 5600
Wire Wire Line
	9100 5600 8850 5600
Wire Wire Line
	8550 5600 8300 5600
Wire Wire Line
	8300 5600 8300 5150
Connection ~ 8300 5150
Wire Wire Line
	8300 5150 8000 5150
$EndSCHEMATC
