Model im

REM *****************************************:
REM * Common entries:
REM *****************************************:

REM Setting the simulation time step...
rtds_write 0x00000000 0x00000050

REM Module block enable
rtds_write 0x00000003 0x00010001
rtds_file_write 0x00820000 indm_v2_imem.txt
rtds_file_write 0x00810010 indm_v2_gprst.txt
rtds_write 0x00810000 0x00000004
rtds_write 0x00810001 0x00000006
rtds_write 0x00810400 0x40400000
rtds_write 0x00810401 0x00000000
rtds_write 0x00810402 0x3F7FF800
rtds_write 0x00810403 0xE6CDC000
rtds_write 0x00810404 0x38FCE70C
rtds_write 0x00810405 0x2F8FC000
rtds_write 0x00810406 0x37D988FB
rtds_write 0x00810407 0x3D63C000
rtds_write 0x00810408 0x3F7FFE47
rtds_write 0x00810409 0xCB6C0000
rtds_write 0x0081040A 0x350635A4
rtds_write 0x0081040B 0x633C4000
rtds_write 0x0081040C 0xAE049800
rtds_write 0x0081040D 0xC0B54000
rtds_write 0x0081040E 0x2CE41A21
rtds_write 0x0081040F 0x1E254000
rtds_write 0x00810410 0xB5063749
rtds_write 0x00810411 0xA0008000
rtds_write 0x00810412 0x41962F71
rtds_write 0x00810413 0xF4A78000
rtds_write 0x00810414 0xC1946EFD
rtds_write 0x00810415 0x770A4000
rtds_write 0x00810416 0x41962F71
rtds_write 0x00810417 0xF4A78000
rtds_write 0x00810418 0x3FC00000
rtds_write 0x00810419 0x00000000
rtds_write 0x0081041A 0x3F000000
rtds_write 0x0081041B 0x00000000
rtds_write 0x00800001 5e-07
rtds_write 0x00800011 0.00018518518518518518
rtds_write 0x00800012 0.5
rtds_write 0x00800017 1.0
rtds_write 0x00800005 0x00000000
rtds_write 0x00800020 0x00000000
rtds_write 0x0080001E 0.0
rtds_write 0x0080001F 2.0
rtds_write 0x00800013 0.006283185307179587
rtds_write 0x00800014 1000.0
rtds_write 0x00800015 0x000003E8
rtds_write 0x00800016 0.001
rtds_write 0x00800021 0x00000010
rtds_write 0x00800023 0x00000000
rtds_write 0x00800022 0x00000001
rtds_write 0x0080002B 1.0
rtds_write 0x0080002C 0.015707963267948963
rtds_write 0x0080002D 0x00000000
rtds_write 0x0080002E 0x00000000
rtds_write 0x00800026 0.0
rtds_write 0x00800027 1.0

REM LUT solver inputs...
rtds_write 0x01000000 0x00000000

REM HSSL configuration files...

REM Parallel DTV configuration...


REM *****************************************:
REM * SPC0 entries:
REM *****************************************:

REM SPC0 Topology Selector (TS) initialization...
rtds_file_write 0x08180000 SPC0_red_table.txt

rtds_write 0x08100020 0x00000004
rtds_write 0x08100021 0x00000003
rtds_write 0x08100023 0x00000000
rtds_write 0x08100024 0x00000000
rtds_write 0x08100025 0x00000000
rtds_write 0x08100026 0x00000000
rtds_write 0x08100027 0x00000000
rtds_write 0x08100030 0x00000000
rtds_write 0x08100031 0x00000000
rtds_write 0x08100032 0x00000000
rtds_write 0x08100033 0x00000000
rtds_write 0x08100034 0x00000001
rtds_write 0x08100035 0x00000001
rtds_write 0x08100036 0x00000000
rtds_write 0x08100037 0x00000000
rtds_write 0x08100038 0x00000000
rtds_write 0x08100039 0x00000000
rtds_write 0x0810003A 0x00000000
rtds_write 0x0810003B 0x00000000
rtds_file_write 0x08140000 igbt_leg_imem.txt
rtds_file_write 0x08142000 igbt_leg_lut.txt
rtds_write 0x08100040 0x00000004
rtds_write 0x08100041 0x00000003
rtds_write 0x08100043 0x00000000
rtds_write 0x08100044 0x00000000
rtds_write 0x08100045 0x00000000
rtds_write 0x08100046 0x00000000
rtds_write 0x08100047 0x00000000
rtds_write 0x08100050 0x00000000
rtds_write 0x08100051 0x00000000
rtds_write 0x08100052 0x00000000
rtds_write 0x08100053 0x00000000
rtds_write 0x08100054 0x00000001
rtds_write 0x08100055 0x00000001
rtds_write 0x08100056 0x00000000
rtds_write 0x08100057 0x00000000
rtds_write 0x08100058 0x00000000
rtds_write 0x08100059 0x00000000
rtds_write 0x0810005A 0x00000000
rtds_write 0x0810005B 0x00000000
rtds_file_write 0x08148000 igbt_leg_imem.txt
rtds_file_write 0x0814A000 igbt_leg_lut.txt
rtds_write 0x08100060 0x00000004
rtds_write 0x08100061 0x00000003
rtds_write 0x08100063 0x00000000
rtds_write 0x08100064 0x00000000
rtds_write 0x08100065 0x00000000
rtds_write 0x08100066 0x00000000
rtds_write 0x08100067 0x00000000
rtds_write 0x08100070 0x00000000
rtds_write 0x08100071 0x00000000
rtds_write 0x08100072 0x00000000
rtds_write 0x08100073 0x00000000
rtds_write 0x08100074 0x00000001
rtds_write 0x08100075 0x00000001
rtds_write 0x08100076 0x00000000
rtds_write 0x08100077 0x00000000
rtds_write 0x08100078 0x00000000
rtds_write 0x08100079 0x00000000
rtds_write 0x0810007A 0x00000000
rtds_write 0x0810007B 0x00000000
rtds_file_write 0x08150000 igbt_leg_imem.txt
rtds_file_write 0x08152000 igbt_leg_lut.txt

REM SPC0 Variable Delay initialization...
rtds_write 0x08100001 0x0

REM SPC0 Output voltage compare mode...
rtds_write 0x08100005 0x00000000

REM SPC0 Matrix multiplier initialization...
rtds_file_write 0x08000000 SPC0_Com_Word.txt
rtds_file_write 0x08020000 SPC0_Com_LUT.txt
rtds_file_write 0x08080000 SPC0_MAC0.txt
rtds_file_write 0x08082000 SPC0_MAC1.txt
rtds_file_write 0x08084000 SPC0_MAC2.txt
rtds_file_write 0x08086000 SPC0_MAC3.txt

rtds_write 0x08100004 0x00000000
REM SPC0 Contactors initialization...

REM SPC0 GDS compensation settings...
rtds_write 0x080C0000 0x00000001
rtds_write 0x080C0001 0x00000003
rtds_write 0x080C0004 0x3C4CCCCC
rtds_write 0x080C0005 0xCCCD0000
rtds_write 0x08100000 0x00000050

REM SPC0 FSM digital input pin assignments...
rtds_write 0x08100028 0x00000000
rtds_write 0x08100029 0x00000000
rtds_write 0x0810002A 0x00000000
rtds_write 0x0810002B 0x00000000
rtds_write 0x0810002C 0x00000000
rtds_write 0x0810002D 0x00000001
rtds_write 0x08100022 0x00000003
rtds_write 0x08100048 0x00000000
rtds_write 0x08100049 0x00000000
rtds_write 0x0810004A 0x00000000
rtds_write 0x0810004B 0x00000000
rtds_write 0x0810004C 0x00000002
rtds_write 0x0810004D 0x00000003
rtds_write 0x08100042 0x00000003
rtds_write 0x08100068 0x00000000
rtds_write 0x08100069 0x00000000
rtds_write 0x0810006A 0x00000000
rtds_write 0x0810006B 0x00000000
rtds_write 0x0810006C 0x00000004
rtds_write 0x0810006D 0x00000005
rtds_write 0x08100062 0x00000003

REM SPC0 Comparators initialization...

REM SPC0 DTSM initialization...

REM *****************************************:
REM * SPC1 entries:
REM *****************************************:

REM SPC1 Topology Selector (TS) initialization...
rtds_file_write 0x08580000 SPC1_red_table.txt

rtds_write 0x08500020 0x00000000
rtds_write 0x08500021 0x00000000
rtds_write 0x08500023 0x00000000
rtds_write 0x08500024 0x00000000
rtds_write 0x08500025 0x00000000
rtds_write 0x08500026 0x00000000
rtds_write 0x08500027 0x00000000
rtds_write 0x08500030 0x00000000
rtds_write 0x08500031 0x00000000
rtds_write 0x08500032 0x00000000
rtds_write 0x08500033 0x00000000
rtds_write 0x08500034 0x00000000
rtds_write 0x08500035 0x00000000
rtds_write 0x08500036 0x00000000
rtds_write 0x08500037 0x00000000
rtds_write 0x08500038 0x00000000
rtds_write 0x08500039 0x00000000
rtds_write 0x0850003A 0x00000000
rtds_write 0x0850003B 0x00000000
rtds_file_write 0x08540000 trivial_imem.txt
rtds_file_write 0x08542000 trivial_lut.txt
rtds_write 0x08500040 0x00000000
rtds_write 0x08500041 0x00000000
rtds_write 0x08500043 0x00000000
rtds_write 0x08500044 0x00000000
rtds_write 0x08500045 0x00000000
rtds_write 0x08500046 0x00000000
rtds_write 0x08500047 0x00000000
rtds_write 0x08500050 0x00000000
rtds_write 0x08500051 0x00000000
rtds_write 0x08500052 0x00000000
rtds_write 0x08500053 0x00000000
rtds_write 0x08500054 0x00000000
rtds_write 0x08500055 0x00000000
rtds_write 0x08500056 0x00000000
rtds_write 0x08500057 0x00000000
rtds_write 0x08500058 0x00000000
rtds_write 0x08500059 0x00000000
rtds_write 0x0850005A 0x00000000
rtds_write 0x0850005B 0x00000000
rtds_file_write 0x08548000 trivial_imem.txt
rtds_file_write 0x0854A000 trivial_lut.txt
rtds_write 0x08500060 0x00000000
rtds_write 0x08500061 0x00000000
rtds_write 0x08500063 0x00000000
rtds_write 0x08500064 0x00000000
rtds_write 0x08500065 0x00000000
rtds_write 0x08500066 0x00000000
rtds_write 0x08500067 0x00000000
rtds_write 0x08500070 0x00000000
rtds_write 0x08500071 0x00000000
rtds_write 0x08500072 0x00000000
rtds_write 0x08500073 0x00000000
rtds_write 0x08500074 0x00000000
rtds_write 0x08500075 0x00000000
rtds_write 0x08500076 0x00000000
rtds_write 0x08500077 0x00000000
rtds_write 0x08500078 0x00000000
rtds_write 0x08500079 0x00000000
rtds_write 0x0850007A 0x00000000
rtds_write 0x0850007B 0x00000000
rtds_file_write 0x08550000 trivial_imem.txt
rtds_file_write 0x08552000 trivial_lut.txt

REM SPC1 Variable Delay initialization...
rtds_write 0x08500001 0x0

REM SPC1 Output voltage compare mode...
rtds_write 0x08500005 0x00000000

REM SPC1 Matrix multiplier initialization...
rtds_file_write 0x08400000 SPC1_Com_Word.txt
rtds_file_write 0x08420000 SPC1_Com_LUT.txt
rtds_file_write 0x08480000 SPC1_MAC0.txt
rtds_file_write 0x08482000 SPC1_MAC1.txt
rtds_file_write 0x08484000 SPC1_MAC2.txt
rtds_file_write 0x08486000 SPC1_MAC3.txt

rtds_write 0x08500004 0x00000000
REM SPC1 Contactors initialization...

REM SPC1 GDS compensation settings...
rtds_write 0x084C0000 0x00000000
rtds_write 0x084C0001 0x00000000
rtds_write 0x084C0004 0x00000000
rtds_write 0x084C0005 0x00000000
rtds_write 0x08500000 0x00000000

REM SPC1 FSM digital input pin assignments...

REM SPC1 Comparators initialization...

REM SPC1 DTSM initialization...

REM *****************************************:
REM * SPC2 entries:
REM *****************************************:

REM SPC2 Topology Selector (TS) initialization...
rtds_file_write 0x08980000 SPC2_red_table.txt

rtds_write 0x08900020 0x00000000
rtds_write 0x08900021 0x00000000
rtds_write 0x08900023 0x00000000
rtds_write 0x08900024 0x00000000
rtds_write 0x08900025 0x00000000
rtds_write 0x08900026 0x00000000
rtds_write 0x08900027 0x00000000
rtds_write 0x08900030 0x00000000
rtds_write 0x08900031 0x00000000
rtds_write 0x08900032 0x00000000
rtds_write 0x08900033 0x00000000
rtds_write 0x08900034 0x00000000
rtds_write 0x08900035 0x00000000
rtds_write 0x08900036 0x00000000
rtds_write 0x08900037 0x00000000
rtds_write 0x08900038 0x00000000
rtds_write 0x08900039 0x00000000
rtds_write 0x0890003A 0x00000000
rtds_write 0x0890003B 0x00000000
rtds_file_write 0x08940000 trivial_imem.txt
rtds_file_write 0x08942000 trivial_lut.txt
rtds_write 0x08900040 0x00000000
rtds_write 0x08900041 0x00000000
rtds_write 0x08900043 0x00000000
rtds_write 0x08900044 0x00000000
rtds_write 0x08900045 0x00000000
rtds_write 0x08900046 0x00000000
rtds_write 0x08900047 0x00000000
rtds_write 0x08900050 0x00000000
rtds_write 0x08900051 0x00000000
rtds_write 0x08900052 0x00000000
rtds_write 0x08900053 0x00000000
rtds_write 0x08900054 0x00000000
rtds_write 0x08900055 0x00000000
rtds_write 0x08900056 0x00000000
rtds_write 0x08900057 0x00000000
rtds_write 0x08900058 0x00000000
rtds_write 0x08900059 0x00000000
rtds_write 0x0890005A 0x00000000
rtds_write 0x0890005B 0x00000000
rtds_file_write 0x08948000 trivial_imem.txt
rtds_file_write 0x0894A000 trivial_lut.txt
rtds_write 0x08900060 0x00000000
rtds_write 0x08900061 0x00000000
rtds_write 0x08900063 0x00000000
rtds_write 0x08900064 0x00000000
rtds_write 0x08900065 0x00000000
rtds_write 0x08900066 0x00000000
rtds_write 0x08900067 0x00000000
rtds_write 0x08900070 0x00000000
rtds_write 0x08900071 0x00000000
rtds_write 0x08900072 0x00000000
rtds_write 0x08900073 0x00000000
rtds_write 0x08900074 0x00000000
rtds_write 0x08900075 0x00000000
rtds_write 0x08900076 0x00000000
rtds_write 0x08900077 0x00000000
rtds_write 0x08900078 0x00000000
rtds_write 0x08900079 0x00000000
rtds_write 0x0890007A 0x00000000
rtds_write 0x0890007B 0x00000000
rtds_file_write 0x08950000 trivial_imem.txt
rtds_file_write 0x08952000 trivial_lut.txt

REM SPC2 Variable Delay initialization...
rtds_write 0x08900001 0x0

REM SPC2 Output voltage compare mode...
rtds_write 0x08900005 0x00000000

REM SPC2 Matrix multiplier initialization...
rtds_file_write 0x08800000 SPC2_Com_Word.txt
rtds_file_write 0x08820000 SPC2_Com_LUT.txt
rtds_file_write 0x08880000 SPC2_MAC0.txt
rtds_file_write 0x08882000 SPC2_MAC1.txt
rtds_file_write 0x08884000 SPC2_MAC2.txt
rtds_file_write 0x08886000 SPC2_MAC3.txt

rtds_write 0x08900004 0x00000000
REM SPC2 Contactors initialization...

REM SPC2 GDS compensation settings...
rtds_write 0x088C0000 0x00000000
rtds_write 0x088C0001 0x00000000
rtds_write 0x088C0004 0x00000000
rtds_write 0x088C0005 0x00000000
rtds_write 0x08900000 0x00000000

REM SPC2 FSM digital input pin assignments...

REM SPC2 Comparators initialization...

REM SPC2 DTSM initialization...

*****************************************:


REM SP data configuration...
*****************************************:


REM Setting the capture sample step...


REM post SP Init calculation...
rtds_write  
rtds_write 0x00000041 0x000011C1
rtds_write 0x00000005 0x00000000
rtds_write 0x00000043 0x00002710
rtds_write 0x00000042 0x000001F3
rtds_write 0x0000000A 0x00000000


REM CoProcessors uBlaze_1, uBlaze_2 and uBlaze_3 configuration
glbl_write 0x40800000 0x7
glbl_write 0xf8000008 0xdf0d


REM CoProcessor ARM_1 configuration...
glbl_write 0xfffffff0 0xffffff2c  
glbl_write 0xFFFFFF00 0xe3e0000f
glbl_write 0xFFFFFF04 0xe3a01000
glbl_write 0xFFFFFF08 0xe5801000
glbl_write 0xFFFFFF0C 0xe320f002
glbl_write 0xFFFFFF10 0xe5902000
glbl_write 0xFFFFFF14 0xe1520001
glbl_write 0xFFFFFF18 0x0afffffb
glbl_write 0xFFFFFF1C 0xe1a0f002
glbl_write 0x00000000 0xe3e0f0ff
glbl_write 0xf8000244 0x2
glbl_write 0xf8000244 0x22


REM special case for HIL402 for eth ve and SV


REM sys_command 0x0


REM disable can devices
sys_command 0x2


REM ifconfig eth0 up
sys_command 0x1


REM enable ETH0 Intr on Core0 CPU
glbl_write 0xF8F01834 0x01010101
glbl_write 0x40800000 0x7


REM Restart counter for collected Linux OS communication apps
app_file_write 0x0 app_init
rtds_write 0x00000027 0x00000050
rtds_write 0x00000040 0x00FFFFFF