### Brief summary

A l'ordinador arriba RPY del gripper i de endowrist.
Als servos només arriba senyal del gripper -> el servos ha de mesurar el torque que fa gripper i poder modifcar el gripper


### To do before 30/10

1. Correcció RPY als sevos desde 90º (write 90 + roll) ✅
2. "Fer que es guardi la última RPY i es mogui en funció d'aquesta" 
3. Fer que el servos calculi Torque i l'envii al PC i al gripper ✅
4. Modificar codi de gripper per "vibrar" en funció del torque (codi del md de LabSession3&4) ✅
5. Afegir torques al TKinter
6. Afegir un botó al Tkinter que canvii de color en funció del torque


### To do before 6/11

1. Correcció angles pitch i roll (no 340 --> -20)
2. Correcció angle yaw
3. Modificació tkinter per veure els torques
4. Modificació tkinter per afegir un botó de canvi de color


### Trial 6/11

**Part 1: Comprovar ús dels sensors**
1. Executar main.cpp
    - *Endowrist_IMU_V1*
    - *Gripper_IMU_V2_recieveTorque_vibrate*
    - *Servos_V3_20251101_002 + V1* --> LÍNIA DE CODI V1: 152-157
2. Executar src/roboDK/Init_SurgeryRobotics_simulation_V2.py --> V2 sabem que funciona
3. Comprovar:
    - Pitch i roll no 340 --> -20 --> LÍNIA DE CODI: 127-131 + 149-150
    - Yaw --> ens agrada? 
4. Executar main.cpp 
    - *Servos_V3_20251101_002 + V2* --> LÍNIA DE CODI V2: 159-169
5. Executar src/roboDK/Init_SurgeryRobotics_simulation_V2.py --> V2 sabem que funciona
6. Comprovar:
    - Yaw --> ens agrada?
7. Executar main.cpp 
    - *Servos_V3_20251101_001 + V3* --> LÍNIA DE CODI V3: 170-177
8. Executar src/roboDK/Init_SurgeryRobotics_simulation_V2.py --> V2 sabem que funciona
9. Comprovar:
    - Yaw --> ens agrada?
10. Escollir V1-V2-V3 en *Servos_V3_20251101_002*

**Part 2: Comprovar simulació**
1. Executar src/roboDK/Init_SurgeryRobotics_simulation_V3.py --> La laia el va editar, hauria de funcionar
2. Comprovar que el PopUp imprimeix torques
3. Executar src/roboDK/Init_SurgeryRobotics_simulation_V4_20251102_inputval.py --> V3 modificat perquè es vegi el botó de colors
4. Comprovar que el botó funciona