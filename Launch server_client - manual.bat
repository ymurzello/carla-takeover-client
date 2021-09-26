@echo off

title launch CARLA

rem Start server
d:
cd "D:\Windows\CARLA\CARLA_0.9.12"
start CarlaUE4.exe
timeout 15
cd "D:\Windows\CARLA\CARLA_0.9.12\PythonAPI\util"
call python config.py -m Town04 

timeout 10
rem Launch client
call "C:\ProgramData\Anaconda3\Scripts\activate.bat" "C:\ProgramData\Anaconda3"
d:
cd "D:\Windows\CARLA\Location A"
start python synchronous_client_manual.py -sp test5.json -sc scenario_configs/bike.json -r True

timeout 10
rem Change weather
call "C:\ProgramData\Anaconda3\Scripts\activate.bat" "C:\ProgramData\Anaconda3"
d:
cd "D:\Windows\CARLA\CARLA_0.9.12\PythonAPI\util"
call python environment.py --clouds 10 --wind 25 --sun day --weather clear --fog 0.0

timeout 5
rem Launch npc
call "C:\ProgramData\Anaconda3\Scripts\activate.bat" "C:\ProgramData\Anaconda3"
d:
cd "D:\Windows\CARLA\Location A"
call python npc_manager.py -sp test5.json
