# Ardupilot_Dronekit

-----------------------------------------------------------------------------------------
# 리눅스용 윈도우 하위 시스템(Windows Subsystem for Linux, WSL) 활성화 필요

# Microsoft store에서 Ubuntu 설치 (Ubuntu 20.04 LTS)

# New Unix User등록 요청 시 그냥 창 닫기(New Username 등록 없이 root으로 사용 권장)

# Ubuntu 창 열기
----------------------------------------------------------------------------------------------

순서 

1. apt install python-is-python3

2. apt-get update

3. apt install python3-pip -y

4. pip install pymavlink && pip install mavproxy

5. git clone -b Copter-4.1 http://github.com/ardupilot/ardupilot

6. cd ardupilot

7. git submodule update --init --recursive

8. cd ardupilot/ArduCopter

## root/ardupilot/ArduCopter 안에 가상 Ardupilot compile
9. /root/ardupilot/Tools/autotest/sim_vehicle.py     

10. Compile 끝난 후 창 닫기 또는 exit (Ctrl + c)

12. 다시 Ubuntu 창 열기

# compile된 파일 위치로 가기
13. cd /root/ardupilot/build/sitl/bin

# /root/ardupilot/build/sitl/bin에서 가상 arudcopter 드론 실행
14. ./arducopter -S -I0 --home -35.363261,149.165230,584,353 --model "+" --speedup 1 --defaults /root/ardupilot/Tools/autotest/default_params/copter.parm

15. 다른 Ubuntu 창 열기 (복수의 Ubuntu 실행 가능)

# Telemetry forwarding/broadcasting, (--out udp:[IP]:[PORT])로 broadcasting 추가 가능
16. mavproxy.py --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 

17. Broadcasting된 IP & Port 중 하나에 MISSION PLANNER 연결, 다른 IP & Port에 MAVSDK 코드 연결하여 실행 및 테스트.
    예:port 14550에서 MISSION PlANNER 연결

18. 다시 실행 시 step 13부터 16까지 반복

19. 명령 간략화를 위해 alias 사용
    nano ~/.bashrc
    alias sitl='/root/ardupilot/build/sitl/bin/arducopter -S -I0 --home -35.363261,149.165230,584,353 --model "+" --speedup 1 --defaults /root/ardupilot/Tools/autotest/default_params/copter.parm'
    저장 후 
    source ~/.bashrc

20. sitl 명령으로 ardupilot 실행.



SITL(Software in the Loop) Installation(Windows)
- pip install dronekit
- pip install dronekit-sitl
- pip install mavproxy  WSL: (source ~/.bashrc)
- pip install -U wxPython
- pip install git+https://github.com/dronekit/dronekit-sitl

Steps: 
1. Run SITL
     - dronekit-sitl copter --home=-35.363261,149.165230,584,353
2. Run mavproxy to generate several UDP ports \
    - mavproxy.py --master=COM41 --out 127.0.0.1:14551 --out 192.168.10.23:14551
3. Connect MissionPlanner to one of the UDP ports
4. Run an example
  - cd dronekit-python/examples/simple_goto/
  - WSL: /mnt/c/Users/ANH/dronekit-python/examples/simple_goto
  - python simple_goto.py
5. Other specific connection
  - python simple_goto.py --connect 127.0.0.1:14550

------------------------------------------------------------------------------------------------
# tweaking parameters (For changing GUIDED mode from MAVProxy)
1. cd ~/.dronekit/sitl/copter-3.3
2. rm *
3. wget https://github.com/dronekit/dronekit-sitl/files/3514761/firmware_36.zip
4. unzip firmware_36.zip
5. mv copter_36 apm
6. sudo chmod -R a+rwx ~/.dronekit/sitl/copter-3.3/

#Arm throttle
1. Inside MAVProxy 
   STABILIZE> param load /home/USER/.dronekit/sitl/copter-3.3/default.parm
----------------------------------------------------------------------------------------------------
## Cube-Orange glitch

sudo pip uninstall pymavlink
sudo pip uninstall dronekit
cd ~
git clone https://github.com/dronekit/dronekit-python.git
cd dronekit-python
sudo python setup.py build
sudo python setup.py install
--------------------------------------------------------------------------------------------------------

https://github.com/mavlink/MAVSDK
