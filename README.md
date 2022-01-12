# Ardupilot_Dronekit

1. apt install python-is-python3

2. apt-get update

3. apt install python3-pip -y

4. pip install pymavlink && pip install mavproxy

5. git clone -b Copter-4.1 http://github.com/ardupilot/ardupilot

6. cd ardupilot

7. git submodule update --init --recursive

8. cd ardupilot/ArduCopter

# Compile Ardupilot compile at /root/ardupilot/ArduCopter
9. /root/ardupilot/Tools/autotest/sim_vehicle.py     

10. After Compilation, exit (Ctrl + c)

12. Open Ubuntu window

# Go to compiled file location
13. cd /root/ardupilot/build/sitl/bin

# Run sitl arducopter at /root/ardupilot/build/sitl/bin
14. ./arducopter -S -I0 --home -35.363261,149.165230,584,353 --model "+" --speedup 1 --defaults /root/ardupilot/Tools/autotest/default_params/copter.parm

15. Open another window for running MAVProxy

# Telemetry forwarding/broadcasting
16. mavproxy.py --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 

17. Connect MISSION PLANNER to one of the broadcasted IP & Port, and connect Dronekit or MAVSDK to another broadcasted IP & Port.

18. Run step 13 to 16 if program or PC is restarted.

19. Using alias shortcut
    nano ~/.bashrc
    alias sitl='/root/ardupilot/build/sitl/bin/arducopter -S -I0 --home -35.363261,149.165230,584,353 --model "+" --speedup 1 --defaults /root/ardupilot/Tools/autotest/default_params/copter.parm'
    - save and source bashrc 
    source ~/.bashrc

20. type sitl and run ardupilot.


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
