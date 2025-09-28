### Setup CAN-interface

---

Prvo što je potrebno je instalirati sve drivere za CanHat RS485.

To se može napraviti pomoću sljedeće naredbe:

```bash
sudo su
wget https://github.com/joan2937/lg/archive/master.zip
unzip master.zip
cd lg-master
sudo make install
# Za više informacija posjetiti: https://www.waveshare.com/wiki/RS485_CAN_HAT
```

Sljedeće treba se obaviti preparacija za CAN interface.

```bash
sudo nano /boot/firmware/config.txt #Bookworm/Ubuntu
# dodati sljedeće linije na kraj datoteke:
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=500000
```

Sada je potrebno postaviti can interface.
Najlaše je to napraviti tako da se postave naredbe u .bashrc datoteci.

To radimo pomoću sljedećih naredbi:

```bash
sudo nano ~/.bashrc
# dodati sljedeće linije na kraj datoteke:
sudo ip link set can0 up type can bitrate 1000000
sudo ifconfig can0 txqueuelen 65536
sudo ifconfig can0 up
```

Isto tako trebamo dodati i virtualni CAN interface koji će nam služiti za testiranje.

```bash
sudo nano ~/.bashrc
# dodati sljedeće linije na kraj datoteke:
sudo ip link add vcan0 type vcan
sudo ip link set vcan0 up
```

Za kraj potrebno je postaviti komunikaciju preko CAN interface-a. To radimo pomoću CAN-utils paketa i CAN-gw.

```bash
sudo nano ~/.bashrc
# dodati sljedeće linije na kraj datoteke:
sudo modprobe can-gw
sudo can-gw -A -s vcan0 -d can0 -e
sudo can-gw -A -s can0 -d vcan0 -e
```

Nakon što smo sve to napravili potrebno je još u raspi-config uključiti serijski port.

```bash
sudo raspi-config
# Prva postavka treba biti stavljena na: No te druga postavka na: Yes
# Interfacing Options -> Serial Port -> No -> Yes
```

Nakon što smo sve to napravili potrebno je restartati Raspberry Pi.

```bash
sudo reboot
```

Nakon restarta možemo provjeriti da li je sve ispravno postavljeno pomoću sljedećih naredbi:

```bash
ifconfig
# Trebali bi vidjeti can0 i vcan0 interface
```

Testiranje CAN interface-a možemo napraviti pomoću sljedećih naredbi:

```bash
# U jednom terminalu pokrenuti:
candump can0
# U drugom terminalu pokrenuti:
cansend vcan0 123#DEADBEEF
# Trebali bi vidjeti poruku DEADBEEF u prvom terminalu
```
