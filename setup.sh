yes | sudo apt-get install libatlas-base-dev
yes | pip3 install numpy transforms3d pigpio pyserial
yes | pip install numpy transforms3d pigpio pyserial
yes | sudo pip install numpy transforms3d pigpio pyserial

sudo apt-get install -y libsdl-ttf2.0-0
yes | sudo pip install ds4drv

# cd ..
# git clone https://github.com/stanfordroboticsclub/PupperCommand.git
# cd PupperCommand
# sudo bash install.sh
# cd ..

# git clone https://github.com/stanfordroboticsclub/PS4Joystick.git
# cd PS4Joystick
# sudo bash install.sh
# cd ..
# # sudo systemctl enable joystick

cd ..
wget https://github.com/joan2937/pigpio/archive/v74.zip
unzip v74.zip
cd pigpio-74
make
sudo make install
cd ..

cd StanfordQuadruped
# sudo ln -s $(realpath .)/robot.service /etc/systemd/system/
# sudo systemctl daemon-reload
# sudo systemctl enable robot
# sudo systemctl start robot

for file in *.service; do
    [ -f "$file" ] || break
    sudo ln -s $FOLDER/$file /lib/systemd/system/
done

sudo systemctl daemon-reload
sudo systemctl enable controller
sudo systemctl start controller
