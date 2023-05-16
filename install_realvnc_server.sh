#!/bin/bash

mkdir realvncserversetup && cd realvncserversetup
echo Add armhf arch
sudo dpkg --add-architecture armhf && sudo apt update

echo Download and install RealVNC server
wget -nv https://downloads.realvnc.com/download/file/vnc.files/VNC-Server-6.11.0-Linux-ARM.deb && sudo apt install ./VNC-Server-6.11.0-Linux-ARM.deb

echo Download and install dependencies
files=( libbcm_host.so libvcos.so libmmal.so libmmal_core.so libmmal_components.so \
	libmmal_util.so libmmal_vc_client.so libvchiq_arm.so libvcsm.so libcontainers.so )
for i in "${files[@]}"
do
	wget -nv https://github.com/raspberrypi/firmware/raw/master/opt/vc/lib/$i
done
sudo mv *.so /usr/lib/
cd ..

echo Enable VNC service on boot
sudo systemctl enable vncserver-x11-serviced.service
echo Start VNC service
sudo systemctl start vncserver-x11-serviced.service
