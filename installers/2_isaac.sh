#!/bin/bash -i

sudo apt install libfuse2

until which nvidia-smi &> /dev/null; do
    echo "Warning: nvidia-smi command not found. Please install nvidia driver using"
    echo "sudo apt-get install nvidia-open"
    read -rp "Confirm installation by pressing [Enter]" 
done
echo "Successfully detected NVIDIA driver installation"


echo "nvidia-driver was installed"

#Optional choice: install a CUDA-enabled PyTorch 2.4.0 build based on the CUDA version available on your system
# python -m pip install torch==2.4.0 

#Ensure upgrade the latest pip version
# python -m pip install --upgrade pip

#Install typegaurd dependencies
# python -m pip install typeguard

echo "Installing Isaac Sim ..."
pushd src/arena/arena-rosnav || exit 1
    rm src/arena/arena-rosnav/poetry.lock
    poetry install --with isaac
popd || exit 1

echo "Remove conflicting packages ..."
_VENV_PATH=$(cd src/arena/arena-rosnav && poetry env info --path)
rm -rf "$_VENV_PATH"/lib/python3.10/site-packages/isaacsim/extscache/*.cp310/pip_prebundle/attrs
rm -rf "$_VENV_PATH"/lib/python3.10/site-packages/isaacsim/extscache/*.cp310/pip_prebundle/attr
rm -rf "$_VENV_PATH"/lib/python3.10/site-packages/isaacsim/extscache/*.cp310/pip_prebundle/typing_extensions.py
rm -rf "$_VENV_PATH"/lib/python3.10/site-packages/omni/data/Kit/Isaac-Sim\ Python/4.5/exts/3/*.cp310/pip_prebundle/typing_extensions.py
unset _VENV_PATH

if [ ! -f ~/.ros/fastdds.xml ]; then
    echo "Creating Fast DDS configuration file..."
    mkdir -p ~/.ros
    echo '<?xml version="1.0" encoding="UTF-8" ?>

    <license>Copyright (c) 2022-2024, NVIDIA CORPORATION.  All rights reserved.
    NVIDIA CORPORATION and its licensors retain all intellectual property
    and proprietary rights in and to this software, related documentation
    and any modifications thereto.  Any use, reproduction, disclosure or
    distribution of this software and related documentation without an express
    license agreement from NVIDIA CORPORATION is strictly prohibited.</license>


    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles" >
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>UdpTransport</transport_id>
                <type>UDPv4</type>
            </transport_descriptor>
        </transport_descriptors>

        <participant profile_name="udp_transport_profile" is_default_profile="true">
            <rtps>
                <userTransports>
                    <transport_id>UdpTransport</transport_id>
                </userTransports>
                <useBuiltinTransports>false</useBuiltinTransports>
            </rtps>
        </participant>
    </profiles>' > ~/.ros/fastdds.xml
fi 

#TODO redo this properly
if [ "$(systemd-detect-virt)" = wsl ] ; then
    python -m pip install git+https://github.com/cpbotha/xdg-open-wsl.git
fi

echo "Completed Isaac Sim installation" 