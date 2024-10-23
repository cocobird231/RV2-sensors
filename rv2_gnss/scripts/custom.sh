# This is an optional script that will be executed under `create-service` command in the `rv2_startup` script.
# The script will be executed before the `runfile.sh` is created, before the service file is created.

#!/bin/bash

# echo "[custom]"

# The argument $1 should be the system.yaml file path.
# The argument $2 should be the repo path, the path could be under ROS2 workspace source directory or the ROS2 global share directory.
# echo ${@}

# ref: https://stackoverflow.com/a/47791935
# yaml file_path key
yaml ()
{
    python3 -c "import yaml;print(yaml.safe_load(open('$1'))$2)" 2>/dev/null
}


ReadYAML ()
{
    local gpsDeviceName=$(yaml $1 "['custom']['gpsDeviceName']")
    local gpsDevicePath=$(yaml $1 "['custom']['gpsDevicePath']")
    local gpsBaudRate=$(yaml $1 "['custom']['gpsBaudRate']")
    local ntripServer=$(yaml $1 "['custom']['ntripServer']")
    local ntripPort=$(yaml $1 "['custom']['ntripPort']")
    local ntripMountPoint=$(yaml $1 "['custom']['ntripMountPoint']")
    local ntripUser=$(yaml $1 "['custom']['ntripUser']")
    local ntripPassword=$(yaml $1 "['custom']['ntripPassword']")

    if [ -z "${gpsDeviceName}" ] || [ -z "${gpsDevicePath}" ] || [ -z "${gpsBaudRate}" ]; then
        echo "The configuration of GPS device is not complete. Please check the system.yaml file."
        return 1
    fi

    local use_ntrip=0
    local anonymous_user=1

    if [ -n "${ntripServer}" ] && [ -n "${ntripPort}" ] && [ -n "${ntripMountPoint}" ]; then
        use_ntrip=1
    fi

    if [ -n "${ntripUser}" ] && [ -n "${ntripPassword}" ]; then
        anonymous_user=0
    fi

    local src_file=$2/scripts/source_env.sh
    sudo rm -rf ${src_file} && sudo touch ${src_file}
    sudo echo "sudo systemctl stop gpsd.service" >> ${src_file}
    sudo echo "sudo systemctl stop gpsdctl@${gpsDeviceName}.service" >> ${src_file}
    sudo echo "sudo systemctl stop gpsd.socket" >> ${src_file}

    # No NTRIP:                 sudo gpsd -D 5 -n -s ${gpsBaudRate} ${gpsDevicePath}
    # Use NTRIP with anonymous: sudo gpsd -D 5 -n 'ntrip://${ntripServer}:${ntripPort}/${ntripMountPoint}' -s ${gpsBaudRate} ${gpsDevicePath}
    # Use NTRIP with user:      sudo gpsd -D 5 -n 'ntrip://${ntripUser}:${ntripPassword}@${ntripServer}:${ntripPort}/${ntripMountPoint}' -s ${gpsBaudRate} ${gpsDevicePath}
    if [ ${use_ntrip} -eq 0 ]; then
        sudo echo "sudo gpsd -D 5 -n -s ${gpsBaudRate} ${gpsDevicePath}" >> ${src_file}
    else
        if [ ${anonymous_user} -eq 1 ]; then
            sudo echo "sudo gpsd -D 5 -n 'ntrip://${ntripServer}:${ntripPort}/${ntripMountPoint}' -s ${gpsBaudRate} ${gpsDevicePath}" >> ${src_file}
        else
            sudo echo "sudo gpsd -D 5 -n 'ntrip://${ntripUser}:${ntripPassword}@${ntripServer}:${ntripPort}/${ntripMountPoint}' -s ${gpsBaudRate} ${gpsDevicePath}" >> ${src_file}
        fi
    fi
    sudo echo "sleep 1" >> ${src_file}
}

ReadYAML $1 $2
