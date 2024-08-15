#ver=0.1.0
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import sys
import json
import yaml
from yaml import load, Loader

pkgName = 'cpp_gnss'

configDirPath = ''
for arg in sys.argv:
    if arg.startswith("cdir:=") : 
        configDirPath = arg.split(":=")[1]

if configDirPath:
    commonFilePath = os.path.join(configDirPath, 'common.yaml')
    serviceFilePath = os.path.join(configDirPath, 'service.json')
else:
    print("No config directory path is given. Using default values.")
    commonFilePath = os.path.join(get_package_share_directory(pkgName), 'launch/common.yaml')
    serviceFilePath = os.path.join(get_package_share_directory(pkgName), 'launch/service.json')



def generate_launch_description():
    with open(commonFilePath, 'r') as f:
        data = yaml.load(f, Loader=Loader)

    with open(serviceFilePath, 'r') as f:
        serviceData = json.load(f)

    return LaunchDescription([
        Node(
            package=pkgName,
            namespace=data['generic_prop']['namespace'],
            executable="pub",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "topicName" : data['topic_GPS']['topicName'] + '_' + str(data['generic_prop']['id']), 
                    "publishInterval_s" : data['topic_GPS']['publishInterval_s'], 
                    "gpsDevice" : data['GPS_prop']['gpsDevice'], 
                    "gpsBaud_dec" : data['GPS_prop']['gpsBaud_dec'], 
                    "ntripCaster" : data['GPS_prop']['ntripCaster'], 
                    "ntripPort" : data['GPS_prop']['ntripPort'], 
                    "ntripMountpoint" : data['GPS_prop']['ntripMountpoint'], 
                    "ntripUsername" : data['GPS_prop']['ntripUsername'], 
                    "ntripPassword" : data['GPS_prop']['ntripPassword'], 
                    "gpsModuleProcTimeout_ms" : data['procedure_timeout_ms_prop']['gpsModule'], 
                    "msgUpdateProcTimeout_ms" : data['procedure_timeout_ms_prop']['msgUpdate'], 
                    "msgPublishProcTimeout_ms" : data['procedure_timeout_ms_prop']['msgPublish'], 

                    # Settings for Params class under vehicle_interfaces/params.h
                    # Do not change the settings rashly
                    "nodeName" : data['generic_prop']['nodeName'] + '_' + str(data['generic_prop']['id']) + '_node', 
                    "id" : data['generic_prop']['id'], 
                    "devManageService" : serviceData['devManageService'], 
                    "devInterface" : serviceData['devInterface'], 
                    "qosService" : serviceData['qosService'], 
                    "qosDirPath" : serviceData['qosDirPath'], 
                    "safetyService" : serviceData['safetyService'], 
                    "timesyncService" : serviceData['timesyncService'], 
                    "timesyncPeriod_ms" : serviceData['timesyncPeriod_ms'], 
                    "timesyncAccuracy_ms" : serviceData['timesyncAccuracy_ms'], 
                    "timesyncWaitService" : serviceData['timesyncWaitService'], 
                }
            ]
        )
    ])
