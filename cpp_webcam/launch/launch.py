#ver=0.1.0
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import sys
import json
import yaml
from yaml import load, Loader

pkgName = 'cpp_webcam'

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
            executable=data['launch_node'],
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "topic_Webcam_topicName" : data['topic_Webcam']['topicName'] + '_' + str(data['generic_prop']['id']), 
                    "topic_Webcam_pubInterval_s" : data['topic_Webcam']['publishInterval_s'], 
                    "topic_Webcam_width" : data['topic_Webcam']['width'], 
                    "topic_Webcam_height" : data['topic_Webcam']['height'], 
                    "topic_use_compression" : data['topic_Webcam']['use_compression'], 
                    "topic_compression_quality" : data['topic_Webcam']['compression_quality'], 
                    "camera_cap_id" : data['camera_prop']['cap_id'], 
                    "camera_fps" : data['camera_prop']['fps'], 
                    "camera_width" : data['camera_prop']['width'], 
                    "camera_height" : data['camera_prop']['height'], 
                    "camera_use_color" : data['camera_prop']['use_color'], 
                    "cameraProcTimeout_ms" : data['procedure_timeout_ms_prop']['Camera'], 
                    "pubImageProcTimeout_ms" : data['procedure_timeout_ms_prop']['pubImage'], 
                    "pubCamInfoProcTimeout_ms" : data['procedure_timeout_ms_prop']['pubCamInfo'], 

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