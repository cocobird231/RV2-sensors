#ver=0.1.0
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import sys
import json
import yaml
from yaml import load, Loader

pkgName = 'cpp_zedcam2'

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
            # output="screen",
            # emulate_tty=True,
            parameters=[
                {
                    "frame_id" : data['topic_prop']['frame_id'], 
                    "downsampleFactor" : data['topic_prop']['downsampleFactor'], 
                    "confidenceThreshold" : data['topic_prop']['confidenceThreshold'], 
                    "rgbQuality" : data['topic_prop']['rgbQuality'], 
                    "camera_topic_ids" : data['camera_prop']['topic_ids'], 
                    "camera_caps" : data['camera_prop']['caps'], 
                    "camera_fps" : data['camera_prop']['fps'], 
                    "camera_res" : data['camera_prop']['res'], 
                    "camera_svo_path" : data['camera_prop']['svo_path'], 
                    "camera_use_svo" : data['camera_prop']['use_svo'], 
                    "camera_sensing_mode" : data['camera_prop']['sensing_mode'], 
                    "camera_depth_quality" : data['camera_prop']['depth_quality'], 
                    "camera_depth_unit" : data['camera_prop']['depth_unit'], 
                    "camera_pc_range" : data['camera_prop']['pc_range'], 
                    "camera_pc_res" : data['camera_prop']['pc_res'], 
                    "camera_use_color" : data['camera_prop']['use_color'], 
                    "camera_use_depth" : data['camera_prop']['use_depth'], 
                    "camera_use_pc" : data['camera_prop']['use_pc'], 
                    "camera_use_sens" : data['camera_prop']['use_sens'], 
                    "zedThProcTimeout_ms" : data['procedure_timeout_ms_prop']['zedTh'], 
                    "pubRgbProcTimeout_ms" : data['procedure_timeout_ms_prop']['pubRgb'], 
                    "pubDepthProcTimeout_ms" : data['procedure_timeout_ms_prop']['pubDepth'], 
                    "pubPCProcTimeout_ms" : data['procedure_timeout_ms_prop']['pubPC'], 
                    "pubSensProcTimeout_ms" : data['procedure_timeout_ms_prop']['pubSens'], 

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