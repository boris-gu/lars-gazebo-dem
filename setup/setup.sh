#!/bin/bash
# https://discuss.px4.io/t/create-custom-model-for-sitl/6700/17
# https://github.com/willinum/SPEC21/wiki/custom-model-for-PX4-sitl-GAZEBO-simulation
RED='\033[1;31m'
YELLOW='\033[1;33m'
NOCOLOR='\033[0m'

if [ -z $1 ] || [ $1 == "--help" ]; then
    echo "Usage: setup.sh PATH_TO_PX4"
    echo "  For example:"
    echo "  setup.sh /home/user/PX4-Autopilot"
elif [ ! -d $1 ]; then
    echo -en "${RED}[ERR]: ${NOCOLOR}"
    echo "Directory $1 not found"
else
    SCRIPT=$(realpath $0)
    SCRIPTPATH=$(dirname $SCRIPT)

    #0 Remove file of the previous setup
    MODELS=$1/Tools/sitl_gazebo/models
    rm -rf $1/build
    rm -rf $MODELS/gps_lars_dem
    rm -rf $MODELS/fpv_cam_lars_dem
    rm -rf $MODELS/iris_lars_dem
    rm -rf $MODELS/iris_lidar_lars_dem
    rm -rf $MODELS/typhoon_h480_lars_dem
    rm -rf $MODELS/MAP_N55E106 $MODELS/MAP_N56E110

    #1 Create a model under Tools/sitl_gazebo/models
    if [ -d $MODELS ]; then
        cp -r $SCRIPTPATH/models/*/ $MODELS
    else
        echo -en "${RED}[ERR]: ${NOCOLOR}"
        echo "Directory $MODELS not found"
    fi

    #2 Create a world file in Tools/sitl_gazebo/worlds
    WORLDS=$1/Tools/sitl_gazebo/worlds
    if [ -d $WORLDS ]; then
        cp -r $SCRIPTPATH/worlds/* $WORLDS
    else
        echo -en "${RED}[ERR]: ${NOCOLOR}"
        echo "Directory $WORLDS not found"
    fi

    #3 Create an airframe file under ROMFS/px4fmu_common/init.d-posix/airframes
    AIRFRAMES=$1/ROMFS/px4fmu_common/init.d-posix/airframes
    if [ -d $AIRFRAMES ]; then
        cp $SCRIPTPATH/airframes/* $AIRFRAMES
    else
        echo -en "${RED}[ERR]: ${NOCOLOR}"
        echo "Directory $AIRFRAMES not found"
    fi

    #4 Add the airframe name to the file platforms/posix/cmake/sitl_target.cmake
    FILE_SITL_TARGET=$1/platforms/posix/cmake/sitl_target.cmake
    if [ -f $FILE_SITL_TARGET ]; then
        lineNum=$(grep -n "set(models" $FILE_SITL_TARGET | head -n 1 | cut -d: -f1)
        lineNum=$((lineNum + 1))
        iris_lars_dem_OK=$(grep -n "iris_lars_dem" $FILE_SITL_TARGET | head -n 1 | cut -d: -f1)
        if [ -z $iris_lars_dem_OK ]; then
            sed -i "${lineNum}i \\\tiris_lars_dem" $FILE_SITL_TARGET
        else
            echo -en "${YELLOW}[WARNING]: ${NOCOLOR}"
            echo "iris_lars_dem already contains in sitl_target.cmake"
        fi
    else
        echo -en "${RED}[ERR]: ${NOCOLOR}"
        echo "File $FILE_SITL_TARGET not found"
    fi

    if [ -f $FILE_SITL_TARGET ]; then
        lineNum=$(grep -n "set(models" $FILE_SITL_TARGET | head -n 1 | cut -d: -f1)
        lineNum=$((lineNum + 1))
        iris_lidar_lars_dem_OK=$(grep -n "iris_lidar_lars_dem" $FILE_SITL_TARGET | head -n 1 | cut -d: -f1)
        if [ -z $iris_lidar_lars_dem_OK ]; then
            sed -i "${lineNum}i \\\tiris_lidar_lars_dem" $FILE_SITL_TARGET
        else
            echo -en "${YELLOW}[WARNING]: ${NOCOLOR}"
            echo "iris_lidar_lars_dem already contains in sitl_target.cmake"
        fi
    else
        echo -en "${RED}[ERR]: ${NOCOLOR}"
        echo "File $FILE_SITL_TARGET not found"
    fi

    if [ -f $FILE_SITL_TARGET ]; then
        lineNum=$(grep -n "set(models" $FILE_SITL_TARGET | head -n 1 | cut -d: -f1)
        lineNum=$((lineNum + 1))
        typhoon_h480_lars_dem_OK=$(grep -n "typhoon_h480_lars_dem" $FILE_SITL_TARGET | head -n 1 | cut -d: -f1)
        if [ -z $typhoon_h480_lars_dem_OK ]; then
            sed -i "${lineNum}i \\\ttyphoon_h480_lars_dem" $FILE_SITL_TARGET
        else
            echo -en "${YELLOW}[WARNING]: ${NOCOLOR}"
            echo "typhoon_h480_lars_dem already contains in sitl_target.cmake"
        fi
    else
        echo -en "${RED}[ERR]: ${NOCOLOR}"
        echo "File $FILE_SITL_TARGET not found"
    fi

    #5 Add the airframe name to the file ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
    FILE_CMAKELISTS=$1/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
    if [ -f $FILE_CMAKELISTS ]; then
        lineNum=$(grep -n "px4_add_romfs_files(" $FILE_CMAKELISTS | head -n 1 | cut -d: -f1)
        lineNum=$((lineNum + 1))
        iris_lars_dem_OK=$(grep -n "22101701_iris_lars_dem" $FILE_CMAKELISTS | head -n 1 | cut -d: -f1)
        if [ -z $iris_lars_dem_OK ]; then
            sed -i "${lineNum}i \\\t22101701_iris_lars_dem" $FILE_CMAKELISTS
        else
            echo -en "${YELLOW}[WARNING]: ${NOCOLOR}"
            echo "iris_lars_dem already contains in CMakeLists.txt"
        fi
    else
        echo -en "${RED}[ERR]: ${NOCOLOR}"
        echo "File $FILE_CMAKELISTS not found"
    fi

    FILE_CMAKELISTS=$1/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
    if [ -f $FILE_CMAKELISTS ]; then
        lineNum=$(grep -n "px4_add_romfs_files(" $FILE_CMAKELISTS | head -n 1 | cut -d: -f1)
        lineNum=$((lineNum + 1))
        iris_lidar_lars_dem_OK=$(grep -n "22111601_iris_lidar_lars_dem" $FILE_CMAKELISTS | head -n 1 | cut -d: -f1)
        if [ -z $iris_lidar_lars_dem_OK ]; then
            sed -i "${lineNum}i \\\t22111601_iris_lidar_lars_dem" $FILE_CMAKELISTS
        else
            echo -en "${YELLOW}[WARNING]: ${NOCOLOR}"
            echo "iris_lidar_lars_dem already contains in CMakeLists.txt"
        fi
    else
        echo -en "${RED}[ERR]: ${NOCOLOR}"
        echo "File $FILE_CMAKELISTS not found"
    fi

    FILE_CMAKELISTS=$1/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
    if [ -f $FILE_CMAKELISTS ]; then
        lineNum=$(grep -n "px4_add_romfs_files(" $FILE_CMAKELISTS | head -n 1 | cut -d: -f1)
        lineNum=$((lineNum + 1))
        typhoon_h480_lars_dem_OK=$(grep -n "22121301_typhoon_h480_lars_dem" $FILE_CMAKELISTS | head -n 1 | cut -d: -f1)
        if [ -z $typhoon_h480_lars_dem_OK ]; then
            sed -i "${lineNum}i \\\t22121301_typhoon_h480_lars_dem.post" $FILE_CMAKELISTS
            sed -i "${lineNum}i \\\t22121301_typhoon_h480_lars_dem" $FILE_CMAKELISTS
        else
            echo -en "${YELLOW}[WARNING]: ${NOCOLOR}"
            echo "typhoon_h480_lars_dem already contains in CMakeLists.txt"
        fi
    else
        echo -en "${RED}[ERR]: ${NOCOLOR}"
        echo "File $FILE_CMAKELISTS not found"
    fi

    #6 Create a launch file
    cp $SCRIPTPATH/launch/* $1/launch

    echo "Setup completed"
fi
