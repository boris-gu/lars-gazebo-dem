# lars-gazebo-dem
Так как PX4-Autopilot быстро развивается, за основу взята стабильная версия 1.13.1.  
Клонировать:  
```bash
git clone --recurse-submodules --branch v1.13.1 https://github.com/PX4/PX4-Autopilot.git
```

Добавьте модель, запустив `setup.sh <PATH_TO_PX4>`. Этот скрипт внесет все необходимые изменения в PX4-Autopilot.

## Запуск
```bash
cd <PX4-Autopilot_clone>
DONT_RUN=1 make px4_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default && \
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd) && \
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 MAP_N55E106.launch
# или
# roslaunch px4 MAP_N56E110.launch
```
Если необходимо запустить дрон в пустом мире, воспользуйтесь командой:
```bash
roslaunch px4 mavros_posix_sitl.launch vehicle:=iris_lars_dem
# Запустите в отдельном терминале для возможности просматривать данные с лидара
rosrun tf static_transform_publisher 0 0 0 0 0 0 laser_frame laser_frame_fixed 10
```
Если необходимо запустить другую модель добавьте параметр `vehicle:=<name>`. Например:
```bash
roslaunch px4 MAP_N55E106.launch vehicle:=iris
```

## Данные с дрона
Дрон публикует в топики ROS данные с камеры и лидара.

Для просмотра изображения с камеры запустите команду `rqt_image_view` и выберите
/iris_lars_dem/usb_cam/image_raw.


Для просмотра данных лидара совершите следующие шаги:
1. Запустите команду `rviz`
2. Слева в поле "Fixed Frame" вместо "map" введите "laser_frame_fixed"
3. Слева внизу нажмите кнопку Add->By topic->laser_scan (LaserScan), нажмите OK
4. Для большей наглядности увеличьте размер точек до 0.1
