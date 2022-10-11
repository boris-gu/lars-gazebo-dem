# lars-gazebo-dem

# Создание модели
Изначально Gazebo умеет строить модель поверхности по карте высот, прописанной в SDF файле. Но из-за бага, коллизия зачастую не совпадает с визуальной частью. Выход - создать модель самостоятельно.
## Инструменты
1. [Blender](https://www.blender.org/)  
    __Ubuntu:__  
    В репозиториях Ubuntu зачастую лежат версии программ не самой первой свежести. Поэтому вместо пакетного менеджера APT (`sudo apt install ...`) советую использовать  
    Snap (установлен "из коробки"): https://snapcraft.io/blender  
    Flatpak: https://flathub.org/apps/details/org.blender.Blender
2. [GDAL](https://gdal.org/index.html)  
    __Ubuntu:__  
    ```bash
    sudo apt install gdal-bin
    ```
3. [BlenderGIS](https://github.com/domlysz/BlenderGIS)

## Карта высот
1. Скачать карту высот: https://gdemdl.aster.jspacesystems.or.jp/index_en.html
    - Из всего архива оставить только файл с названием AST**GTM**V003\_ N35E135\___dem__.tif (числа могут отличаться)  
    Описание ASTER GDEM https://www.jspacesystems.or.jp/ersdac/GDEM/E/2.html
2. Обрезать, исходя из принципа 1пкс~30м (9000м~300пкс)
    ```bash
    gdal_translate -srcwin 0 3301 300 300 ./ASTGTMV003_N56E110_dem.tif ./DEM_cut.tif
    # 0 3301  - координаты левого правого угла вырезаемой области
    # 300 300 - размер вырезаемой области в пикселях
    ```
3. Перевести в равнопромежуточную проекцию [WGS 84 / World Equidistant Cylindrical (EPSG:4087)](https://epsg.io/4087)
    ```bash
    gdalwarp -t_srs EPSG:4087 ./DEM_cut.tif ./DEM_cut_4087.tif
    ```

## Текстура
1. Узнать координаты углов оригинального вырезанного куска
    ```bash
    gdalinfo ./DEM_cut.tif
    # Посмотреть информацию GeoTIFF, узнать координаты углов
    ```
1. Скачать спутниковый снимок: https://apps.sentinel-hub.com/eo-browser/
    - Зарегистрироваться на сайте
    - Выбрать прямоугольник чуть больше обрезанного для карты высот
    - Скачать __Highlight Optimized Natural Color__ в формате TIFF(8 bit), высокое разрешение, WGS 84
2. Перевести в равнопромежуточную проекцию [WGS 84 / World Equidistant Cylindrical (EPSG:4087)](https://epsg.io/4087)
    ```bash
    gdalwarp -t_srs EPSG:4087 ./COLOR.tif ./COLOR_4087.tif
    ```

## Модель Blender
1. Удалить все объекты
2. Создать модель
    - GIS -> Import -> Georeferenced raster
    - Добавить новую CRS -> Search -> WGS 84 / World Equidistant Cylindrical
    - Mode: DEM as displacement texture -> Выбрать файл DEM_cut_4087.tif и импортировать
    - Add Modifier -> Subdivision Surface -> Перенести его наверх -> Simple -> Увеличить Levels Viewport до приемлимого
3. Добавить текстуру
    - GIS -> Import -> Georeferenced raster -> Mode: Basemap on mesh -> CRS: WGS 84 / World Equidistant Cylindrical
    - Выбрать файл COLOR_4087.tif и импортировать
    - Shading -> Principled BSDF -> Убрать Specular
4. Преобразовать в меш
    ПКМ -> Convert To -> Mesh
5. Экспортировать как Collada (.dae)

## Модель Gazebo
Пользовательские модели хранятся по пути ~/.gazebo/models  
Структура каталога модели:  
```
MAP_N56E110
├── collada
│   ├── COLOR_4087.tif
│   └── map_N56E110.dae
├── model.config
└── model.sdf
```

### Файл model.config
```xml
<?xml version="1.0"?>
<model>
  <name>map N56E110</name>
  <version>1.0</version>
  <sdf version="1.5">model.sdf</sdf>

  <author>
    <name>Boris Gubanov</name>
  </author>

  <description>
    Map model ~9x9km, lower left  N56E110
  </description>

</model>

```
### Файл model.sdf
```xml
<?xml version="1.0" ?>
<sdf version="1.5">
  <model name="map N56E110">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://MAP_N56E110/collada/map_N56E110.dae</uri>
          </mesh>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://MAP_N56E110/collada/map_N56E110.dae</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>

```

## Полезные ссылки
Популярные системы отсчета  
[WGS 84 / World Geodetic System, EPSG:4326, градусы](https://epsg.io/4326) [Wiki](https://en.wikipedia.org/wiki/World_Geodetic_System)  
[Web Mercator, EPSG:3857, метры](https://epsg.io/3857) [Wiki](https://en.wikipedia.org/wiki/Web_Mercator_projection)  
[WGS 84 / World Equidistant Cylindrical, EPSG:4087, метры](https://epsg.io/4087) [Wiki](https://ru.wikipedia.org/wiki/Равнопромежуточная_проекция)

[После импорта в Blender карты высот, вместо модели огромная "палка"](https://github.com/domlysz/BlenderGIS/wiki/FAQ#why-after-the-import-of-my-data-the-resulting-mesh-is-extremely-small-on-x-and-y-axis-but-seems-correct-on-z-axis-)

