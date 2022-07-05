# ReMake

A 3D object scanner written in python using a a **Raspberry Pi** and a **Raspberry Pi Pico**

## Documentation

You can read our documentation [here](https://github.com/KaiduY/ReMake/blob/main/Documentatie%20ReMake.pdf) (written in romanian).
If you need only the docs for the modules used in this project check out this [page](https://kaiduy.github.io/ReMake/) (written in english).

#

## Dependencies and libraries used

To install all dependencies you can use **our install script**
This will instal miniconda (a python environment manager) and will create an new environment called ReMakeENV from [conda_environment.yml](conda_environment.yml) file.

```sh
git clone https://github.com/KaiduY/ReMake
cd ReMake
chmod u+x install.sh
./install.sh
```

To get the data from the scanner run from the main directory:

```sh
conda init
conda activate ReMakeENV
python3 pointcloud.py
```

### Next you will find a list of dependencies that may not work with the script provided or need to be installed separately on the **Raspberry Pi**

  **Open Computer Vision**
  
 ```sh
 pip install opencv-python
```

 **numpy**

 ```sh
 pip install numpy
 ```

**picamera**

```sh
pip install picamera
```

**yaml**

```sh
pip install pyyaml
```

**pyserial**

```sh
pip install pyserial
```

**netifaces**

```sh
pip install netifaces
```

### For manipulating the hardware you will need the following libraries installed on the **Raspberry Pi Pico** microcontroller

**[Rotary Encoder by Gurgleapps](https://github.com/gurgleapps/rotary-encoder)** (check the author for installing instructions)

**[Python lcd](https://github.com/dhylands/python_lcd/)** (check the author for installing instructions)

```sh
sudo apt install python3-dev
python -m pip install smbus 
python -m pip install -e .
```

**[Nema motor driver](Raspberry_Pi_Pico/Nema.py)** (copy the file to the Pi Pico)

**[The main script](Raspberry_Pi_Pico/main.py)** (copy the file to the Pi Pico and **restat** the device)