# Oxford Driver SDK fork

This fork has changes necessary to use the oxford driver as SDK

## Build from source

Dependencies:

```bash
sudo apt install doxygen
pip3 install sphinx breathe sphinx_rtd_theme
```

To selectivly build the driver as SDK:

```bash
git clone --no-checkout --depth 1 --sparse --filter=blob:none https://github.com/npd15393/oxts_ros2_driver
cd oxts_ros2_driver/
git sparse-checkout init --cone
git sparse-checkout add oxts_driver
git checkout master
cd oxts_driver
mkdir build
cd build
cmake ..
make -j4
```

The 'build' folder would contain the shared lib.