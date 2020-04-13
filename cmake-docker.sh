wget https://github.com/Kitware/CMake/releases/download/v3.17.1/cmake-3.17.1.tar.gz
tar -zvxf cmake-3.17.1.tar.gz
cd cmake-3.17.1
./bootstrap
make
make install