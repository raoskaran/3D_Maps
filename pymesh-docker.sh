apt-get update

apt-get install \
	libeigen3-dev \
	libgmp-dev \
	libgmpxx4ldbl \
	libmpfr-dev \
	libboost-dev \
	libboost-thread-dev \
	libtbb-dev \
    libssl-dev \
	python3-dev \
	python3-setuptools \
	python3-numpy \
	python3-scipy \
	python3-nose \
	python3-pip \
    wget \
    git

wget https://github.com/Kitware/CMake/releases/download/v3.17.1/cmake-3.17.1.tar.gz
tar -zvxf cmake-3.17.1.tar.gz
cd cmake-3.17.1
./bootstrap
make
make install

pip3 install https://github.com/qnzhou/PyMesh/releases/download/v0.2.0/pymesh2-0.2.0-cp36-cp36m-linux_x86_64.whl