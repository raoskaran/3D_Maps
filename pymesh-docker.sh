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

pip3 install https://github.com/qnzhou/PyMesh/releases/download/v0.2.0/pymesh2-0.2.0-cp36-cp36m-linux_x86_64.whl
apt install libgl1-mesa-glx