#!/bin/sh

# Note: It is best to install libraries in /prefix to make the use of docker multistage easier.

# tkDNN meeds cmake > 3.15  -> /prefix/bin/cmake
apt-get update
apt-get purge -y cmake
apt-get install -y --no-install-recommends wget gcc ca-certificates
wget -q https://github.com/Kitware/CMake/releases/download/v3.18.4/cmake-3.18.4.tar.gz
tar -xvf cmake-3.18.4.tar.gz && cd cmake-3.18.4
# install cmake with https support for cpprestsdk and opencv intel backend dowanload
apt-get install -y --no-install-recommends libcurl4-gnutls-dev
apt-get install -y --no-install-recommends zlib1g-dev
./bootstrap --parallel=$(nproc) --prefix=/prefix --system-curl \
-- \
-DCMAKE_BUILD_TYPE=RELEASE \
-DCMAKE_USE_OPENSSL=OFF
make && make install && cd ..
rm -rf cmake-3.18.4.tar.gz cmake-3.18.4

CMAKE_FILE="/prefix/bin/cmake"
if [ ! -f $CMAKE_FILE ]; then
    echo "CMAKE was not correctly installed."
    exit 1
fi

exit 0