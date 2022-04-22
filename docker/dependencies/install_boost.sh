#!/bin/sh

# Note: It is best to install libraries in /prefix to make the use of docker multistage easier.

# Remove any other boost installation
apt-get update
apt-get purge -y cmake
apt-get install -y --no-install-recommends wget gcc ca-certificates
# Remove previous boost
apt-get -y --purge remove libboost-all-dev libboost-doc libboost-dev \
&& rm -rf /usr/lib/libboost_* /usr/include/boost/
# Our server implementation use cpprest which depends on boost
wget -q https://dl.bintray.com/boostorg/release/1.68.0/source/boost_1_68_0.tar.gz
tar -xvf boost_1_68_0.tar.gz && cd boost_1_68_0
./bootstrap.sh
./b2 install --prefix=/prefix --with-filesystem --with-system --with-random --with-random --with-thread \
--with-iostreams --with-date_time --with-chrono --with-atomic --with-regex && cd ..
rm -rf boost_1_68_0.tar.gz boost_1_68_0

BOOST_FILE="/prefix/lib/libboost_filesystem.so.1.68.0"
if [ ! -f $BOOST_FILE ]; then
    echo "BOOST was not correctly installed."
    exit 1
fi

exit 0
