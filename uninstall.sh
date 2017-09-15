#!/bin/bash


bash debian/prerm
cd build/
make clean
rm -rf build/
bash debian/postrm
ldconfig
