# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/trong/esp/v5.1.3/esp-idf/components/bootloader/subproject"
  "D:/text_ex/Doan/Doan1/Smart-home/build/bootloader"
  "D:/text_ex/Doan/Doan1/Smart-home/build/bootloader-prefix"
  "D:/text_ex/Doan/Doan1/Smart-home/build/bootloader-prefix/tmp"
  "D:/text_ex/Doan/Doan1/Smart-home/build/bootloader-prefix/src/bootloader-stamp"
  "D:/text_ex/Doan/Doan1/Smart-home/build/bootloader-prefix/src"
  "D:/text_ex/Doan/Doan1/Smart-home/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/text_ex/Doan/Doan1/Smart-home/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/text_ex/Doan/Doan1/Smart-home/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
