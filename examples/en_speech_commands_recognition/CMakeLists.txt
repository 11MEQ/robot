cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS
    ../../components
    ../../components/u8g2-master
    )

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(speech_commands_recognition)
