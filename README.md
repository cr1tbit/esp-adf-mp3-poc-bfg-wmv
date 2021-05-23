# This is a project

This is my project, do not use it probably


Prequisites:

Do ESP SDK stuff.

What to do:

0. Prepare esp-idf, esp-adf
```
 . ./export.sh #in ESP IDF
export ADF_PATH=~/Projects/SDKs/esp-adf  
```


1. Configure the SDK

idf.py menuconfig -> (TODO WHAT) -> setup flash size and custom partition table

creating spiffs:
```
/Projects/software/mkspiffs/mkspiffs -c ./music -b 4096 -p 256 -s 0x390000 ./tools/adf_music.bin
```

uploading spiffs:
```
esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 write_flash -z 0x070000 ./tools/adf_music.bin
```