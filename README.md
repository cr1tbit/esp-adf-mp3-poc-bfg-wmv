# This is a project

This is my project, do not use it probably

dd

And the  commands:

creating spiffs:
```
/Projects/software/mkspiffs/mkspiffs -c ./tools -b 4096 -p 256 -s 0x390000 ./tools/adf_music.bin
```

uploading spiffs:
```
esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 write_flash -z 0x070000 ./tools/adf_music.bin
```

I think I finally lost it completely
