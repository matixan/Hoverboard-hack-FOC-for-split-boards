## Hoverboard-hack-FOC-for-split-boards
Its going to work... someday

# Based on.  
https://github.com/Candas1/Hoverboard-Firmware-Hack-Gen2

Feel free to try at your own risk and contribute 

## FOC not tested yet
- [ ] Repair fried output stage(again...)
- [x] added new ADC and input defines for future use

## What works?
- [x] sin and motor reading
- [x] ADC readings including between phase measurements
- [x] CLI
- [x] Properly turning on and off
- [x] LED definition and control
- [ ] master slave communication TO TEST

## To do:
- [ ] whole bunch of main functions test
- [ ] upgrade foc drivers from Eferu repo
- [ ] clean up code, better comments
- [ ] enhance master slave communication
- [ ] add possible motherboard variants
- [ ] add possible steering variants
- [ ] add documentation and descriptions
- [ ] code organisation for possibility to merge with original FOC

I am building escooter, so at first I'm going to match code for my specyfic setup, later on there'll be more ifdefs for different variants
