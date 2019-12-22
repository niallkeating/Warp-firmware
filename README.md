# Development Aid for Colour Blind Children
Developed by: \
Niall Keating \
Gonville and Caius College \
njk37

# Introduction:
For people with full colour sensitivity it is difficult to perceive a world with fewer colours in. This means it is very difficult to predict what colours will and won’t be visible and/or effective in being stimulating to a child with colour blindness and aid their development. This project aims to tackle that problem by having a device which is calibrated towards a particular child’s level of colour blindness, that can then can be used by a family member/teacher to sense the colour of an object/item, such that it can be translated into what the child effectively sees.

The device therefore detects the RGB elements of a colour, converts them to RGB values for the OLED screen and then applies a filter such that the colour shown on the screen is a far closer representation to what the colour-blind person sees. The use of several push buttons also makes it very user friendly such that a child could easily use it. The device is capable of simulating 3 types of colour blindness: Deuteranopia (green-deficiency), Protanopia (red-deficiency) and Tritanopia (blue-deficiency).


# Baseline firmware
This firmware is modified from the original firmware for the [Warp hardware](https://github.com/physical-computation/Warp-hardware) family of hardware platforms. Thank you to Phillip Stanley-Marbel and Martin Rinard for making it's usage available for this project.

## 1.  Compiling the Warp firmware
Please follow the link above to the Warp Hardware for information on how to compile and run the firmware in conjunction with SEGGER's J-link software.

## 2. Interacting with the device
Once the Firmware has been uploaded to the FRDMKL03 board, the user has the option of leaving the device connected to the compiling device and running 

	JLinkRTTClient
	
This enables read-outs, explaining what is happening at each stage when a button is pressed. When taking colour readings and converting them, this also displays the filtered and raw values taken by the TCS34725 sensor.

Alternatively, the board can still be used with just connection to a portable charger via it's USB cable making it portable. The layout of the board and buttons is very intuitve to help in this task.

## 3. Breadboard button definitions:

There are 4 buttons layed out in a cross style, each representing up, down, previous and next. To the right of the plus there is a fifth button, which is used as a 'select' feature.

## 4a. Configuring the device

On boot-up, the user is presented with a rainbow of colours, as a person with normal vision would see it. Clicking the up/down buttons on this spectrum adjusts the contrast/brightness across *all* the colours on the SSD1331 OLED device. 

Using the previous/next buttons, the user can cycle through how that spectrum would be observed by someone with a type of colour blindness. The order of the colour blindness types are: Normal --> Deuteranopia (green-deficiency) --> Protanopia (red-deficiency) --> Tritanopia (blue-deficiency) --> Normal --> ... . 

For the different colour-blindness filters used, the user can tailor the filter to a specific person's level of colour blindness by adjusting the contrast of the colour which that particular blindness type suggests a deficiency in (i.e. on the Deuteranopia filter, the up/down arrows adjust the green contrast, with the blue/red values constant). 

Once the user is happy with the settings, they can press the select button to lock in the filter and contrast level set. (Note, selecting the 'normal' mode will select no filter)

## 4b. Running tests
With the settings locked in, colour-blindness conversions can begin. In order to take a reading, place the object you want to convert the colour of and place it up to the white light on the TCS34725 sensor. The optimal distance from the sensor is around 1cm, although distances up to 5cm should still be ok if the rest of the room is dark.

To take the reading, press the select button again. This will light up the OLED as a block of colour, which is the colour of the object put up to the sensor, filtered by the chosen setting.

To modify the setting, simply press the previous button. To go back to the normal vision setting, press next.



## 5. Modifying the implementation
The core of the interfacing is dictated by the main while loop within 

	src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot.c
	
To see how the TCS34725 was configured and make any adjustments, please view the comments in 

	src/boot/ksdk1.1.0/devTCS34725.c

To modify any of the functions for configuring and displaying the OLED, please see the comments in

	src/boot/ksdk1.1.0/devSSD1331.c
	
Finally, for modifying the underlying colour transformation matrices, dictating how the spectrums are created please view

	src/boot/ksdk1.1.0/devSSD1331.h


### Acknowledgements
This firmware is based upon the the Warp-firmware: 
Phillip Stanley-Marbell and Martin Rinard. “A Hardware Platform for Efficient Multi-Modal Sensing with Adaptive Approximation”. ArXiv e-prints (2018). arXiv:1804.09241.
