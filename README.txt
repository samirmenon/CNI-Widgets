Arduino projects for the Stanford Center for Cognitive and Neurobiological Imaging (CNI).

We often use the Teensy board, due to the flexible USB interface. E.g., it can be
configured to emulate a USB keyboard. Having USB separate from the AVR UART is also
very handy.

To get the Teensy working, first install Arduino (http://arduino.cc/en/Main/Software).
Then, install the Teensy tools (http://www.pjrc.com/teensy/td_download.html).

To use the Teensy Serial mode on Windows, you'll need the .inf file that configures
the USB to serial device. We have a slightly customized inf file in this repository
(CNI_serial.inf).

