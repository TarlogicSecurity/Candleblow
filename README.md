# Candleblow
Sniffing firmware for Microchip's ATPL360-EK evaluation kit, to use in conjunction with [PLCTool](https://github.com/TarlogicSecurity/PLCTool).

## Requirements 
You will need to install the latest version version of [Atmel Studio 7](https://www.microchip.com/mplab/avr-support/avr-and-sam-downloads-archive) (Windows only). You will also need an unexpensive 20-pin JTAG ARM programmer [like this one](https://www.amazon.es/Gen%C3%A9rico-depurador-soporte-Ortex-M3-reemplazo/dp/B086RJZXGJ/ref=sr_1_4?__mk_es_ES=%C3%85M%C3%85%C5%BD%C3%95%C3%91&dchild=1&keywords=Jtag+Usb+arm&qid=1606856754&sr=8-4) and, of course, the [ATPL360-EK](https://www.microchip.com/DevelopmentTools/ProductDetails/PartNo/ATPL360-EK?utm_source=MicroSolutions&utm_medium=Article&utm_content=DevTools&utm_campaign=StandAlone)

## Build
Open the solution file `Candleblow.atsln` with Atmel Studio 7 and press F7. This should start the build process:

![](https://raw.githubusercontent.com/TarlogicSecurity/Candleblow/main/doc/atmel1.png)

If the build is successful, now it's time to plug the ARM Programmer to your computer via USB and the board with the JTAG cable. Now click on the green "play" button or just press F5. This will attempt to flash the board:

![](https://raw.githubusercontent.com/TarlogicSecurity/Candleblow/main/doc/atmel2.png)

If the board's LCD displays a message saying `Probe ready :)`, congratulations! You are ready to use Candleblow with PLCTool. 
