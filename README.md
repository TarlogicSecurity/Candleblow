# Candleblow
Sniffing firmware for Microchip's ATPL360-EK evaluation kit, to use in conjunction with [PLCTool](https://github.com/TarlogicSecurity/PLCTool).

## Requirements
You will need to install the latest version version of [Microchip Studio 7](https://www.microchip.com/en-us/tools-resources/develop/microchip-studio#Downloads) (Windows only). You will also need an unexpensive 20-pin JTAG ARM programmer [like this one](https://www.amazon.es/SETCTOP-J-Link-JLink-V9-5-depurador/dp/B081SFZFXR/) and, of course, the [ATPL360-EK](https://www.microchip.com/en-us/development-tool/ATPL360-EK)

## Build
Open the solution file `Candleblow.atsln` with Atmel Studio 7 and press F7. This should start the build process:

![](https://raw.githubusercontent.com/TarlogicSecurity/Candleblow/main/doc/atmel1.png)

If the build is successful, now it's time to plug the ARM Programmer to your computer via USB and the board with the JTAG cable. Now click on the green "play" button or just press F5. This will attempt to flash the board:

![](https://raw.githubusercontent.com/TarlogicSecurity/Candleblow/main/doc/atmel2.png)

If the board's LCD displays a message saying `Probe ready :)`, congratulations! You are ready to use Candleblow with PLCTool. 
