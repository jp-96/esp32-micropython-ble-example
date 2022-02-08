# Getting Started

You can build micropython firmware for esp32 and write to your board to learn micropython programming on esp32 using pymakr, an Visutal Studio Code extension.

## Required

- [git](https://git-scm.com/)
- [node.js](https://nodejs.org/)
- [python](https://www.python.org/)
- [Docker](https://www.docker.com/)
- [Visual Studio Code](https://code.visualstudio.com/)
- [Docker for Visual Studio Code](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker)
- [Visual Studio Code Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- [Pymakr VSC Package](https://marketplace.visualstudio.com/items?itemName=pycom.Pymakr)
- [esptool.py](https://github.com/espressif/esptool)

# Building and flashing

## Building micropython firmware for esp32

1. Open this folder with Visual Studio Code.
1. *(Remote-Containers)* Click `Remote Host` icon at the bottom left bar, and then select `Reopen in Container`.
1. *(Menu)* Select `Term` > `Run Build Task...` to run task, `build - micropython esp32`.
1. *(Remote-Containers)* Click `Remote Host` icon at the bottom left bar, and then select `Reopen Folder Locally`.

The firmware will be compiled in the `.build` folder.

## Flashing firmware using esptool.py

1. Install venv, `python -m venv venv`.
1. Activate venv, `venv\Scripts\activate`.
1. Install esptool.py, `pip install esptool`.
1. Connect esp32, ex. `COM3`.
1. Flashing firmware, with the following command.

```
venv\Scripts\esptool.py -p COM3 -b 460800 --before default_reset --after hard_reset --chip esp32  write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x1000 ./.build/bootloader/bootloader.bin 0x8000 ./.build/partition_table/partition-table.bin 0x10000 ./.build/micropython.bin
```

# Using Pymakr VSC Package

## Global Setting

1. Click `All Commands` at the bottom bar, then select `Pymakr > Global Setting`.
1. Edit the `pymakr.json` configuration file,  
`address` to `""`(empty string),  
`auto_connect` to `true`  
and add the `Silicon Labs` manufacturer (or other) to the `autoconnect_comport_manufacturers` section. 
1. Save the `pymakr.json` configuration file.

## REPL

1. Reopen this folder with Visual Studio Code.
1. Pymakr will automatically detect your esp32.  
If that doesn’t happen, click to toggle the `Pymakr Console` button manually.
1. Type `help()` after the prompt (>>>) on the `TERMINAL` window of `Pymarkr Console` and see your esp32 responding.
1. Try typing the following code to toggle the LED connected to GPIO2 on and off.

```
import machine
pin2 = machine.Pin(2, machine.Pin.OUT)
pin2.value(1)
pin2.value(0)
```
