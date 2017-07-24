Welcome to Mister Mister!

This is the code that is used to run DeMentha's Arduino powered mist machine: Mister Mister.

# Mister Mister Developer Setup

## Clone Repo

Run `git clone git@github.com:DeMentha/MisterMister.git` to clone this git repository.

## Environment Setup
1. Install Arduino IDE (https://www.arduino.cc/en/Guide/MacOSX)
  1. This may not be necessary
2. Install Visual Studio Code
3. Install `platformio-ide`: http://docs.platformio.org/en/latest/ide/vscode.html
4. The installation will ask you to reload VS Code. After reloading, open the MisterMister source code:
  1. Go to File → Open and select the directory where you cloned the MisterMister repo.
5. Enter hotkey `Cmd+Shift+P` and search for `PlatformIO: Initialize or Update Project` and press enter.
6. You should be all set to now build, upload, run and test the code.
## Ensure you have a PIO Account to do the following

We’re using platformIO which is a pretty awesome framework for building Arduino sketches. It’s free to use except for the additional features for Unit testing, Remote Arduino board uploading, etc.

More details here: http://docs.platformio.org/en/latest/ide/vscode.html#pio-account


- Build and Upload (Always free)
- Unit Testing (30 day trial and then $11.99 / month)
## Run on Arduino

Ensure Arduino UNO board connected via USB.

This code has been run and tested on an Arduino UNO board. Cannot verify that this works on other boards.

- Press the `PIO Terminal` button at the button of VS Code (http://docs.platformio.org/en/latest/ide/vscode.html#ide-vscode-toolbar)
- In the terminal run `platformio run -t upload && platformio serialports monitor --baud 9600`
  - This will compile and upload code to the Arduino UNO board
  - Once uploaded, this command will also start the Serial monitor allowing you to immediately view log output.
## Unit Testing

Ensure Arduino UNO board connected via USB.

This code has been run and tested on an Arduino UNO board. Cannot verify that this works on other boards.

- Press the `PIO Terminal` button at the button of VS Code (http://docs.platformio.org/en/latest/ide/vscode.html#ide-vscode-toolbar)
- In the terminal run `pio test -e uno --verbose`
  - This will compile, upload and run the unit tests on the Arduino board.


