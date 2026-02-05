# Needs

You need Python 3.x, BladeRF library, and Rustup installed.

https://www.python.org/downloads/

https://www.nuand.com/

https://rustup.rs/

# How

First, open a prompt and type `python bladesdr.py --serial 9da --freq-offset 0`. But, replace `9da` with the serial of your BladeRF. If this works it means
you have Python 3.x and the BladeRF library installed correctly.

Next, open a prompt and type `cargo run --release -- --thread-count 16 --cycle-count 20`. At first, it will build the libraries and the application.

Finally, you should see messages about elapsed and buffer time. If it says `TOO SLOW` reduce the `--cycle-count`. At 5 second intervals it will print the statistics.

# Map

Thanks to obj!

You can use antirez's dump1090 as a map. The way you do it is first clone the repository. Make sure you have `Make`, `rtl-sdr-devel`, and `GCC` installed on Linux. If you do not have a Linux machine you can create a virtual machine using the free Oracle's VirtualBox. Next go into the directory that you cloned and type `make`. This will build the program. The program will be in the same directory under the name `dump1090`.

https://github.com/antirez/dump1090/

Once the program is built you can use the command line `./dump1090 --net-only`. After that, start the Python frontend for the BladeSDR with `python bladesdr.py --serial 9da --freq-offset 0`. Replace serial with the appropriate serial number for your card. Next type something like `cargo run --release -- --thread-count 16 --cycle-count 20 --net-raw-out 192.168.0.89:30001`. Replace the `192.168.0.89` with the IP address that dump1090 is running on. It might be `localhost` or an IP address.

Finally, open your web browser and visit `http://localhost:8080` and replace `localhost` with the IP address of the dump1090 instance that you started up. This will load a map that will display the planes.

# Two BladeSDR Boards

To use two BladeSDR boards, you need a micro SMB cable to connect the `CLKOUT` of the master card to the `CLKIN` of the slave. Next, you need a small jumper wire with female ends to connect pin J51[1] of the two
boards together. Then of course, you need your antennas setup which will be four of them.

Then you run `bladesdr4x.py`. This will require two serial numbers since you need two boards. Pick one to be the master and the other the slave. The master should be the one with the cable connected to `CLKOUT` and the slave the one with the cable connected to `CLKIN`.

To find pin `J51[1]` first turn the board so the stenciled lettering is oriented where you can read it. Now,
look for the JTAG connector. The JTAG connector is ten pins oriented in two rows of five pins each. The `J51` connector is right above it and above it you will see the tiny letters J51. The first pin is on the left side and that is `J51[1]`. You can use any wire. I liked the little jumper wires that have a female end that fits nicely over the pin. You link both of these pins on both cards. This is the trigger pin. The master toggles the pin and this tells both cards to start streaming at the same instant. Well, it's close to the same instant but not perfect because obviously the electrical signal has a propogation speed.

If you are looking for the `CLKIN` and `CLKOUT`. Hold the board the same way and look near the top center and they are beside each other. Make sure the master is `CLKOUT` because the program is going to configure it to output the clock signal and the slave will be configured to read the clock signal.