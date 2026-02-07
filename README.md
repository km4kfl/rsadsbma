# What Does It Do

This project demodulates, partially decodes, and does CRC checking. It allows one to recieve ADS-B transmission on 1090MHZ. These are transmissions sent by airplanes that fly high in the sky. You can
use it to track these aircraft. It enables you to also use dump1090, another project, which provides
a real-time map. See the section Map for more information.

The purpose of this project was not to replace the many others but to do some experimenting with
beamforming which is where the signals from two or more antennas, part of an array, are mixed 
together to form directivity. The directivity allows software to aim at different parts of the sky
for improved SNR and when it works well allows one to find, demodulate, and decode messages that
would have otherwise been lost to a single antenna system.

Here are some other projects which are more full featured.

https://github.com/wiedehopf/readsb

https://github.com/mgrone/stream1090

https://github.com/antirez/dump1090

https://github.com/flightaware/dump1090

You can also search the Internet for ADS-B, RTL-SDR, and you should find many articles, how tos, and
other things to read about the activity.

# Discussion

https://discussions.flightaware.com/

You can find a lot of people who can offer help with hardware. This project was designed primarily
for the BladeSDR but it could be used with other cards that support multiple coherent channels simply
by changing the frontend script. However, at the link above you will find resources for using other
cheaper cards for ADS-B.

# Needs

You need Python 3.x, BladeRF library, and Rustup installed.

https://www.python.org/downloads/

https://www.nuand.com/

https://rustup.rs/

# How

First, open a prompt and type `python bladesdr.py --serial 9da --freq-offset 0`. But, replace `9da` with the serial of your BladeRF. If this works it means
you have Python 3.x and the BladeRF library installed correctly.

Next, open a prompt and type `cargo run --release -- --thread-count 16 --cycle-count 20`. At first, it will build the libraries and the application. However, change `--thread-count` to be a number equal to the number of cores you have in the system! Also, you may have to adjust `--cycle-count`.

Finally, you should see messages about elapsed and buffer time. If it says `TOO SLOW` reduce the `--cycle-count`. At 5 second intervals it will print the statistics.

# Antenna Placement

You want each antenna to be within about half a wavelength. Since the current state of the program uses
a random approach to beamforming you can setup the antennas in any kind of configuration. It may take
some testing to find an optimal setup. I like the random approach because it isn't dependent on how you
setup the antennas and it seems to outperform a single antenna setup by a significant factor. However,
obviously it isn't the most optimal solution. 

So feel free to setup the antenna as a uniform linear array, circular, or in a 2D configuration and
then let me know the results at kburtes42@proton.me.

# Map

Thanks to obj!

You can use antirez's dump1090 as a map. The way you do it is first clone the repository. Make sure you have `Make`, `rtl-sdr-devel`, and `GCC` installed on Linux. If you do not have a Linux machine you can create a virtual machine using the free Oracle's VirtualBox. Next go into the directory that you cloned and type `make`. This will build the program. The program will be in the same directory under the name `dump1090`.

https://github.com/antirez/dump1090/

Once the program is built you can use the command line `./dump1090 --net-only`. After that, start the Python frontend for the BladeSDR with `python bladesdr.py --serial 9da --freq-offset 0`. Replace serial with the appropriate serial number for your card. Next type something like `cargo run --release -- --thread-count 16 --cycle-count 20 --net-raw-out 192.168.0.89:30001`. Replace the `192.168.0.89` with the IP address that dump1090 is running on. It might be `localhost` or an IP address.

Finally, open your web browser and visit `http://localhost:8080` and replace `localhost` with the IP address of the dump1090 instance that you started up. This will load a map that will display the planes.

# Two BladeSDR Boards

Using two boards allows you to have four coherent RX streams instead of only two. It should be possible to connect any number of boards together and you are not limited to only two.

To use two BladeSDR boards, you need a micro SMB cable to connect the `CLKOUT` (clock out) of the master card to the `CLKIN` (clock in) of the slave. Next, you need a small jumper wire with female ends to connect pin J51[1] of the two
boards together. Then of course, you need your antennas setup which will be four of them.

Then you run `bladesdr4x.py`. This will require two serial numbers since you need two boards. Pick one to be the master and the other the slave. The master should be the one with the cable connected to `CLKOUT` and the slave the one with the cable connected to `CLKIN`.

To find pin `J51[1]` first turn the board so the stenciled lettering is oriented where you can read it. Now,
look for the JTAG connector. The JTAG connector is ten pins oriented in two rows of five pins each. The `J51` connector is right above it and above it you will see the tiny letters J51. The first pin is on the left side and that is `J51[1]`. You can use any wire. I liked the little jumper wires that have a female end that fits nicely over the pin. You link both of these pins on both cards. This is the trigger pin. The master toggles the pin and this tells both cards to start streaming at the same instant. Well, it's close to the same instant but not perfect because obviously the electrical signal has a propogation speed.

If you are looking for the `CLKIN` (clock in) and `CLKOUT` (clock out). Hold the board the same way and look near the top center and they are beside each other. Make sure the master is `CLKOUT` because the program is going to configure it to output the clock signal and the slave will be configured to read the clock signal.

https://www.nuand.com/libbladeRF-doc/v2.5.0/group___f_n___t_r_i_g.html

https://www.nuand.com/libbladeRF-doc/v2.5.0/group___f_n___b_l_a_d_e_r_f2___l_o_w___l_e_v_e_l___c_l_o_c_k___b_u_f_f_e_r___o_u_t_p_u_t.html

# ULA Mode

It supports a uniform linear array (ULA) mode with the command line option `--ula-spacing-wavelength`. This activates the mode
when used and it treats the antennas if they are an array with the same spacing between each element. The argument to this
command line option is the wavelength of the spacing. For example, if you antennas are spaced at half a wavelength then the
value would be `0.5`. You will have to calculate the wavelength by hand into meters with `300e6 / 1090e6` then if you wanted
a half wavelength spacing you could do `300e6 / 1090e6 * 0.5` which would give you the distance the elements need to be spaced
at half a wavelength in meters. The `0.5` is the wavelength spacing argument for `--ula-spacing-wavelength`.

This mode is in contrast to the default random mode where the antenna weights are randomly selected.