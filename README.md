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
