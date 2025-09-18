# Code for my Battlebot

This repo holds various programs I've written for my battlebot(s?), this includes diagnostic tools, as well as production firmware and such.

## Things to install to get everything working

```sh
cargo-binstall esp-generate espflash espup
espup install -f ./export-esp.sh
```

you can later uninstall with `espup uninstall` or update with `espup update`

Note that you need to source the created `export-esp.sh` file in your shell every time you open a new terminal to be able to use the esp tools.

```sh
. ./export-esp.sh
```

You also need to add yourself to the `dialout` group to be able to access the serial port without sudo:

```sh
sudo usermod -a -G dialout $USER
```

> Note: if on Arch linux, you may need to add yourself to the `uucp` group instead.
