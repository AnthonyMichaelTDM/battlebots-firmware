# Code for my Battlebot

This repo holds various programs I've written for my battlebot(s?), this includes diagnostic tools, as well as production firmware and such.

## Things to install to get everything working

```sh
cargo-binstall esp-generate espup
espup install -f ./export-esp.sh
```

you can later uninstall with `espup uninstall` or update with `espup update`

Note that you need to source the created `export-esp.sh` file in your shell every time you open a new terminal to be able to use the esp tools.

```sh
. ./export-esp.sh
```

### If using `espflash` (I use it for most things)

```sh
cargo binstall espflash
```

You also need to add yourself to the `dialout` group to be able to access the serial port without sudo:

```sh
sudo usermod -a -G dialout $USER
```

> Note: if on Arch linux, you may need to add yourself to the `uucp` group instead.

### If using `probe-rs` (I use it for testing)

```sh
cargo binstall probe-rs-tools
```

You need to add a udev rule to be able to access the probe without sudo.

Download the rules file from https://probe.rs/files/69-probe-rs.rules and place it in `/etc/udev/rules.d/`, then reload the udev rules with:

```sh
sudo udevadm control --reload
sudo udevadm trigger
```

you may also need to add yourself to the `plugdev` group:

```sh
sudo usermod -a -G plugdev $USER
```

#### To run tests with `probe-rs`

you need to switch the runner in `.cargo/config.toml` to use `probe-rs` instead of `espflash`, then you can run tests with:

```sh
cargo test
```