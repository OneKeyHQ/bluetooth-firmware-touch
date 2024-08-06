# OneKey Touch Bluetooth Firmware

This repo contains bluetooth firmware for OneKey Touch

The firmware is based on NRF5 SDK 16.0.0, and build with Makefile

## How to build

```shell
# make sure you have cmake, Python 3, amd aarm-none-eabi toolchain available in PATH

# export your OWN key for firmware signing
export BT_SIG_PK=$(cat <<EOF
-----BEGIN EC PRIVATE KEY-----
xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
-----END EC PRIVATE KEY-----
EOF
)

# build binaries first
./make_binaries.sh

# then convert binaries to images with signature
./make_images.sh
```

## License

Please check License.md for details
