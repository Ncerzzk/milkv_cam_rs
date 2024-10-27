duo_path="/home/ncer/duo-buildroot-sdk"
lib_path="$duo_path/middleware/v2/lib"

# -C target-feature=+crt-static \
export RUSTFLAGS="-C link-arg=--sysroot=/home/ncer/duo-buildroot-sdk/host-tools/gcc/riscv64-linux-musl-x86_64/sysroot\ 
 "

cargo zigbuild --target riscv64gc-unknown-linux-musl -Zbuild-std 