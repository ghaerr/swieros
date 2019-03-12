#!/bin/sh
rm -f xc xem xeu xmkfs root/etc/os root/etc/sfs.img fs.img

# alternative bootstrap uses existing c compiler binary and host compiled eu.c
gcc -o xeu -O3 -Ilinux -Iroot/lib root/bin/eu.c
./xeu root/bin/c -Iroot/lib -o root/bin/c root/bin/c.c
./xeu root/bin/c -Iroot/lib -o root/etc/mkfs root/etc/mkfs.c
./xeu root/bin/c -Iroot/lib -o root/etc/os root/etc/os.c

# other non-needed stuff to put on file system
./xeu root/bin/c -Iroot/lib -o root/bin/echo root/bin/echo.c
./xeu root/bin/c -Iroot/lib -o root/bin/eu root/bin/eu.c
./xeu root/bin/c -Iroot/lib -o root/bin/eu-org root/bin/eu-org.c

./xeu root/etc/mkfs sfs.img root
mv sfs.img root/etc
./xeu root/etc/mkfs fs.img root

gcc -o xem -O3 -Wno-bitwise-op-parentheses -Wno-parentheses -Ilinux -Iroot/lib root/bin/emsafe.c -lm
./xem -f fs.img root/etc/os
exit

# original bootstrap
gcc -o xc -O3 -m32 -Ilinux -Iroot/lib root/bin/c.c
gcc -o xem -O3 -m32 -Ilinux -Iroot/lib root/bin/em.c -lm
gcc -o xmkfs -O3 -m32 -Ilinux -Iroot/lib root/etc/mkfs.c
./xc -o root/bin/c -Iroot/lib root/bin/c.c
./xc -o root/etc/os -Iroot/lib root/etc/os.c
./xmkfs sfs.img root
mv sfs.img root/etc/.
./xmkfs fs.img root
./xem -f fs.img root/etc/os
