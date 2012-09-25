zt280-kernel
============

Kernel source for Zenithink C91 Tablet (Board variants: H2 1a, G0 2n, H1 2f, H1 2n, H2, K0 2n)

Make sure you got the arm-none-linux-gnueabi compiler from amlogic, put it where you like it and enter this path in the env.sh file.

Enable your board version by uncommenting the corresponding lines in maketest.sh,
pull a config from your running tablet (proc/config.gz) and copy it in the root folder as .config.

run ". env.sh" from terminal,
run makemenuconfig, load the .config file and configure to your needs.
run ./maketest.sh

kernel output is in out/yourboard/subversion.
