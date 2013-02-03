zt280-kernel
============

Kernel source for Zenithink C91 Tablet (Board variants: H2 1a, G0 2n, H1 2f, H1 2n, H2, K0 2n)

Make sure you got the arm-none-linux-gnueabi compiler from amlogic, put it where you like it and enter this path in the env.sh file.

Enable your board version by uncommenting the corresponding lines in maketest.sh,
pull a config from your running tablet (proc/config.gz) and copy it in the root folder as .config.

Also copy the ramdisk from your device, preferably by unpacking the *.kernel and *.recovery files with the supplied scripts.
Move ramdisk content to a new folder in device and edit makeone_recovery and makeone_release accordingly to include your files.

run ". env.sh" from terminal,
run makemenuconfig, load the .config file and configure to your needs.
run ./maketest.sh

compiled kernel binary output is in out/yourboard/subversion.

This device has been deprecated and there is no further support available. I have managed to compile my own kernel to improve performance noticably,
but your mileage may vary. Also, Zenithink did release really helpful up to date source codes, but rather a mess of strange own changes to the AMLogic source code.

Also, please be aware that the ICS kernel is compiled from the GB source tree...

I stopped development on this device because of unsufficient manufacturer support. Perhaps anyone is able to use this source to create something for this aging device...
