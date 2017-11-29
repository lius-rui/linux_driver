obj-m:=eeprom.o
PWD:=$(shell pwd)
#
KDIR:=/home/workspace/IMX6/src/linux-3.14.52/
all:
	$(MAKE) -C $(KDIR) M=$(PWD)
clean:
	rm -rf *.ko *.order *.symvers *.cmd *.o *.mod.c *.tmp_versions
