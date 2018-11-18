obj-m:=ad7606.o
PWD:=$(shell pwd)
#
KDIR:=/home/liushuang/workspace/TI-SDK/board-support/linux-4.4.41/
all:
	$(MAKE) -C $(KDIR) M=$(PWD)
clean:
	rm -rf *.ko *.order *.symvers *.cmd *.o *.mod.c *.tmp_versions
