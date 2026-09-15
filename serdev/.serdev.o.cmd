cmd_/home/u-yoimiya/文档/mems_driver/serdev/serdev.o := /home/u-yoimiya/rk3588/kernel/scripts/gcc-wrapper.py aarch64-linux-gnu-gcc -Wp,-MMD,/home/u-yoimiya/文档/mems_driver/serdev/.serdev.o.d -nostdinc -isystem /usr/lib/gcc-cross/aarch64-linux-gnu/13/include -I./arch/arm64/include -I./arch/arm64/include/generated  -I./include -I./arch/arm64/include/uapi -I./arch/arm64/include/generated/uapi -I./include/uapi -I./include/generated/uapi -include ./include/linux/kconfig.h -include ./include/linux/compiler_types.h -D__KERNEL__ -mlittle-endian -DCC_USING_PATCHABLE_FUNCTION_ENTRY -DKASAN_SHADOW_SCALE_SHIFT= -fmacro-prefix-map=./= -Wall -Wundef -Werror=strict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -fshort-wchar -fno-PIE -Werror=implicit-function-declaration -Werror=implicit-int -Werror=return-type -Wno-format-security -std=gnu89 -mgeneral-regs-only -DCONFIG_CC_HAS_K_CONSTRAINT=1 -Wno-psabi -mabi=lp64 -fno-asynchronous-unwind-tables -fno-unwind-tables -mbranch-protection=pac-ret+leaf -Wa,-march=armv8.5-a -DARM64_ASM_ARCH='"armv8.5-a"' -DKASAN_SHADOW_SCALE_SHIFT= -fno-delete-null-pointer-checks -Wno-frame-address -Wno-format-truncation -Wno-format-overflow -Wno-address-of-packed-member -O2 -fno-allow-store-data-races -Wframe-larger-than=2048 -fstack-protector-strong -Werror -Wimplicit-fallthrough -Wno-unused-but-set-variable -Wno-unused-const-variable -fno-omit-frame-pointer -fno-optimize-sibling-calls -g -fpatchable-function-entry=2 -Wdeclaration-after-statement -Wvla -Wno-pointer-sign -Wno-stringop-truncation -Wno-zero-length-bounds -Wno-array-bounds -Wno-stringop-overflow -Wno-restrict -Wno-maybe-uninitialized -fno-strict-overflow -fno-stack-check -fconserve-stack -Werror=date-time -Werror=incompatible-pointer-types -Werror=designated-init -Wno-packed-not-aligned -mstack-protector-guard=sysreg -mstack-protector-guard-reg=sp_el0 -mstack-protector-guard-offset=1336 -I/home/u-yoimiya/文档/mems_driver/core -I/home/u-yoimiya/文档/mems_driver/data -I/home/u-yoimiya/文档/mems_driver/protocol -I/home/u-yoimiya/文档/mems_driver/sensor -I/home/u-yoimiya/文档/mems_driver/serdev  -DMODULE  -DKBUILD_BASENAME='"serdev"' -DKBUILD_MODNAME='"ssf_mems"' -D__KBUILD_MODNAME=kmod_ssf_mems -c -o /home/u-yoimiya/文档/mems_driver/serdev/serdev.o /home/u-yoimiya/文档/mems_driver/serdev/serdev.c

source_/home/u-yoimiya/文档/mems_driver/serdev/serdev.o := /home/u-yoimiya/文档/mems_driver/serdev/serdev.c

deps_/home/u-yoimiya/文档/mems_driver/serdev/serdev.o := \
  include/linux/kconfig.h \
    $(wildcard include/config/cc/version/text.h) \
    $(wildcard include/config/cpu/big/endian.h) \
    $(wildcard include/config/booger.h) \
    $(wildcard include/config/foo.h) \
  include/linux/compiler_types.h \
    $(wildcard include/config/have/arch/compiler/h.h) \
    $(wildcard include/config/enable/must/check.h) \
    $(wildcard include/config/cc/has/asm/inline.h) \
  include/linux/compiler_attributes.h \
  include/linux/compiler-gcc.h \
    $(wildcard include/config/arm64.h) \
    $(wildcard include/config/retpoline.h) \
    $(wildcard include/config/arch/use/builtin/bswap.h) \
    $(wildcard include/config/kcov.h) \
  arch/arm64/include/asm/compiler.h \

/home/u-yoimiya/文档/mems_driver/serdev/serdev.o: $(deps_/home/u-yoimiya/文档/mems_driver/serdev/serdev.o)

$(deps_/home/u-yoimiya/文档/mems_driver/serdev/serdev.o):
