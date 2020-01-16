bootloader_fw.elf: ice40feather_sections.lds rom_boot.s
	$(CROSS)gcc $(CFLAGS) -DICE40FEATHER -march=rv32ic -Wl,-Bstatic,-T,ice40feather_sections.lds,--strip-debug -ffreestanding -nostdlib -o bootloader_fw.elf rom_boot.s

bootloader_fw.hex: bootloader_fw.elf
	$(CROSS)objcopy -O verilog bootloader_fw.elf bootloader_fw.hex
	$(CROSS)objdump -S bootloader_fw.elf >> bootloader_fw.lss

bootloader_fw.lss: bootloader_fw.elf
	$(CROSS)objdump -S bootloader_fw.elf >> bootloader_fw.lss

bootloader_ram.bin: bootloader_ram.asc
	icetime -d up5k -c 12 -mtr bootloader_ram.rpt bootloader_ram.asc
	icepack bootloader_ram.asc bootloader_ram.bin

bootloader_ram.asc: bootloader_ram.json
	nextpnr-ice40 --freq 13 --up5k --package sg48 --asc bootloader_ram.asc --pcf ice40feather.pcf --json bootloader_ram.json --pcf-allow-unconstrained

bootloader_ram.json: bootloader_ram.v
	yosys -ql bootloader_ram.log -p 'synth_ice40 -top rom_test -json bootloader_ram.json' $^

sim: bootloader_ram_tb.vvp

runsim: sim
	vvp -N bootloader_ram_tb.vvp

view:
	gtkwave testbench_rom.vcd

bootloader_ram_tb.vvp: bootloader_ram_tb.v bootloader_ram.v
	iverilog -s rom_tb -o $@ $^ `yosys-config --datdir/ice40/cells_sim.v`

