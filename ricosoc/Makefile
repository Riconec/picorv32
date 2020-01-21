CROSS=riscv32-unknown-elf-
CFLAGS=
SRC_LIST= defines.v ice40feather.v bootloader_ram.v ../picosoc/ice40up5k_spram.v ../picosoc/simpleuart.v ../picosoc/spimemio.v ricosoc.v ../picorv32.v

# ---- iCE40 Feather Board ----

ice40feather_tb.vvp: ice40feather_tb.v ../picosoc/spiflash.v $(SRC_LIST)
	iverilog -s testbench -o $@ $^ `yosys-config --datdir/ice40/cells_sim.v`

icefsim: ice40feather_tb.vvp bootloader_fw.hex bootloader_fw.hex ice40feather_fw.hex ice40feather_fw.lss
	vvp -N $<

icefsynsim: ice40feather_syn_tb.vvp ice40feather_fw.hex
	vvp -N $< +firmware=ice40feather_fw.hex

ice40feather_syn_tb.vvp: ice40feather_tb.v ice40feather_syn.v 
	iverilog -s testbench -o $@ $^ `yosys-config --datdir/ice40/cells_sim.v`

ice40feather_syn.v: ice40feather.json
	yosys -p 'read_json ice40feather.json; write_verilog ice40feather_syn.v'


ice40feather.json: $(SRC_LIST)
	yosys -ql ice40feather.log  -DPICOSOC_MEM=ice40up5k_spram -p 'synth_ice40 -top ice40feather -json ice40feather.json' $^

ice40feather.asc: ice40feather.pcf ice40feather.json
	nextpnr-ice40 --freq 13 --up5k --package sg48 --asc ice40feather.asc --pcf ice40feather.pcf --json ice40feather.json

ice40feather.bin: ice40feather.asc ice40feather_fw.hex
	icetime -d up5k -c 12 -mtr ice40feather.rpt ice40feather.asc
	icepack ice40feather.asc ice40feather.bin

icefprog: ice40feather.bin ice40feather_fw.bin
	iceprog ice40feather.bin
	iceprog -o 1M ice40feather_fw.bin

icefprog_fw: ice40feather_fw.bin
	iceprog -o 1M ice40feather_fw.bin


# firmware part 

ice40feather_sections.lds: sections.lds
	$(CROSS)cpp -P -DICE40FEATHER -o $@ $^

ice40feather_fw.elf: ice40feather_sections.lds start.s main.c#rom_boot.s firmware_feather.c #rom_boot.s firmware_feather.c
	$(CROSS)gcc $(CFLAGS) -DICE40FEATHER -march=rv32i -Wl,-Bstatic,-T,ice40feather_sections.lds,--strip-debug -ffreestanding -nostdlib -o ice40feather_fw.elf start.s main.c 

ice40feather_fw.hex: ice40feather_fw.elf
	elf2hex --bit-width 32 --input ice40feather_fw.elf --output ice40feather_fw.hex
	cp ice40feather_fw.hex firmware.hex

ice40feather_fw.lss: ice40feather_fw.elf
	$(CROSS)objdump -S ice40feather_fw.elf > ice40feather_fw.lss
	$(CROSS)objdump -t ice40feather_fw.elf > ice40feather_fw.map

ice40feather_fw.bin: ice40feather_fw.elf ice40feather_fw.lss
	$(CROSS)objcopy -O binary ice40feather_fw.elf ice40feather_fw.bin

# bootloader fw part

bootloader_fw.elf: ice40feather_sections.lds bootrom.s
	$(CROSS)gcc $(CFLAGS) -DICE40FEATHER -march=rv32i -Wl,-Bstatic,-T,ice40feather_sections.lds,--strip-debug -ffreestanding -nostdlib -o bootloader_fw.elf bootrom.s

bootloader_fw.hex: bootloader_fw.elf
	elf2hex --bit-width 32 --input bootloader_fw.elf --output bootloader_fw.hex
	$(CROSS)objdump -S bootloader_fw.elf > bootloader_fw.lss

bootloader_fw.lss: bootloader_fw.elf
	$(CROSS)objdump -S bootloader_fw.elf > bootloader_fw.lss

# bootloader debug part

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

clean:
	rm -f ice40feather_fw.elf ice40feather_fw.hex ice40feather_fw.lss ice40feather_sections.lds ice40feather_fw.map ice40feather_tb.vvp
	rm -f bootloader_fw.elf bootloader_fw.hex bootloader_fw.lss firmware.hex
	rm -f ice40feather.bin ice40feather.asc ice40feather.json ice40feather.log ice40feather.rpt
