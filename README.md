# MVisor: A mini x86 hypervisor

## Goal

1. A minimal hypervisor based on KVM and x86 (replace QEMU)
2. A limited number of emulated devices (support plugins in later version)
3. Linux and Windows as guest VMs
4. VFIO (especially vGPU) and migration
5. Extremely stable and high performance


## Screenshot

### Ubuntu

<img src="./docs/ubuntu.jpg" width="640">

### vGPU

<img src="./docs/vgpu.jpg" width="640">

### Multimedia

<img src="./docs/multimedia.jpg" width="640">



## Roadmap And Current Status

What's supported now:

### Basic functions

1. 440FX ✅ / Q35 Chipset ✅
2. SeaBIOS ✅
3. Memory Region Management ✅
4. IOPort Management ✅
5. Devices Management ✅
6. RTC (CMOS) ✅
7. PS/2 ✅
8. PCI ISA ICH9-LPC ✅
9. QEMU CFG ✅
10. Legacy DMA ✅
11. IDE ✅ / AHCI ✅
12. Floppy Disk ✅
13. Serial Port ✅
14. VGA / VBE ✅
15. Option Roms ✅ / SMBIOS ✅ / ACPI Table ✅
16. Boot DOS ✅
17. Boot OS (Win98 to Win11 / DOS / Ubuntu) ✅
18. QCOW2 ✅

### Multimedia & Networking

1. Virtio (Console ✅ / Block ✅ / Net ✅ / VirtioFS ✅ / VGPU ✅ / CUDA ✅ / Balloon)
2. SpiceAgent ✅
3. QemuGuestAgent ✅
4. Qxl ✅
5. Audio (ICH9-HDA / AC97) ✅
6. Tap network ✅
7. User network ✅
8. VFIO (mdev & passthrough) ✅
9. Samba
10. USB 1.0 UHCI ✅ / USB 3.0 XHCI ✅ / USB Tablet ✅ / USB Midi ✅ / USB Wacom ✅

### Hyper-V & Migration

1. CPU migration ✅
2. VFIO migration ✅
3. Migration to sparse files ✅
4. Hyper-V enlightenments ✅
5. Live migration ✅

## Compile & Run

### For RockyLinux 9.3,

```bash
dnf install epel-release gdb cmake gcc-c++ acpica-tools
dnf --enablerepo=devel install -y protobuf-compiler protobuf-devel glib2-devel yaml-cpp-devel pixman-devel libzstd-devel zlib-devel

# If SDL enabled
dnf --enablerepo=devel install -y SDL2-devel alsa-lib-devel
```

### For Debian 12,

```bash
apt install meson gdb cmake build-essential g++ acpica-tools
apt install protobuf-compiler libprotobuf-dev libglib2.0-dev libyaml-cpp-dev libpixman-1-dev libzstd-dev zlib1g-dev

# If SDL enabled
apt install libsdl2-dev libasound2-dev
```

### Compile and Run

```bash
meson setup build -Dsdl=true # SDL is disabled by default
meson compile -C build

./build/mvisor -c config/sample.yaml -vnc 5900
```

## Paravirtualized Drivers
An ISO image file is needed to install OS. Edit YAML file to configure image path.

Virtio is recommended for Windows guests:

<a href="https://fedorapeople.org/groups/virt/virtio-win/direct-downloads/stable-virtio/virtio-win.iso">Download Virtio Guest Tools</a>
