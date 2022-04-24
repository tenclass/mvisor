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

Milestone 1: Boot OS
Nov 2021

1. Multi vCPU ✅
2. Load SeaBIOS ✅
3. Memory Region Management ✅
4. IOPort Management ✅
5. Devices Management ✅
6. RTC(CMOS) ✅
7. PS/2 ✅
8. PCI ISA ICH9-LPC ✅
9. QEMU CFG ✅
10. Legacy DMA ❌
11. IDE ❌ / AHCI ✅
12. Parallel Port ❌
13. Serial Port ✅
14. VGA / VBE ✅
15. Option Roms ✅ / SMBIOS ✅ / ACPI Table ✅
16. Boot DOS ✅
17. Boot OS (Windows 8 or 10) ✅
18. QCOW2 ✅

Milestone 2: Multimedia & Networking
Feb 2022

1. Virtio (Console ✅ / Block ✅ / Net ✅ / Balloon)
2. SpiceAgent ✅
3. QemuGuestAgent ✅
4. Qxl ✅
5. Audio (ICH9-HDA) ✅
6. Tap network
7. User network ✅
8. SfAgent
9. VFIO ✅
10. Samba
11. XHCI ✅ / USB Tablet ✅

Milestone 3: Migration
Mar 2022
1. CPU migration ✅
2. VFIO migration ✅
3. Migration to compressed file
4. Hyper-V enlightenments ✅

Milestone 4: Virtualization Management
Apr 2022
1. Sweet server ✅
2. Audio playback ✅
3. Opus audio streaming ✅
4. H264 display streaming ✅
5. JPEG screendump ✅


## Install

For CentOS 8,

```
yum install -y gcc-c++ SDL2-devel yaml-cpp-devel libuuid-devel protobuf-devel protobuf-compiler libzstd-devel zlib-devel alsa-lib-devel
yum install -y libyuv-devel x264-devel opus-devel
make -j8
```

An ISO image file is needed to install OS. Modify config/default.yaml to configure image path.

Virtio is recommended:

<a href="https://fedorapeople.org/groups/virt/virtio-win/direct-downloads/stable-virtio/virtio-win.iso">Downlaod Virtio Guest Tools</a>
