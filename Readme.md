Coexistance tester for Ble

This code is based on the Silicon Labs Throughput App running on the EFR32BG13P632F512GM48, WSTK + BRD4104A with Gecko SDK version 2.12.3.0 and it was built to be able to text the Wifi Coexistence as described on AN1128. The Coexistence pins are:

Request PC10
Grant PF3
Priority PD12
RHO PC11

Based on the Silicon Labs Throughput App KBA:

https://www.silabs.com/community/wireless/bluetooth/knowledge-base.entry.html/2017/11/21/throughput_testerex-octu

AN1128:

https://www.silabs.com/documents/public/application-notes/an1128-bluetooth-coexistence-with-wifi.pdf

Tools and SDKs:

This was tested with:

Silicon Labs Bluetooth Stack 2.12.3.0

Gecko SDK Suite v2.6.3

GNU GCC Compiler: Apple LLVM version 9.0.0 (clang-900.0.39.2) Target: x86_64-apple-darwin16.7.0 Thread model: posix

Development Board: Silicon Labs WSTK with BRD4181A (EFR32MG21A010F1024IM32)

Usage:

Clone this to your Simplicity Studio Workspace and import with the option "Import existing projects"

It should compile as is.
