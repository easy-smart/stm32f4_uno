# STM32F4 Uno ethtest

This test program is used to test the ethernet shield. FreeRTOS is used to enable thread support (required for sockets api).

Read the EUI-48 from the SST26 and initializes ethernet. LWIP requests an IP from DHCP.

