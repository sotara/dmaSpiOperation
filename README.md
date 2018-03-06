# dmaSpiOperation
perform SPI communication to a MCP3911 ADC

This code is intended to use the FRDM-K64 SPI0 bus to communicate with a MCP3911 ADC using the development board: 
MCP3911 and PIC18F85K90 Single-Phase Anti-Tamper Energy Meter
which can be found at the following link: http://www.microchip.com/DevelopmentTools/ProductDetails.aspx?PartNO=ard00385

Pulled the reistors: R57, R58, R59, R60, R61, R62, R63
The lines that are jumpered between the FRDM-K64 to MPC3911.
<table>
<tr><th>FRDM-K64</th><th>MCP3911 Test Point</th><th>Name</th>
</tr>
<tr><th>J2-14</th><th>--</th><th>Gnd</th>
</tr>
<tr><th>J1-4 PTC17</th><th>TP1</th><th>Reset</th>
</tr>
<tr><th>J2-8 PTD2</th><th>TP2</th><th>MOSI</th>
</tr>
<tr><th>J2-10 PTD3</th><th>TP3</th><th>MISO</th>
</tr>
<tr><th>J2-12 PTD3</th><th>TP4</th><th>Clk</th>
</tr>
<tr><th>J2-6 PTD0</th><th>TP5</th><th>CS</th>
</tr>
<tr><th>J1-5 PTC1</th><th>TP6</th><th>MCP3911 OSC</th>
</tr>
<tr><th>J1-6 PTB9</th><th>TP7</th><th>Data Ready</th>
</tr>
</table>

Use the power source for the MCP3911 development board.
