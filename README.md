# radio-paella
(micro)python interface to different radio modules

## current hardware
|hardware   |   status  |
|:-         |:-:        |
|esp8266    |working*   |
|esp32      |working    |
|SAMD51     |WIP        |
|STM32      |WIP        |
\* memory issues, need to move to frozen module

#### other rf chips to eventually support

|chip      |ask|ook|2fsk|2gfsk|4fsk|msk|gmsk|psk|lora|note|
|:--       |:-:|:-:|:-: |:-:  |:-: |:-:|:-: |:-:|:-: |:-  |
|cc1101    |Y  |Y  |Y   |Y    |Y   |Y  |    |   |    |WIP |
|RFM69     |   |Y  |Y   |Y    |    |Y  |Y   |   |    |WIP |
|SX1276    |   |Y  |Y   |Y    |    |Y  |Y   |   |Y   |    |
|SPIRIT1   |Y  |Y  |Y   |Y    |    |Y  |Y   |   |    |find dev board|
|nRF905    |   |   |    |Y    |    |   |    |   |    |    |
|Si4432    |   |Y  |Y   |Y    |    |   |    |   |    |find dev board|
|ATA8510   |Y  |   |Y   |     |    |   |    |   |    |find dev board|
|generic   |Y? |Y  |    |     |    |   |    |   |    |actually possible?*|

\* saw resonator exists, should be possible 