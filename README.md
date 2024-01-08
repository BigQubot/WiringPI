## How to download wiringPi

```
# apt-get update
# apt-get install -y git
# git clone https://github.com/BigQubot/wiringPi.git
```

## How to build wiringPi

```
# cd wiringPi
# ./build clean
# ./build 
```

---
## The output of the gpio readall command

## Allwinner H618

### Banana Pi M4Berry / M4Zero

```
 +------+-----+----------+--------+---+BPI-M4B/Z +---+--------+----------+-----+------+
 | GPIO | wPi |   Name   |  Mode  | V | Physical | V |  Mode  | Name     | wPi | GPIO |
 +------+-----+----------+--------+---+----++----+---+--------+----------+-----+------+
 |      |     |     3.3V |        |   |  1 || 2  |   |        | 5V       |     |      |
 |  208 |   0 |    SDA.4 |    OFF | 0 |  3 || 4  |   |        | 5V       |     |      |
 |  207 |   1 |    SCL.4 |    OFF | 0 |  5 || 6  |   |        | GND      |     |      |
 |  211 |   2 |    PWM.1 |    OFF | 0 |  7 || 8  | 0 | ALT2   | TXD.1    | 3   | 198  |
 |      |     |      GND |        |   |  9 || 10 | 0 | ALT2   | RXD.1    | 4   | 199  |
 |  226 |   5 |    TXD.5 |   ALT2 | 0 | 11 || 12 | 0 | OFF    | PCM_BCLK | 6   | 203  |
 |  227 |   7 |    RXD.5 |   ALT2 | 0 | 13 || 14 |   |        | GND      |     |      |
 |  194 |   8 |     PG02 |    OFF | 0 | 15 || 16 | 0 | OFF    | RTS.1    | 9   | 200  |
 |      |     |     3.3V |        |   | 17 || 18 | 0 | OFF    | CTS.1    | 10  | 201  |
 |  231 |  11 |   MOSI.1 |   ALT4 | 0 | 19 || 20 |   |        | GND      |     |      |
 |  232 |  12 |   MISO.1 |   ALT4 | 0 | 21 || 22 | 0 | OFF    | PG01     | 13  | 193  |
 |  230 |  14 |   SCLK.1 |   ALT4 | 0 | 23 || 24 | 0 | OFF    | CS0.1    | 15  | 229  |
 |      |     |      GND |        |   | 25 || 26 | 0 | ALT4   | CS1.1    | 16  | 233  |
 |  210 |  17 |    SDA.3 |    OFF | 0 | 27 || 28 | 0 | OFF    | SCL.3    | 18  | 209  |
 |  195 |  19 |     PG03 |    OFF | 0 | 29 || 30 |   |        | GND      |     |      |
 |  196 |  20 |     PG04 |    OFF | 0 | 31 || 32 | 0 | OFF    | PG00     | 21  | 192  |
 |  197 |  22 |     PG05 |    OFF | 0 | 33 || 34 |   |        | GND      |     |      |
 |  204 |  23 | PCM_LRCK |    OFF | 0 | 35 || 36 | 0 | OFF    | SPDIF    | 24  | 228  |
 |  202 |  25 | PCM_MCLK |    OFF | 0 | 37 || 38 | 0 | OFF    | PCM_IN   | 26  | 206  |
 |      |     |      GND |        |   | 39 || 40 | 0 | OFF    | PCM_OUT  | 27  | 205  |
 +------+-----+----------+--------+---+----++----+---+--------+----------+-----+------+
 | GPIO | wPi |   Name   |  Mode  | V | Physical | V |  Mode  | Name     | wPi | GPIO |
 +------+-----+----------+--------+---+BPI-M4B/Z +---+--------+----------+-----+------+
```

