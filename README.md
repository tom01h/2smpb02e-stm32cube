# 2smpb02e-st32cube

オムロン製センサ 2SMPB-02E を STM32CubeIDE で評価する為のライブラリです。  
[Arduino向けライブラリ](https://github.com/omron-devhub/2smpb02e-grove-arduino) をもとに変更しています。

2SMPB-02E は高精度・低消費電流の小型MEMS絶対圧センサです。  
低ノイズの24bit ADCを内蔵し、気圧を高精度に測定することができます。  
制御と出力は I2C/SPI インターフェースを通じたデジタル方式で、自動スリープモードによる低消費電流を実現しています。

## 変更元のREADME

- [英語](./OMRON_README.md)
- [日本語](./OMRON_README_ja.md)

## 使い方

`examples/sample/main_cpp.cpp` を参照してください。

## ライセンス

MITライセンスの下でライセンスされます。
