<h1 align = "center">USB自学笔记</h1>

[TOC]

# 第一章 USB介绍 🔌📡🎥🧮💿

## 1.1 USB历史起源

## 1.2 USB协议电气特点以及通信原理

|表头|表头|表头|
|---|---|----|
|单元格|单元格|单元格|
|单元格|单元格|单元格|

'''wavedrom

{signal: [
  {    name: 'clk',   wave: 'p..Pp..P'},
  ['Master',
    ['ctrl',
      {name: 'write', wave: '01.0....'},
      {name: 'read',  wave: '0...1..0'}
    ],
    {  name: 'addr',  wave: 'x3.x4..x', data: 'A1 A2'},
    {  name: 'wdata', wave: 'x3.x....', data: 'D1'   },
  ],
  {},
  ['Slave',
    ['ctrl',
      {name: 'ack',   wave: 'x01x0.1x'},
    ],
    {  name: 'rdata', wave: 'x.....4x', data: 'Q2'},
  ]
]}

'''


```gnuplot {cmd=true output="html"}
set terminal svg
set title "Simple Plots" font ",20"
set key left box
set samples 50
set style data points
plot [-10:10] sin(x),atan(x),cos(atan(x))
```



- [x]
+ 西瓜
+ :smile:
+ 

​    