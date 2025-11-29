mode i :

i = 0; --> 原地静止

i = 1; --> 只参与巡线

i = 2; --> 巡线 + 避障

i = 3; --> 巡线 + 过减速带

i = 4; --> 转向 + 回正 （写死）

比赛流程：

session 1: 
```原地静止30s```

mode 0 (time.now -time.start)tosec = 30

session 2:
```巡线，避障```

mode 2 

session 3:
```巡线，过减速带```

mode 3

session 4:
```斑马线前停车，检测指示牌，确定左右转```

mode 0 

session 5:
```根据指示牌信息进行写死的左右转```

mode 4

其余时间均作巡线处理：

mode 1