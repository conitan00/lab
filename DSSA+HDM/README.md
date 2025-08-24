# DSSA+HDM

## 実行方法

```bash
# コンパイル
g++ -std=c++17 -O2 -o main main.cpp

# 実行
main.exe
```

## 設定

`config.h`で設定を変更可能

## 衝突地点の可視化

```bash
python visualize_collision.py
```

衝突地点を地図上にプロットし、`collision_map.png`として保存