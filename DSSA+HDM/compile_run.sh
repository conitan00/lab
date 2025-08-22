#!/bin/bash

# コンパイル・実行統合スクリプト
CXX=g++
CXXFLAGS="-std=c++17 -O2 -Wall -Wextra"
TARGET=main
SOURCE=main.cpp

echo "==== コンパイル開始 ===="
$CXX $CXXFLAGS -o $TARGET $SOURCE

if [ $? -eq 0 ]; then
    echo "コンパイル成功"
    
    # 実行権限付与
    chmod +x $TARGET
    
    echo ""
    echo "==== プログラム実行 ===="
    ./$TARGET
    
    echo ""
    echo "==== 実行完了 ===="
else
    echo "コンパイルエラー"
    exit 1
fi