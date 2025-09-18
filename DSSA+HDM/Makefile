# DSSA+HDM Makefile

# コンパイラとフラグの設定
CXX = g++
CXXFLAGS = -std=c++17 -O2
TARGET = main
SRC_DIR = src
SOURCE = $(SRC_DIR)/main.cpp

# コンパイル
compile:
	@echo "=== コンパイル中... ==="
	cd $(SRC_DIR) && $(CXX) $(CXXFLAGS) -o ../$(TARGET) main.cpp
	@echo "=== コンパイル完了 ==="

# 実行
run:
	@echo "=== 実行中... ==="
	./$(TARGET)
	@echo "=== 実行完了 ==="

# 可視化
visualize:
	@echo "=== 衝突地点の可視化中... ==="
	cd $(SRC_DIR) && python visualize_collision.py
	@echo "=== 可視化完了 ==="