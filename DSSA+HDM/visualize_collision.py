#!/usr/bin/env python3
"""
衝突地点可視化スクリプト
DSSA+HDMプロジェクトの衝突データ（collision.csv）を可視化します
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import argparse
from pathlib import Path

def load_collision_data(csv_path):
    """collision.csvファイルを読み込む"""
    try:
        df = pd.read_csv(csv_path)
        print(f"データ読み込み完了: {len(df)}件の衝突地点")
        print(f"X座標範囲: {df['x'].min():.1f} - {df['x'].max():.1f}")
        print(f"Y座標範囲: {df['y'].min():.1f} - {df['y'].max():.1f}")
        return df
    except Exception as e:
        print(f"エラー: ファイルの読み込みに失敗しました - {e}")
        return None

def create_basic_plot(df, output_path=None):
    """基本的な散布図を作成"""
    plt.figure(figsize=(10, 8))
    plt.scatter(df['x'], df['y'], alpha=0.6, s=20, c='red', marker='o')
    plt.xlabel('X座標')
    plt.ylabel('Y座標')
    plt.title('衝突地点の分布')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    if output_path:
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"基本プロット保存: {output_path}")
    plt.show()

def create_density_plot(df, output_path=None):
    """密度ヒートマップを作成"""
    plt.figure(figsize=(12, 8))
    
    # ヒートマップを作成
    plt.hist2d(df['x'], df['y'], bins=50, cmap='hot', alpha=0.8)
    plt.colorbar(label='衝突頻度')
    plt.xlabel('X座標')
    plt.ylabel('Y座標')
    plt.title('衝突地点の密度分布')
    
    if output_path:
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"密度プロット保存: {output_path}")
    plt.show()

def create_statistics_plot(df, output_path=None):
    """統計情報付きプロット"""
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 12))
    
    # 散布図
    ax1.scatter(df['x'], df['y'], alpha=0.6, s=15, c='red')
    ax1.set_xlabel('X座標')
    ax1.set_ylabel('Y座標')
    ax1.set_title('衝突地点分布')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # X座標のヒストグラム
    ax2.hist(df['x'], bins=30, alpha=0.7, color='blue', edgecolor='black')
    ax2.set_xlabel('X座標')
    ax2.set_ylabel('頻度')
    ax2.set_title('X座標の分布')
    ax2.grid(True, alpha=0.3)
    
    # Y座標のヒストグラム
    ax3.hist(df['y'], bins=30, alpha=0.7, color='green', edgecolor='black')
    ax3.set_xlabel('Y座標')
    ax3.set_ylabel('頻度')
    ax3.set_title('Y座標の分布')
    ax3.grid(True, alpha=0.3)
    
    # 密度プロット
    ax4.hist2d(df['x'], df['y'], bins=30, cmap='YlOrRd')
    ax4.set_xlabel('X座標')
    ax4.set_ylabel('Y座標')
    ax4.set_title('衝突密度')
    
    plt.tight_layout()
    
    if output_path:
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"統計プロット保存: {output_path}")
    plt.show()

def print_statistics(df):
    """統計情報を表示"""
    print("\n=== 衝突データ統計情報 ===")
    print(f"総衝突数: {len(df)}")
    print(f"\nX座標統計:")
    print(f"  平均: {df['x'].mean():.2f}")
    print(f"  標準偏差: {df['x'].std():.2f}")
    print(f"  最小値: {df['x'].min():.2f}")
    print(f"  最大値: {df['x'].max():.2f}")
    print(f"\nY座標統計:")
    print(f"  平均: {df['y'].mean():.2f}")
    print(f"  標準偏差: {df['y'].std():.2f}")
    print(f"  最小値: {df['y'].min():.2f}")
    print(f"  最大値: {df['y'].max():.2f}")

def main():
    parser = argparse.ArgumentParser(description='衝突地点データの可視化')
    parser.add_argument('csv_file', nargs='?', 
                       default='data/20250730_164933/collision.csv',
                       help='collision.csvファイルのパス')
    parser.add_argument('--output', '-o', 
                       help='出力ディレクトリ（指定しない場合は画面表示のみ）')
    parser.add_argument('--plot-type', '-t', 
                       choices=['basic', 'density', 'stats', 'all'],
                       default='all',
                       help='プロットの種類')
    
    args = parser.parse_args()
    
    # ファイルの存在確認
    csv_path = Path(args.csv_file)
    if not csv_path.exists():
        print(f"エラー: ファイルが見つかりません - {csv_path}")
        return
    
    # データ読み込み
    df = load_collision_data(csv_path)
    if df is None:
        return
    
    # 統計情報表示
    print_statistics(df)
    
    # 出力パスの設定
    output_dir = Path(args.output) if args.output else None
    if output_dir:
        output_dir.mkdir(exist_ok=True)
    
    # プロット作成
    if args.plot_type in ['basic', 'all']:
        output_path = output_dir / "collision_basic.png" if output_dir else None
        create_basic_plot(df, output_path)
    
    if args.plot_type in ['density', 'all']:
        output_path = output_dir / "collision_density.png" if output_dir else None
        create_density_plot(df, output_path)
    
    if args.plot_type in ['stats', 'all']:
        output_path = output_dir / "collision_statistics.png" if output_dir else None
        create_statistics_plot(df, output_path)

if __name__ == "__main__":
    main()