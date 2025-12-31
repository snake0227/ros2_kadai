#!/bin/bash -xv
# (C) 2025 snake
# SPDX-License-Identifier: BSD-3-Clause

# エラー時に行番号を表示する関数
ng(){
    echo "${1}行目が違う"
    res=1
}

res=0
dir=${1:-~}

cd "$dir/ros2_ws" || ng "$LINENO"
source install/setup.bash || ng "$LINENO"

# --- テスト1: 正常系 ---
## --- テスト1: 正常系 ---
echo "Test 1: Normal execution"
# バックグラウンドで起動（15秒間動かし続けるように少し長めにする）
timeout 30s ros2 run kadai disk_monitor /tmp > /tmp/disk_test.log 2>&1 &
PID=$!

# 起動するまで少し待つ（長すぎるとtimeoutにぶつかるので、3〜5秒程度にする）
sleep 5

# トピックを確認（ノードが動いている間にキャッチする）
timeout 5s ros2 topic echo /system_info --once || ng "$LINENO"

# 終わったらPIDを確実に殺す（念のため）
kill $PID 2>/dev/null
# --- テスト2: 異常系（存在しないパス） ---
echo "Test 2: Invalid path"
# 存在しないパスを指定。終了コード 1 を期待
ros2 run kadai disk_monitor /non_existent_path_999 > /dev/null 2>&1
[ "$?" = 1 ] || ng "$LINENO"

# --- テスト3: 異常系（引数なし） ---
echo "Test 3: No arguments"
# 引数が足りない場合もエラー終了 1 を期待
ros2 run kadai disk_monitor > /dev/null 2>&1
[ "$?" = 1 ] || ng "$LINENO"

# 全てのテストを通れば OK を表示
[ "$res" = 0 ] && echo OK

exit $res
