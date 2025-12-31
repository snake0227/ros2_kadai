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

# --- テスト1: 正常系（正しいトピックが出るか） ---
echo "Test 1: Normal execution"
# バックグラウンドで起動
timeout 10s ros2 run kadai disk_monitor /mnt/c > /tmp/disk_test.log 2>&1 &
PID=$!

# 5秒待ってトピックを確認。一度もデータが来なければ ng
ros2 topic echo /disk_info --once --timeout 5 || ng "$LINENO"

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
