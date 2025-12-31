#!/bin/bash -xv
# (C) 2025 snake
# SPDX-License-Identifier: BSD-3-Clause

ng(){
    echo "${1}行目が違う"
    res=1
}

res=0
dir=${1:-~}

cd "$dir/ros2_ws" || ng "$LINENO"
source install/setup.bash || ng "$LINENO"

#正常
echo "Test 1: Normal execution"
timeout 30s ros2 run kadai disk_monitor /tmp > /tmp/disk_test.log 2>&1 &
PID=$!

sleep 5


timeout 5s ros2 topic echo /system_info --once || ng "$LINENO"


kill $PID 2>/dev/null
#異常（存在しないパス）
echo "Test 2: Invalid path"
# 存在しないパスを指定。終了コード 1 を期待
ros2 run kadai disk_monitor /non_existent_path_999 > /dev/null 2>&1
[ "$?" = 1 ] || ng "$LINENO"

#異常（引数なし）
echo "Test 3: No arguments"
# 引数が足りない場合もエラー終了 1 を期待
ros2 run kadai disk_monitor > /dev/null 2>&1
[ "$?" = 1 ] || ng "$LINENO"

[ "$res" = 0 ] && echo OK

exit $res
