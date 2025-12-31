# kadai2
  ロボットシステム学課題2

   ![test](https://github.com/snake0227/ros2_kadai/actions/workflows/test.yml/badge.svg)
# system_monitor_launch.py
  cpu使用率,メモリ使用率,GPU使用率,ストレージの使用率と指定したフォルダの中で容量の大きいものを上から5つ教えてくれるものです.

  使用したノード
  1. system_monitor:指定されたディレクトリパスを引数として受け取り、システム情報を定期的にパブリッシュします.
     配信トピック：system_info
     内容:CPU使用率(%), RAMの使用状況(GB), ディスクの空き容量(GB), および指定ディレクトリ内のファイルサイズTOP5

  2.  system_info_sub
      sytem_infoを受け取り、標準室力に表示するサブスクライバ.
 
# ダウンロード
  pip install psutil --break-system-packages 
  
  git clone https://github.com/snake0227/ros2_kadai

# 使い方
  ros2 launch kadai system_monitor_launch.py path:=<ディレクトリのパス>
# 実行環境
  Ubuntu 24.04
  Python3.7~3.10

# ライセンス
  -このソフトウェアパッケージは、3条項BSDライセンスの下、再頒布および使用が許可されます.
  -このパッケージは、Ryuichi Ueda由来のコード(© 2025 ryuichiueda)を利用しています.
  -このパッケージのコードは、下記のスライド(CC-BY-SA 4.0 by Ryuichi Ueda)のものを、本人の許可を得て自身が改変・作成したものです.
    -[ryuichiueda/my_slides robosys_2025] (https://github.com/ryuichiueda/slides_marp/tree/master/prob_robotics_2025)

  -©　2025 Daichi Utsugi
